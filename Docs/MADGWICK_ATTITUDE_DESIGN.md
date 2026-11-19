# Madgwick Attitude Estimation: Design

Milestone 3 · 2026-11-18 · Status: **implemented, host-tested, firmware builds. NOT hardware-tested,
NOT flight-validated, NOT safety-validated.**
Code: `Components/Algorithms/Attitude/`. Results: `MADGWICK_VALIDATION_REPORT.md`. Teaching text:
`MADGWICK_ATTITUDE_WALKTHROUGH.txt`.

---

## 1. Scope

```
Sensor drivers -> SensorAcquisitionTask -> SensorDataHub -> SensorProcessingTask
   -> PreprocessingPipeline -> ProcessedSensorFrame                       (Milestone 2)
   -> attitudeInputFromFrame() -> AttitudeEstimator                       (Milestone 3)
        [MadgwickFilter: quaternion q_ENU_from_body]
        [GravityCompensator: a_ENU = R(q) f_body + (0,0,-g)]
   -> AttitudeEstimate -> AttitudeMailbox -> (future UKF / safety logic)
```

In scope: quaternion orientation (Madgwick), initialisation and recovery, measurement gating,
magnetometer policy, gravity compensation, `AttitudeEstimate`, RTOS integration, simulation validation.
Out of scope (not started): UKF, velocity/altitude integration, aerodynamic model, crash detection,
landing guidance, SOS logic.

The drivers, the acquisition task and the preprocessing pipeline are unchanged. Everything in
`Components/Algorithms/Attitude/` is portable C++17 (no HAL, no FreeRTOS, no heap) and is compiled
unchanged into the firmware and into the host tests.

## 2. Filter choice and the library reuse decision

### 2.1 Candidate: the "Infusion" (x-io Fusion) copy from the SOAR altitude filter

Inspected: `Soar/kalman filter/AltitudePredictionFilter/Infusion.cpp` + `infusion.hpp` (1850 lines).

| Aspect | Finding | Consequence |
|---|---|---|
| Origin | Modified copy of x-io Technologies' Fusion (Seb Madgwick), co-author line added | Fusion is MIT-licensed; **this copy carries no licence text**, so its provenance would have to be re-established before reuse |
| Algorithm | Fusion's *revised* AHRS: a proportional feedback of the gravity/magnetic error (dimensionless "gain", initial ramp from gain 10) | Not the original gradient-descent Madgwick filter; its "gain 0.5" is not the β of the paper |
| Units | accelerometer in g, gyroscope in deg/s | needs conversions at the boundary |
| Conventions | `EarthConventionNed` default; NWU/ENU/NED selectable | usable, but its quaternion and ours would need an adapter |
| Rejection | `accelerationRejection` / `magneticRejection` in degrees, converted to `(0.5 sin θ)^2` thresholds on the half-error vector; `recoveryTriggerPeriod` **in samples** | sample counts are wrong with variable dt |
| Code quality | `#include <string>`, `<ctime>`, `<stdio.h>`, `using namespace std`; known gravity-handling bugs in the SOAR project (gravity removed twice, see that project's review) | would have to be cleaned before entering firmware |

### 2.2 Decision

**Self-contained implementation of the original Madgwick algorithm** (Madgwick 2010 report; ICORR 2011),
~150 lines of mathematics (`MadgwickFilter.cpp`), with the policy logic (gating, recovery, timing,
magnetometer trust) in a separate class (`AttitudeEstimator.cpp`). Reasons:

1. The brief asks for the actual Madgwick algorithm; Fusion's revised update law is a different filter.
2. Our conventions (FRD body, ENU navigation, scalar-first `q_ENU_from_body`) are used natively,
   without an adapter layer that could hide a sign error.
3. Time-based (not sample-based) recovery is needed because dt varies.
4. No licence ambiguity, no dynamic-language includes, no global state.
5. The algorithm is small enough to verify line by line (gradient checked against finite differences,
   §4.4) and to explain in a design review.

Fusion's *ideas* that were kept, re-derived and documented: an acceleration-rejection **angle**, a magnetic
rejection angle, and a recovery mechanism, all re-specified in time units.

## 3. Conventions (single source of truth)

| Item | Convention |
|---|---|
| Body frame | FRD: +x forward, +y right, +z down (right-handed). Unchanged from Milestone 2 |
| Navigation frame | ENU: +x east, +y north, +z up (right-handed) |
| Quaternion storage | `struct Quaternion { float w, x, y, z; }`: **scalar first** |
| Meaning | `q = q_ENU_from_body`: `v_ENU = q ⊗ [0, v_body] ⊗ q*  =  R(q) v_body` (active rotation of the body vector into ENU coordinates) |
| Product | Hamilton, `i·j = k`. Chaining: `q_A_from_C = q_A_from_B ⊗ q_B_from_C` |
| Gyro kinematics | body rates, **right** multiplication: `dq/dt = ½ q ⊗ [0, ω_body]` |
| NED ↔ ENU | `q_ENU_from_NED = (0, 1/√2, 1/√2, 0)` = 180° about (N+E)/√2; matrix `[[0,1,0],[1,0,0],[0,0,-1]]` = `kEnuFromNed` of Milestone 2; self-inverse |
| Euler (display only) | aerospace Z-Y-X of FRD body relative to **NED**: `R_NED_from_body = Rz(yaw) Ry(pitch) Rx(roll)`; roll + = right side down, pitch + = nose up, yaw = heading clockwise from north, −180..180°. Degrees in `AttitudeEstimate::eulerDeg`. Never propagated; clamped at pitch ±90° (gimbal lock) |
| Gravity | `g_ENU = (0, 0, −g)`, g = 9.80665 m/s² (configurable) |
| Specific force | what an accelerometer measures: `f = a − g_vec`. At rest `f_ENU = (0, 0, +g)`; level FRD `f_body = (0, 0, −g)` |
| Linear acceleration | `a_ENU = R(q) f_body + (0, 0, −g)`; at rest ≈ 0, free fall ≈ (0, 0, −g) |

Every row is checked by an independent test (`test_attitude_math.cpp`, `att_quat_*`, `att_gravity_*`),
using a double-precision rotation-matrix implementation in `Tests/sim/AttitudeSimulator.hpp` that shares no
code with the production quaternion functions.

## 4. Madgwick equations

### 4.1 Gyroscope propagation
`q̇_ω = ½ q ⊗ [0, ω]` with ω the bias-corrected body rate (rad/s) from preprocessing.

### 4.2 Accelerometer objective (gravity)
Normalised measured specific force `â = f/|f|` is the body-frame "up" direction at rest. The estimate
predicts "up" in body axes as `R(q)ᵀ e_up`, the third row of R:

```
f_g(q) = R(q)^T e_up - â = [ 2(q1q3 − q0q2) − âx ]
                           [ 2(q0q1 + q2q3) − ây ]
                           [ 2(½ − q1² − q2²) − âz ]
J_g = [[−2q2,  2q3, −2q0, 2q1],
       [ 2q1,  2q0,  2q3, 2q2],
       [   0, −4q1, −4q2,   0]]
```
(q0 = w, q1 = x, q2 = y, q3 = z). Identical to Madgwick's paper because ENU, like his frame, has z up.

### 4.3 Magnetometer objective (ENU adaptation, derived here)
Madgwick's reference is `b = (b_x, 0, b_z)` in his x-north, z-up frame. In ENU, north is **+y**, so
`b = (0, b_n, b_u)`, re-derived from the measurement every step (distortion compensation):

```
h = R(q) m̂            (measured field in ENU)
b_n = sqrt(h_e² + h_n²),  b_u = h_u
f_b(q) = R(q)^T b − m̂ = b_n·(row 2 of R) + b_u·(row 3 of R) − m̂
f1 = 2b_n(q1q2 + q0q3) + 2b_u(q1q3 − q0q2) − m̂x
f2 = b_n(1 − 2q1² − 2q3²) + 2b_u(q2q3 + q0q1) − m̂y
f3 = 2b_n(q2q3 − q0q1) + b_u(1 − 2q1² − 2q2²) − m̂z
J_b rows (∂/∂q0..q3):
 f1: [ 2b_n q3 − 2b_u q2,  2b_n q2 + 2b_u q3,  2b_n q1 − 2b_u q0,  2b_n q0 + 2b_u q1]
 f2: [ 2b_u q1,           −4b_n q1 + 2b_u q0,  2b_u q3,           −4b_n q3 + 2b_u q2]
 f3: [−2b_n q1,           −2b_n q0 − 4b_u q1,  2b_n q3 − 4b_u q2,  2b_n q2          ]
```

### 4.4 Update
```
∇ = J_gᵀ f_g  (+ J_bᵀ f_b when the magnetometer is accepted)
q̇ = q̇_ω − β ∇/|∇|          (no correction if |∇| = 0, i.e. exactly at the optimum)
q ← normalise(q + q̇ dt);  sign canonicalised to w ≥ 0 (q and −q are the same rotation)
```
Verification: both gradients are compared with central finite differences of an independent objective
½|R(q/|q|)ᵀd − s|² (tangential components, 100 random cases each), and are zero at the true orientation
(`att_madgwick_*_gradient_*`). One step with β = 0.5, dt = 0.01 rotates the estimate by exactly 2β·dt
= 0.573° towards gravity (`att_madgwick_correction_step_reduces_tilt_error`).

Integration is first order (as in Madgwick's formulation). With 10 ms steps the truncation error per step
is ≈ θ³/12 (θ = |ω|dt); for a 180°/s peak it is 3·10⁻⁷ rad, negligible next to sensor errors (90° roll test:
0.02° after 100 pure-gyro steps).

## 5. How each sensor is used

| Sensor | Role | Condition |
|---|---|---|
| Gyroscope | **drives** the filter: one step per NEW gyro timestamp | valid, finite, age ≤ 30 ms, bias-calibrated (`requireCalibratedGyro`) |
| Accelerometer | gravity direction (roll/pitch) | valid, finite, age ≤ 30 ms, and all four gates of §6 pass |
| Magnetometer | heading | policy allows (§6.5), fresh (≤ 100 ms), accelerometer accepted in the same step, field checks pass |

Only the voted outputs of the preprocessing layer are used (`angularRate_radps`, `specificForce_mps2`,
`mag`), never an individual IMU directly; the contributing-IMU masks are copied into the estimate.

## 6. Gain and rejection policies

### 6.1 Parameters (all in `AttitudeConfig.hpp`, all validated by `isValid()`)

| Parameter | Default | Basis | NOT validated for flight |
|---|---|---|---|
| `beta_radps` | 0.1 | Madgwick's reference-code default; sweep §6 of the validation report: 0.1 keeps noise-induced tilt RMS 0.24° and cancels gyro errors up to 2β = 0.2 rad/s **while corrections are accepted** | ✓ |
| `recoveryBeta_radps` | 0.5 | re-converges a 50° error in ~3 s (S14) | ✓ |
| `accelRejectionDeg` | 10° | angle between measured and predicted "up"; the previously proposed value | ✓ |
| `accelMagnitudeTolerance_mps2` | 1.0 | ‖f‖ − g; rejects free fall and strong manoeuvres (≈ 0.1 g) | ✓ |
| `accelRejectionRate_radps` (turn gate) | 0.10 | §6.3; equals the stationary-detector limit | ✓ |
| `degradedAfterMs` | 2000 | report "gyro-only" after 2 s without accepted gravity correction | ✓ |
| `accelRecoveryMs` / `recoveryQuietMs` | 5000 / 1000 | previously proposed 5 s recovery period, **in time**, plus 1 s of quasi-static evidence | ✓ |
| `magPolicy` | `CalibratedOnly` | no iron calibration exists → effectively 6-axis | ✓ |
| mag field window | 20–70 µT | Earth's field ≈ 25–65 µT | ✓ |
| `magFieldTolerance`, `magInclinationToleranceDeg`, `magRejectionDeg` | 15 %, 10°, 10° | disturbance detection (§6.5) | ✓ |
| `magRecoveryMs`, `magAcquireSamples`, `headingHoldMs` | 5000, 10, 10000 | | ✓ |
| `maxIntegrationDtMs` / `maxGapMs` | 50 / 500 | 50 = preprocessing `kMaxGyroDtMs`; 500 ms of unknown rotation is treated as lost orientation | ✓ |
| `initWindowMs` / `initMinSamples` | 500 / 40 | 0.5 s of quasi-static data, ≥ 40 of 50 samples (dropout tolerant) | ✓ |

**β is not Fusion's gain.** β is a *rate* (rad/s) of the normalised gradient step; Madgwick derives
β = √(3/4)·ω̃ for gyroscope error ω̃. Because the gradient is normalised, the estimate always moves by
2β·dt per accepted step: this is both the maximum drift it can cancel (2β rad/s) and the size of the
steady-state limit cycle (0.115° at β = 0.1; measured 0.115°). **β = 0.5 is not adopted**: it gives a
0.57° limit cycle and 2.4× the noise-induced tilt of β = 0.1, with no benefit in any simulated
scenario. It is not claimed that 0.1 is optimal for paragliding.

### 6.2 Accelerometer gates (in `AttitudeEstimator::decideAccel`, evaluated in this order)
1. **Unavailable**: invalid / stale / non-finite.
2. **Magnitude**: `| ‖f‖ − g | > 1.0 m/s²` → `RejectedMagnitude` (includes free fall, ‖f‖ ≈ 0).
3. **Turn gate**: `‖ω‖ > 0.10 rad/s` → `RejectedRotation`.
4. **Direction**: angle(â, R(q)ᵀ e_up) > 10° → `RejectedDirection` unless recovery applies (§6.4).

The angle test compares a *direction* with the current estimate; the magnitude test alone would accept a
coordinated turn (‖f‖ = g/cos φ ≈ g for small φ) whose specific force is tilted by the bank angle.

### 6.3 Why the turn gate exists (found during this milestone)
In a coordinated turn the specific force lies along body −z for **any** bank angle. A filter whose estimate
is near the truth sees an angle error equal to the bank, but during a gradual roll-in every accepted
correction pulls the estimate towards "wings level", keeping the observed error *below* the gate, so the
angle gate never fires. Simulation without the turn gate: a sustained 15° turn ends with the full 15° tilt
error, a 30° turn peaks at 20°. A turn at bank φ, airspeed V rotates at ω = g·tan φ / V (0.26 rad/s at 15°,
10 m/s). Rejecting corrections while ‖ω‖ > 0.10 rad/s removes most of this (30°: 1.16°, 15°: 1.63°).
Costs, reported honestly:
- turns slower than 0.10 rad/s (≈ 6° bank at 10 m/s) still leak fully (5° turn: 5.2° error);
- during long turns and in turbulence the attitude is **gyro-only** (in simulated turbulence only 4 % of
  updates are corrected), so gyro bias stability becomes critical (§12);
- the proper fix (future) is to subtract the centripetal specific force ω × v using GNSS/airspeed velocity.

### 6.4 Recovery (forced acceptance)
Outside the angle gate, a correction is still accepted (`AcceptedRecovery`, gain `recoveryBeta`) only if
- the motion has been **quasi-static** for `recoveryQuietMs` (‖ω‖ ≤ 0.10 rad/s and |‖f‖ − g| ≤ 0.3 m/s²
  continuously), **and**
- either a gap made the estimate suspect (`GapDetected`), a recovery is already running, or no correction
  has been accepted for `accelRecoveryMs` (5 s).

A pure timer (Fusion style) would, in a sustained spiral, align "up" with the apparent vertical: an error of
the full bank angle. Requiring quasi-static evidence prevents this. Recovery ends when the error is below
half the gate or the motion stops being quasi-static. Residual risk: a sustained, non-rotating linear
acceleration with ‖f‖ ≈ g for > 1 s is indistinguishable from a tilted gravity vector.

### 6.5 Magnetometer policy and disturbance handling
Separate notions, each visible in the output:
| Notion | Where |
|---|---|
| measurement valid | `AttitudeInput::magValid` (preprocessing: fresh, accepted) |
| calibration valid | `magCalibrated` (from `frame.mag.ironCalibrated`: **always false today**) |
| correction accepted | `AttitudeEstimate::magCorrection` (Accepted / Rejected… / Disabled / NoGravityReference / Acquiring) |
| heading reliable | `heading == MagneticReferenced` → `yawReferenced = true` |

Policies: `Disabled`; `CalibratedOnly` (default: with today's data the magnetometer is never used);
`AllowUncalibrated` (experimental; heading reported as `MagneticUnvalidated`, never "referenced").

Disturbance detection (each step, `decideMag`):
1. ‖m‖ outside 20–70 µT → `RejectedMagnitude`.
2. **Acquisition** (first use after levelling): 10 *distinct* consistent samples (±15 % strength), then one
   yaw-only rotation of q so the measured horizontal field points north, and the reference strength and
   dip are stored. Documented discontinuity: the heading was undefined before.
3. ‖m‖ changed > 15 % from the reference → `RejectedMagnitude`; dip changed > 10° → `RejectedInclination`.
4. Heading disagreement > 10° → `RejectedDirection`; if a **self-consistent** disagreement (strength and
   dip unchanged) persists for 5 s → `AcceptedRecovery` (gyro heading is assumed wrong).
5. Skipped (`NoGravityReference`) whenever the accelerometer was not accepted in the same step: the Madgwick
   magnetometer gradient also tilts roll/pitch when the field has a dip angle, so it is never applied without
   gravity holding the tilt.

A single corrupted sample is rejected by (1), (3) or (4) and moves q by < 0.2° (test). A persistent, self-
consistent disturbance (e.g. a magnet fixed near the sensor) is accepted after 5 s and pulls the heading
(42° of a 60° disturbance after 7 s in simulation): the inherent trade-off of rule 4.
No declination is applied: "north" is **magnetic** north.

## 7. Initialisation and state machine

```
            reset()
               |
         [Uninitialized] --- no calibrated gyro / no accel ---(stay)
               | gyro (calibrated) + accel valid
               v
         [Initializing]  quasi-static window: |w| <= 0.10 rad/s, ||f|-g| <= 0.6 m/s^2,
               |          >= 40 samples over >= 500 ms, no hole > 50 ms; motion restarts it
               | level(): roll = atan2(-fy,-fz), pitch = atan2(fx, sqrt(fy^2+fz^2)), yaw = 0 (pseudo-north)
               v                                       or previous yaw after a re-initialisation
   +--> [Tracking6Axis] <------> [Tracking9Axis]   (mag accepted within 10 s and acquired)
   |        |    ^                    |
   |        v    | accepted accel     v
   |     [Degraded]  no accepted accel correction for 2 s, recovery running, or gap not yet confirmed
   |        |
   |        | no gyro for > 500 ms, or numerical failure
   |        v
   +---- [Invalid]  orientation discarded (outputs invalid); re-levelled by the same quasi-static window
```

- Yaw cannot be determined from the accelerometer (gravity is invariant under rotation about the vertical).
  Without a magnetometer the first yaw is **0 = body x towards "pseudo-north"**, flagged
  `HeadingUnreferenced`; after a re-initialisation the previous yaw is kept (best guess, still unreferenced).
- Gyro calibration (preprocessing, 5 s still) must be complete: if the wearer moves from power-on, the
  estimator stays `Uninitialized` with `GyroNotCalibrated` (no attitude is manufactured). Test:
  `att_e2e_moving_at_startup_never_fabricates_attitude`.
- "Quasi-static" is not "on the ground": steady straight flight satisfies it, which is what allows
  re-levelling in the air after an invalidation.
- Quaternion discontinuities happen only at: levelling, heading acquisition (yaw only), re-initialisation
  after Invalid. Each is counted (`initializations`, `headingAcquisitions`, `invalidations`).

## 8. Timestamp handling

- Time base: the gyro timestamp of the voted angular rate (`frame.angularRate_radps.timestampMs`, MCU ms,
  read-completion time). `dt = gyroTs − lastGyroTs` (wrap-safe signed difference, Milestone 2 `timebase`).
- One estimator step per frame; integration only if dt > 0:

| dt | Action | Status |
|---|---|---|
| = 0 (same sample republished, e.g. 50 ms idle-timeout frame) | nothing | `NotNew` |
| < 0 | nothing | `OutOfOrder` |
| 1..50 ms | integrate with this dt (skipped samples → larger dt, still exact in total) | `Integrated` |
| 51..500 ms | **not** integrated (rates inside unknown); resume from this sample; `GapDetected`, Degraded until an accepted gravity correction | `GapSkipped` |
| > 500 ms (or no gyro for > 500 ms) | orientation discarded → Invalid → re-level | gap counter, `invalidations` |

- Asynchronous sensors: accelerometer and magnetometer are used if fresh; the freshest sample is used as is
  (no interpolation, no fabrication). The 50 Hz magnetometer can be ≤ 20 ms older than the gyro sample; at
  0.3 rad/s that is ≤ 0.35° of heading, below the 10° gate and the magnetometer's calibration uncertainty.
  Heading acquisition counts only distinct magnetometer timestamps (each 50 Hz sample is seen by two 100 Hz
  gyro steps).
- Barometer / GNSS / LiDAR messages never trigger integration: the estimator only reacts to a newer gyro
  timestamp, so the ~575 messages/s entering the processing task produce at most one attitude step per IMU
  sample.
- Timestamp quantisation (1 ms) makes individual dt values ±1 ms, but the sum of dt equals the true elapsed
  time, so the integrated angle is unbiased (irregular-dt test: 5–17 ms spacing, 0.12° final error).

## 9. Gravity compensation

`GravityCompensator::compute()`: `a_ENU = R(q) f_body + (0, 0, −g)`, `vertical = a_ENU.z` (positive up).

| Input condition | Output |
|---|---|
| orientation not valid, accel not valid, non-finite input | `valid = false`, all values **NaN** (never 0) |
| ‖q‖ within 1 ± 0.01 | renormalised (float round-off) |
| ‖q‖ further from 1 | rejected (corruption is not repaired) |
| estimator Degraded | `valid = true`, `reliable = false` (+ `LinearAccelUnreliable`) |
| estimator Tracking6Axis/9Axis | `valid = reliable = true` |

Heading does not affect the vertical component, so 6-axis tracking is sufficient for vertical acceleration.
Sensitivity (from tests): tilt error δ → horizontal error g·sin δ (0.171 m/s² per degree), vertical error
g(1 − cos δ) (0.0015 m/s² at 1°); accelerometer bias appears 1:1 (rotated) and is not removed, because no
accelerometer calibration exists yet (`AccelNotCalibrated`). Nothing is integrated to velocity or altitude.

## 10. Error and confidence semantics

`AttitudeEstimate` (164 bytes, trivially copyable): quaternion (authoritative), `orientationValid`, Euler
(display), `yawReferenced`, `linearAccel {enu, vertical, valid, reliable, timestampMs}`, `state`, `heading`,
`dtMs`, `gyroIntegration`, `accelCorrection`, `magCorrection`, diagnostic angles, `betaUsed_radps`, ages,
time since last accepted corrections, source masks, `qualityFlags` (17 bits, `attitude::*`), and
saturating lifetime counters.

Rules: a value is meaningful only if its flag says so; invalid orientation ⇒ q and Euler are NaN, linear
acceleration invalid; "finite" never implies "valid". Inherited preprocessing facts are re-exported
(`MountingUnverified`, `AccelNotCalibrated`, `GyroNotCalibrated`, `ImuRedundancyDegraded`), so a consumer of
the attitude alone still sees them.

## 11. FreeRTOS integration, memory and timing

### 11.1 Option chosen: inside SensorProcessingTask (option A)

| Criterion | A: in SensorProcessingTask | B: separate AttitudeTask |
|---|---|---|
| Memory | +600 B in the task object | + stack (≥ 1 KB) + TCB + queue of frames (≥ 2 × 420 B) |
| CPU | one update per frame (≈ 25–80 µs est.) | same + queue copies + context switches |
| Latency | attitude published in the same wake as the frame | one extra scheduling hop |
| Freshness | always the newest frame | queue may hold old frames (or mailbox polling misses frames) |
| Priorities | unchanged (32 < acquisition 40) | a new priority to justify |
| Ownership | one writer of both mailboxes | two |

`SensorProcessingTask::Run()`: `buildFrame → frames_.publish → attitude_.update(attitudeInputFromFrame(work_))
→ attitudeOut_.publish(...)`. Readers use `SensorProcessingTask::Inst().attitude().read(est)`
(critical-section copy, never blocks the producer). The acquisition task is untouched and is never blocked.
`HealthMonitorTask` copies the newest estimate into `g_systemHealth` (`attitudeState`, `headingReference`,
roll/pitch/yaw in centi-degrees, vertical acceleration in mm/s², counters; `INT32_MIN` = invalid).

### 11.2 Memory (STM32, measured from the ELF)

| Item | Bytes |
|---|---|
| `AttitudeEstimator` (incl. 124-byte config copy, estimate, filter) | 424 |
| `AttitudeMailbox` | 172 |
| SensorProcessingTask object | 16144 → 16744 (+600) |
| HealthMonitorTask object (`attitudeCopy_`) | 2624 → 2788 (+164) |
| `g_systemHealth` | 532 → 588 (+56) |
| **Total static RAM (.bss)** | **+816** (Release 45200 → 46016) |
| Heap | 0 (no allocation anywhere in the attitude code; `nm` shows no malloc/new references) |
| Stack (Release `-fstack-usage`) | worst path Run 208 + update 40 + stepTracking 80 + Madgwick update 104 + magGradient 48 ≈ 0.5 KB (+ libm leaf), sequential to the preprocessing path; 4 KB task stack unchanged |
| Flash (Release text) | 37744 → 49484 (+11740): own code 6608, float libm (`sinf`, `cosf`, `asinf`, `acosf`, `atan2f` + tables) ≈ 3.5 KB, libgcc soft-double object pulled in by `sinf`'s range reduction (`__aeabi_l2d` shares one 888-byte object with soft add/sub) ≈ 0.9 KB (rarely executed), task/health growth 602 |

### 11.3 Timing

Per update (6-axis, accel accepted): ~180 multiply/add for the gradient and quaternion step, ~60 for
rotations, ~100 for checks and Euler; ~10 square roots, ~6 divisions, `acosf` (accel angle), `asinf` +
2 × `atan2f` (Euler), + 2 `atan2f` with the magnetometer. `sinf`/`cosf` only at levelling and heading
acquisition.
- **Host, measured**: 76 ns (6-axis) / 86 ns (9-axis) per update, Apple M4, `-O2`.
- **Target, estimated, NOT measured**: ≈ 1600–5000 cycles → **25–80 µs at 64 MHz** with caches disabled
  and flash wait states (libm calls dominate). ≤ 0.8 % of the 10 ms period; theoretical maximum update rate
  > 10 kHz, far above the 100 Hz IMU rate. To be measured with a GPIO toggle (HARDWARE_INTEGRATION_TODO §13).
- Bounded work: no loops depend on data except the fixed 4-element gradient loops.

## 12. Known limitations

1. **No hardware, no flight data.** Every number is from simulation; mountings are unverified placeholders.
2. **Accelerometer-aided attitude in sustained manoeuvres**: during turns and turbulence the accelerometer is
   mostly gated out and the attitude is gyro-only; turns slower than 0.10 rad/s leak their bank angle into
   the tilt estimate. Fix: centripetal compensation with GNSS velocity (future).
3. **Gyro bias stability decides gyro-only accuracy** (residual 2·10⁻⁴ rad/s in simulation → 0.7°/min; a
   thermally drifting 0.005 rad/s bias → 17°/min). Unknown until measured.
4. **Yaw is unreferenced** with today's data (no iron calibration): it drifts (simulation: 4.3° in 10 min with
   the residual bias, 68° in 2 min with a 0.01 rad/s bias) and starts at an arbitrary value.
5. **Persistent magnetic disturbances** are accepted after 5 s (design trade-off, §6.5). Heading is magnetic.
6. **In-flight restart**: no gyro calibration without 5 s of stillness → no attitude after a reset in flight.
7. Euler angles are degenerate at pitch ±90° (display only; the quaternion is unaffected).
8. Accelerometer bias is not calibrated and enters linear acceleration 1:1.
9. Timestamps are read-completion times at 1 ms resolution; sensor latencies are assumed zero.
10. Target timing is an estimate.

## 13. Files

| File | Content |
|---|---|
| `Components/Algorithms/Attitude/Quaternion.hpp` | quaternion type and maths, conventions, Euler |
| `.../MadgwickFilter.hpp/.cpp` | Madgwick step and gradients |
| `.../AttitudeEstimator.hpp/.cpp` | state machine, timing, gates, recovery, magnetometer policy, output |
| `.../GravityCompensator.hpp/.cpp` | linear acceleration |
| `.../AttitudeConfig.hpp` | parameters + `isValid` |
| `.../AttitudeTypes.hpp` | enums, flags, `AttitudeInput`, `LinearAcceleration`, `AttitudeEstimate` |
| `.../AttitudeFrameAdapter.hpp` | frame → input |
| `.../AttitudeMailbox.hpp` | latest-value hand-over |
| `Components/Tasks/SensorProcessingTask.*`, `HealthMonitorTask.*`, `SystemHealth.hpp`, `TaskConfig.hpp` | integration |
| `Tests/sim/AttitudeSimulator.hpp` | independent truth + sensor model |
| `Tests/unit/test_attitude_{math,estimator,scenarios,integration}.cpp` | 73 tests |
