# Madgwick Attitude Validation Report

Milestone 3 · 2026-11-18 · **Simulation and host testing only. No hardware, no flight data. Nothing
here demonstrates flight readiness or safety.**

## 1. Baseline (before any change)

| Check | Result |
|---|---|
| Git | branch `Pre-processing` (HEAD `c26ff8c`, merged into `main` as PR #1 `8602405`), working tree clean, no stash; user `alymasani` |
| SoarOS | `426c1efe895bb58ba2a57ea128c9aad642e9be9f`, branch `master`, clean, up to date with `origin/master` |
| Host tests | **205/205** passed |
| CubeIDE Debug (headless, scratch copy) | 0 errors, 0 warnings; text 61552, data 104, bss 45208 |
| CubeIDE Release | 0 errors, 0 warnings; text 37744, data 104, bss 45200 |

## 2. Test counts

| | Count |
|---|---|
| Existing tests (unchanged files) | 205 |
| New attitude tests | **73** |
| Final total | **278** |

New tests by file:

| File | Tests | Covers |
|---|---|---|
| `test_attitude_math.cpp` | 24 | quaternion conventions vs independent matrices, ENU/NED, Euler, normalisation, gradient derivations (finite differences), pure gyro integration, gravity compensation (all 10 brief cases + free fall + sensitivity) |
| `test_attitude_estimator.cpp` | 22 | state machine, levelling, motion during init, duplicates, out-of-order, skipped samples, gaps (short/long), wraparound, invalid config, NaN/stale gyro, invalid accel, inherited flags, magnetometer policy (8 tests) |
| `test_attitude_scenarios.cpp` | 20 | the 17 brief scenarios + simulator self-check + 3 sensitivity sweeps |
| `test_attitude_integration.cpp` | 7 | end-to-end through `PreprocessingPipeline`, republished frames, faulty IMU, moving at start-up, uncalibrated magnetometer, frame adapter, mailbox concurrency |

### Runs executed (final code)

| Configuration | Result |
|---|---|
| no sanitizer (`SANITIZE=0`) | 278/278 passed |
| AddressSanitizer + UndefinedBehaviorSanitizer (default) | 278/278 passed, no reports |
| ThreadSanitizer (`SANITIZE=thread`) | 278/278 passed, 0 warnings (incl. the mailbox writer/reader test) |
| Compiler | Apple clang (host), `-std=c++17 -Wall -Wextra -Wpedantic -Wshadow -Werror` |

Skipped: none. Not possible: any on-target execution (no hardware).

## 3. Independence of the expected values

`Tests/sim/AttitudeSimulator.hpp` holds the truth in **double-precision rotation matrices** propagated by the
exact Rodrigues exponential (0.5 ms sub-steps, body rates right-multiplied). Specific force is generated
physically, `f_body = R^T (a_ENU + (0,0,g))`; accelerating motion enters through `a_ENU`, never by tilting
gravity. Truth quaternions come from Shepperd's method; estimated quaternions are turned into matrices by
the simulator's own formula. Errors are reported with the brief's metric
`2·acos(clamp(|q_true·q_est|, 0, 1))` and cross-checked with the trace formula of `R_trueᵀR_est`.
Tilt error = angle between true and estimated "up" in body axes (yaw-independent); heading error from the
horizontal projection of body +x. The coordinated-turn truth is itself checked (`att_sim_12b`: lateral
specific force < 0.02 m/s², ‖f‖ = g/cos φ within 0.02 m/s²).

One simulator bug was found and fixed during development: the default truth was the identity matrix, which
as `R_ENU_from_body` describes an **upside-down** FRD body (body z = down pointing up). The filter levelled
correctly to roll 180°, which exposed it. Default is now level with yaw 0.

Sensor model "realistic": residual gyro bias (2, −1.5, 1)·10⁻⁴ rad/s (Milestone 2 calibration error), gyro
noise 0.002 rad/s, accelerometer noise 0.02 m/s², magnetometer noise 0.2 µT, 100 Hz IMU, 50 Hz magnetometer.

## 4. Orientation accuracy (default configuration)

Provisional development targets: stationary < 1°, controlled 90° rotation < 2° (final), stationary gravity
compensation < 0.2 m/s². **All met.**

| # | Scenario | Result |
|---|---|---|
| 1 | Stationary level, 30 s | max 0.333°, RMS 0.155°; ‖a_lin‖ max 0.088, RMS 0.028 m/s² |
| 2 | Stationary roll +45° / −120° | max 0.319° / 0.316° |
| 3 | Stationary pitch −30° / +60° / roll 20° pitch 75° | max 0.264° / 0.256° / 0.239° |
| 4 | 90° roll in 1 s | final **0.121°** (max during 0.960°) |
| 5 | 90° pitch in 1 s (ends at gimbal lock) | final **0.076°** (max 0.872°) |
| 6 | 90° yaw in 2 s, 6-axis | final **0.141°** (max 0.490°) |
| 7 | Multi-axis rotation vector (60, −40, 120)° in 3 s | final **0.101°** (max 0.518°) |
| 8 | Uncompensated bias 0.01 rad/s per axis, 120 s | tilt max 0.343° (gravity bounds it); yaw drift **68.5°** (unobservable, expected) |
| 9 | 20 % samples dropped + 5 % invalid accel during rotation | final 0.160° |
| 10 | Irregular dt 5–17 ms during rotation | final 0.119°, max 0.659° |
| 16 | 10 min 6-axis with repeated manoeuvres | tilt max 0.344°; yaw drift 4.3° (unreferenced) |
| — | Levelling at roll 30° / pitch −20° | tilt error 0.109° |
| — | 90° yaw with dt = 30 ms (every third sample) | final 0.011° |
| — | Rotation across the uint32 millisecond wrap | final 0.012°, 0 gaps, 0 out-of-order |
| E2E | Through the preprocessing pipeline (per-IMU biases calibrated by Milestone 2), 90° roll | final 0.065°, max 0.900° |

Max-during-manoeuvre values (0.5–1°) come from the turn gate: corrections pause while rotating and the
first-order integration of noisy, residually biased rates accumulates a small error that is removed after
the manoeuvre.

## 5. Gravity compensation

| Case | Result |
|---|---|
| Level rest, rolled 90°, pitched 90° | 0 within 1e-4 m/s² |
| +2 / −2 m/s² vertical at 20 random attitudes each | exact within 1e-4; matches independent `R f + (0,0,−g)` |
| Coordinated turn 30° (simulation assumption: sensor at rotation centre) | (g tan 30°, 0, 0) east, vertical 0 |
| Pure yaw spin at centre | 0 |
| NaN / zero quaternion | rejected, NaN outputs |
| ‖q‖ = 1.005 / 1.2 | renormalised / rejected |
| Invalid accel, orientation unavailable | invalid (NaN), not 0 |
| Degraded attitude | valid, not reliable |
| Free fall (f = 0) | vertical −9.80665 |
| Stationary in simulation (S1) | ‖a_lin‖ max 0.088 m/s² |
| 1.5 s free fall in simulation (S15) | vertical error max 0.054 m/s², reads −g; accel corrections rejected (magnitude) throughout; tilt max 0.269° |
| Sensitivity | 1° tilt error → horizontal 0.171 m/s², vertical 0.0015 m/s²; +0.05 m/s² bias → −0.05 m/s² vertical (body z down) |

## 6. Filter gain sensitivity (β)

Columns: (a) limit cycle, noise-free with a 0.002 rad/s bias; (b) tilt RMS, stationary with gyro noise
0.01 rad/s and accel noise 0.1 m/s²; (c) tilt max with an uncompensated 0.05 rad/s bias; (d) 30° turn max
tilt with all gates; (e) same turn with no gates.

| β (rad/s) | (a) | (b) | (c) | (d) | (e) |
|---|---|---|---|---|---|
| 0.010 | 0.012° | 0.103° | **24.31°** | 0.46° | 4.36° |
| 0.033 | 0.038° | 0.146° | 0.07° | 0.87° | 12.84° |
| **0.100 (default)** | 0.115° | 0.238° | 0.12° | 1.16° | 27.19° |
| 0.300 | 0.345° | 0.427° | 0.36° | 1.45° | 30.50° |
| 0.500 | 0.572° | 0.576° | 0.57° | 1.31° | 30.70° |

How β acts:
- **Convergence / drift cancellation**: the estimate moves at 2β rad/s towards gravity, so β must exceed half
  the gyro error to be cancelled: β = 0.01 (2β = 0.02 < 0.05) loses the tilt (24°).
- **Noise**: every accepted step moves 2β·dt in a noisy direction; the limit cycle equals 2β·dt (0.115° at
  0.1, matches theory exactly) and noise-induced tilt grows with β.
- **Sustained acceleration**: without gates, larger β follows the false vertical faster (4° → 31°). With
  gates the dependence is weak (0.5–1.5°).
- **Choice**: 0.1. 0.033 is slightly better in these simulations, but the simulated gyro has no scale-factor
  error or thermal drift, and with the turn gate corrections are accepted only part of the time (4 % in the
  turbulence run), so the effective correction authority is 2β × acceptance fraction; 0.1 keeps margin. β = 0.5
  (earlier proposal) is not adopted: no benefit, 5× the limit cycle. **Neither value is validated for
  paragliding.**

## 7. Dynamic acceleration and rejection

### 7.1 Angle-gate sweep (turn gate disabled, β = 0.1)

| Angle gate | 30° turn max tilt | 15° turn max tilt | Turbulence tilt max / RMS |
|---|---|---|---|
| 5° | 12.80° | 15.27° | 10.62° / 7.28° |
| 10° | 20.05° | 15.27° | 9.07° / 4.78° |
| 20° | 20.05° | 15.27° | 10.66° / 5.27° |
| none (180°, no magnitude gate) | 27.19° | 15.27° | 13.08° / 6.65° |

The angle gate **alone does not protect** against coordinated turns (15° turn: full 15° error for every gate),
the finding that led to the turn gate (design §6.3).

### 7.2 Turn-gate sweep (angle gate 10°, β = 0.1)

| Rate gate | 30° turn | 15° turn | 5° turn | Turbulence tilt max / RMS | Turbulence vertical-accel RMS err | Corrections accepted |
|---|---|---|---|---|---|---|
| off | 20.05° | 15.27° | 5.24° | 9.07° / 4.78° | 0.079 m/s² | 35 % |
| 0.30 rad/s | 5.79° | 15.27° | 5.24° | 9.07° / 4.78° | 0.079 | 35 % |
| 0.15 rad/s | 1.97° | 3.16° | 5.24° | 8.93° / 4.78° | 0.106 | 12 % |
| **0.10 rad/s (default)** | **1.16°** | **1.63°** | 5.24° | **4.85° / 2.46°** | **0.046** | 4 % |
| 0.05 rad/s | 0.64° | 0.67° | 1.08° | 1.86° / 1.39° | 0.032 | 0 % |

Turn: 3 s roll-in, 20 s hold, 3 s roll-out at 10 m/s, realistic noise. Turbulence: band-limited linear
acceleration ±2–3 m/s² and rotations 0.12–0.2 rad/s for 55 s. 0.05 rad/s scores best **only because the
simulated gyro is nearly perfect**: it rejects every correction in turbulence (pure gyro), which with a real,
drifting gyro would accumulate error without bound. 0.10 is chosen (same as the stationary detector) and
must be re-evaluated with flight data.

### 7.3 Scenario results with the defaults

| # | Scenario | Result |
|---|---|---|
| 11 | 0.4 s horizontal push 3 m/s² (physical) + one 3 g spike | 39 corrections rejected; tilt max 0.297° |
| 12 | Coordinated turn 30°, 20 s | tilt max 1.16°, final 0.21°; vertical-accel error max 0.175, RMS 0.060 m/s²; ‖a_lin‖ error max 0.267 m/s²; state reported Degraded during the turn |
| 12 | Same, all gates off | tilt max 27.19°; vertical-accel error max 1.41 m/s²; ‖a_lin‖ error max 5.07 m/s² |
| 15 | Free fall 1.5 s | see §5 |

## 8. Magnetometer tests

| Test | Result |
|---|---|
| Magnetometer unavailable, 60 s | Tracking6Axis, heading Unreferenced, tilt < 1° |
| Calibrated constant field, truth heading 70° | acquired once; Tracking9Axis, MagneticReferenced; heading error 0.417° |
| Uncalibrated + default policy | `Disabled`, 6-axis, Unreferenced (also end to end through the pipeline) |
| Uncalibrated + `AllowUncalibrated` | MagneticUnvalidated, `MagUncalibrated`, `yawReferenced = false` |
| Policy `Disabled` | never used |
| Strength ×1.5 / ×3 | `RejectedMagnitude`; heading error < 1° |
| Dip changed 20° (same strength) | `RejectedInclination`; heading error < 1° |
| 3 s field rotated 60° | `RejectedDirection`; max heading error 0.333°; accepted again after it ends |
| Persistent 60° rotation | `AcceptedRecovery` after 5 s; heading pulled 42° within 7 s (**documented trade-off**) |
| Single wild sample | rejected; q moved < 0.2° |
| Magnetometer lost 11 s | MagneticLost after 10 s, Tracking6Axis |
| S13: 9-axis 90° yaw, 4 s 45° disturbance (2 s in the turn, 2 s after) | heading error max 0.398°; 248 heading-gate rejections; 308 steps skipped during the turn (`NoGravityReference`); never accepted while disturbed |

## 9. Dropout, timing and recovery

| Test | Result |
|---|---|
| Every sample delivered twice + 10 % late (older) re-deliveries (S17) | final quaternion **bit-identical** to the clean run; 545 extra deliveries skipped |
| Frames rebuilt twice per cycle through the real pipeline | bit-identical to single build; ≥ 600 `NotNew` |
| Republished input, out-of-order input | `NotNew` / `OutOfOrder`; q unchanged |
| 200 ms gyro outage | Degraded + GapDetected; hole not integrated; Tracking again after the next accepted gravity correction |
| 130 ms hole followed by a 5 rad/s sample | `GapSkipped`, not integrated, tilt < 0.5° |
| 800 ms outage | Invalid (outputs invalid), re-levelled after 0.5 s quiet, previous yaw kept |
| S14: 300 ms gyro loss mid-roll | 50.4° error after the gap; recovery (quasi-static evidence) back below 2° and Tracking after 2.97 s; final 0.124° |
| IMU 1 gyro fault 0.3 rad/s (end to end) | voted out by preprocessing (mask 0x5); attitude error max 0.446° |
| Moving from power-on for 10 s | never a valid attitude; levels after calibration once still |
| NaN gyro, stale gyro (40 ms) | Unavailable, q stays finite |
| Invalid configuration | `InvalidConfig`, never valid |
| Mailbox, 20 000 writes vs. concurrent reads (TSan) | 0 torn, 0 backwards reads |
| Never-NaN / never-falsely-healthy monitors in every scenario | no violation |

## 10. CPU and memory

| Item | Value | Status |
|---|---|---|
| Host time per update | 76 ns (6-axis), 86 ns (9-axis); Apple M4, `-O2`, 20 000-sample runs, best of 5 | measured |
| Target time per update | 25–80 µs at 64 MHz (≈ 1600–5000 cycles; caches off; libm-dominated) | **estimate** |
| CPU share at 100 Hz | ≤ 0.8 % | estimate |
| Static RAM | +816 B (.bss 45200 → 46016 Release) | measured |
| Heap | 0; no malloc/new references in attitude objects (`nm -u`: only `memset/memcpy`, float libm) | measured |
| Stack | ≈ 0.5 KB on the SensorProc path (Release `.su`); task stack 4 KB unchanged | measured (static analysis) |
| Flash | Release +11.7 KB (own code 6.6 KB, float libm ≈ 3.5 KB, libgcc soft-double object ≈ 0.9 KB (int64→double conversion for `sinf` range reduction; rarely executed), integration 0.6 KB); Debug +19.4 KB | measured |
| Struct sizes (ARM) | `AttitudeEstimate` 164, `AttitudeEstimator` 424, `AttitudeInput` 64, `AttitudeConfig` 124, `AttitudeMailbox` 172 | measured |

## 11. Firmware build status (final code, headless CubeIDE on a scratch copy)

| Configuration | Errors | Warnings | text | data | bss |
|---|---|---|---|---|---|
| Debug | 0 | 0 | 80916 (+19364) | 104 | 46024 (+816) |
| Release | 0 | 0 | 49484 (+11740) | 104 | 46016 (+816) |

Additional strict ARM pass (`arm-none-eabi-g++ 14.3`, `-O2 -Wall -Wextra -Wshadow -Wpedantic
-Wdouble-promotion -Wconversion`) on all attitude sources, `SensorProcessingTask.cpp`,
`HealthMonitorTask.cpp`: no warning in ParaGuardian code; only SoarOS header warnings (`Command.hpp` shadow,
`Queue.hpp` conversion, `CubeDefines.hpp` sized deallocation), which are pre-existing and not modified.
No undefined symbols in the ELF. `.ioc`, `.cproject`, `Src/`, `Inc/`, drivers, acquisition and SoarOS
unchanged.

## 12. Remaining risks

1. All results are simulation-only; real IMU errors (scale factor, cross-axis, temperature drift,
   vibration, gyro saturation in violent manoeuvres) are not modelled.
2. The attitude is gyro-only for long periods in turns and turbulence; real gyro bias drift may dominate.
3. Slow turns (< 0.10 rad/s) bias the tilt by up to their bank angle.
4. Mounting matrices unverified: every output carries `MountingUnverified`.
5. Heading unreferenced (no iron calibration); persistent magnetic disturbances would be accepted after 5 s
   once a calibrated magnetometer is enabled.
6. No attitude after an in-flight reset until 5 s of stillness.
7. Target CPU time is an estimate.
8. Thresholds (β, gates, recovery) are untuned starting points.
