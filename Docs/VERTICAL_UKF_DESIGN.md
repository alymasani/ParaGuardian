# Vertical UKF: Design (Milestone 4)

**Status:** implemented, integrated into `SensorProcessingTask`, validated in **host simulation only**.
No PCB, sensor or flight data exist yet. Nothing here is hardware-validated, flight-validated or
safety-validated. The process model is a **temporary placeholder** and is reported as such in every
output (`navflag::ModelPlaceholder`).

Related: `VERTICAL_UKF_VALIDATION_REPORT.md` (results), `VERTICAL_UKF_WALKTHROUGH.txt` (teaching
text), `VERTICAL_UKF_GIT_SUMMARY.md` (files), `MADGWICK_ATTITUDE_DESIGN.md` (input acceleration),
`SENSOR_PREPROCESSING_DESIGN.md` (barometer vote, GNSS pass-through).

---

## 1. What this milestone delivers

| Delivered | Not delivered (next packages) |
|---|---|
| Generic fixed-size UKF engine `UnscentedKalmanFilter<N>` | Paraglider aerodynamic process model |
| Replaceable process model `IVerticalProcessModel` + placeholder `ConstantAccelerationModel` | Flight-state detection, landing guidance |
| `VerticalNavigationEstimator`: barometer, acceleration and optional GNSS-velocity fusion, timing, initialisation, recovery, diagnostics | Crash detection, SOS, BLE, audio, logging |
| `NavigationEstimate` + `NavigationMailbox`: the interface for safety consumers | Automatic fallback switching to the baseline |
| `BaroAlphaBetaEstimator`: independent barometer-only baseline | GNSS altitude fusion (needs datum alignment) |
| Independent linear-KF reference, vertical simulator, 30 scenarios, NEES/NIS, sweeps | LiDAR terrain clearance |

---

## 2. Architecture

```
SensorAcquisitionTask (prio 40) --hub--> queue --> SensorProcessingTask (prio 32), one wake:
    PreprocessingPipeline::ingest()/buildFrame()  -> ProcessedSensorFrame   -> ProcessedFrameMailbox
    AttitudeEstimator::update(attitudeInputFromFrame(frame))              -> AttitudeMailbox
    VerticalNavigationEstimator::update(navigationInputFrom(frame, att))  -> NavigationMailbox   (NEW)
```

Three separated responsibilities, as required:

| Layer | Class / file | Knows about | Does NOT know about |
|---|---|---|---|
| **A. UKF engine** | `UnscentedKalmanFilter<N>` (`UnscentedKalmanFilter.hpp`), maths in `NavMath.hpp` | sigma points, weights, predict, evaluate/apply, Cholesky, finiteness | altitude, sensors, paragliders, time stamps |
| **B. Process model** | `IVerticalProcessModel`, `ConstantAccelerationModel` | how `[h, vz, az]` evolves over `dt`, its process noise | sensors, the engine's internals |
| **C. Navigation integration** | `VerticalNavigationEstimator` | sample identity, timing, noise selection, gating, initialisation, state machine, output | the model's equations (calls it only through B) |

Safety consumers read **only** `NavigationEstimate` (through `NavigationMailbox`), never the engine
or the model.

### Files (all in `Components/Algorithms/Navigation/`)

| File | Role |
|---|---|
| `NavMath.hpp` | `Mat<R,C>`, `Vec<N>`, multiply/transpose/add, `cholesky`, `spdInverse`, `symmetrize`, finiteness |
| `UnscentedKalmanFilter.hpp` | engine (template on N; header-only) |
| `IVerticalProcessModel.hpp` | the replacement boundary (3-state) |
| `ConstantAccelerationModel.hpp/.cpp` | placeholder model + `fromConfig()` |
| `NavigationTypes.hpp` | `NavigationInput`, `NavigationEstimate`, enums, `navflag::*` |
| `NavigationConfig.hpp` | every tunable number + `isValid()` |
| `BaroSampleTracker.hpp` | "is this barometer vote a new observation?" |
| `VerticalNavigationEstimator.hpp/.cpp` | layer C |
| `NavigationInputAdapter.hpp` | `ProcessedSensorFrame` + `AttitudeEstimate` → `NavigationInput` (mapping only) |
| `NavigationMailbox.hpp` | latest-value hand-over |
| `BaroAlphaBetaEstimator.hpp/.cpp` | barometer-only baseline (comparison / possible fallback) |

---

## 3. Which process model, and why

The brief required inspecting the repository for existing paraglider dynamics before choosing.

**Searched** (whole repository except the vendor `Drivers/` and the `SoarOS` submodule; sources, docs,
scripts): `polar`, `aerodyn`, `kalman`, `ukf`, `process model`, `sink rate`, `glide ratio`,
`lift coeff`, `drag`. Also searched for data/scripts (`*.py`, `*.m`, `*.ipynb`, `*.csv`, `*.mat`):
none exist.

**Found:** only mentions that a UKF "will come later" (`Components/Algorithms/README.md`,
`ProcessedSensorFrame.hpp`, `SENSOR_PREPROCESSING_*`, `MADGWICK_*`). There is **no** speed polar, no
lift/drag coefficients, no vertical-motion predictor and no UKF interface in ParaGuardian. The SOAR
rocket altitude filter (a different project, with a rocket flight-simulation "scenario" model) was
deliberately **not** used: it models a rocket, not a paraglider.

**Decision:** the documented three-state constant-acceleration model with white-jerk process noise
is the temporary placeholder (section 5). It is linear, which has a useful side effect: an ordinary
Kalman filter is mathematically equivalent, so the UKF can be verified exactly (validation report §4).
It is **not** an aerodynamic model and no accuracy for paraglider dynamics is claimed.

---

## 4. State and conventions

`x = [h, vz, az]ᵀ`

| | Meaning | Unit | Sign |
|---|---|---|---|
| `h` | altitude **relative to the barometric reference** (where the barometers were calibrated) | m | up |
| `vz` | vertical velocity | m/s | up |
| `az` | vertical **linear** (gravity-free) acceleration | m/s² | up |

* `h` is not height above ground. Terrain clearance is the LiDAR's job (future landing package).
* `az` is measured by the attitude estimator's `linearAccel.vertical_mps2` (ENU z, positive up,
  gravity already removed by `GravityCompensator`). Gravity is **not** removed a second time.
* GNSS vertical velocity arrives positive up: the NEO-M9N driver converts NAV-PVT `velD` (positive
  **down**) with a minus sign (`NeoM9n.cpp:68`). The adapter passes it on unchanged; a test checks the
  sign end to end.
* Time: MCU milliseconds (HAL tick), all comparisons wrap-safe (`timebase::diffMs`).

---

## 5. The placeholder process model

```
h'  = h + vz·dt + ½·az·dt²
vz' = vz + az·dt
az' = az
```

### 5.1 Process noise

Two independent white-noise inputs:

```
Q(dt) = qj · [dt⁵/20  dt⁴/8  dt³/6]   +   qa · [dt³/3  dt²/2  0]
             [dt⁴/8   dt³/3  dt²/2]            [dt²/2  dt     0]
             [dt³/6   dt²/2  dt   ]            [0      0      0]
```

* **qj** (m²/s⁵): white **jerk** driving `az`. Derivation: `Q = ∫₀^dt Φ(t) G qj Gᵀ Φ(t)ᵀ dt` with
  `G = [0 0 1]ᵀ`, `Φ(t)G = [t²/2, t, 1]ᵀ`. Verified independently by Simpson integration in double
  precision for dt from 1 ms to 1 s (`nav_white_jerk_Q_matches_independent_numerical_integration`).
  Meaning: over 1 s without measurements, the `az` standard deviation grows by `√qj`.
* **qa** (m²/s³): white **acceleration acting on h and vz that the `az` state does not contain**
  (same integral with `G = [0 1 0]ᵀ`, `Φ(t)G = [t, 1, 0]ᵀ`). This term was **added because simulation
  showed it is needed**:
  `az` is measured at 100 Hz, so with qa = 0 the filter treats the accelerometer's error as white noise
  that averages out and claims a vertical-velocity σ of about 4 cm/s. Real accelerometer error is not
  white: there is an uncalibrated bias of up to ~0.2 m/s² (`AccelNotCalibrated`), attitude error, and
  model error. With a model that was even slightly wrong, `vz` drifted by metres per second while the
  filter stayed confident and gated out the barometer that should have corrected it (validation report
  §6.1). qa keeps `vz` correctable by the barometer. It is model-specific, so it lives in the model.

### 5.2 Variable dt

`dt` is always the real time between the filter time and the sample time (section 8). Q depends on dt
exactly (no fixed-rate assumption). Long intervals are split into steps of at most `maxPredictStepMs`
(100 ms). The linear model doesn't need this, but a future nonlinear model will.

### 5.3 Limitations of the placeholder

* It knows nothing about the wing: no sink-rate polar, no trim speed, no relation between bank angle and
  sink, no air-mass (thermal) model. In thermals and turbulence it simply follows the measured
  acceleration. It does this well (validation §7) because `az` is measured, not predicted.
* A constant accelerometer bias is not a state. `vz` is biased by about bias × 1.3 s (S17: +0.27 m/s
  for 0.2 m/s²). Here the barometer-only baseline is better.
* Barometric errors that are correlated in time (airflow, weather) are treated as white: altitude σ
  is over-confident under realistic errors (validation §8).

---

## 6. The replacement boundary

### 6.1 Interface

```cpp
class IVerticalProcessModel {
public:
    virtual ProcessModelInfo info() const = 0;                       // id + version, reported in every estimate
    virtual VerticalStateVec propagate(const VerticalStateVec& x, float dt_s,
                                       const ProcessModelInput& u) const = 0;   // may be NONLINEAR
    virtual void processNoise(float dt_s, const VerticalStateVec& x,
                              const ProcessModelInput& u, VerticalCov& Q) const = 0;
    virtual bool parametersValid() const = 0;
protected:
    ~IVerticalProcessModel() = default;   // never deleted through the base: no heap, no operator delete
};
```

* `ProcessModelInput` carries **optional model inputs** (time stamp, bank and pitch angle from the
  attitude, attitude validity). The placeholder ignores them; an aerodynamic model would read bank
  angle, for example. Adding fields here changes neither the engine nor any consumer.
* **Model-specific parameters** (qj, qa today; polar coefficients later) belong to the model object.
* **Process-noise assumptions** are the model's (`processNoise`), not the engine's.
* Why a virtual interface and not a template policy: the engine **is** compile-time generic (any
  object with `propagate`/`processNoise` works), but the production estimator holds the model by
  reference so a model can be swapped by constructing the estimator with a different object. That needs
  no template parameter on `SensorProcessingTask` and no recompilation of the estimator. Cost: 7
  indirect calls per prediction.

### 6.2 How to replace the model (same 3 states)

1. Write `class MyModel final : public IVerticalProcessModel` (+ a new `ProcessModelId`).
2. In `SensorProcessingTask.hpp`, change the type of `navModel_` (one line) and its constructor
   argument.
3. Run the host suite: the equivalence tests keep testing the engine; the scenarios now test the new
   model; add model-specific tests.

Nothing changes in: the sensor drivers, preprocessing, attitude, the engine,
`VerticalNavigationEstimator`, `NavigationEstimate`, the mailbox, or any consumer.

**Demonstrated:** `nav_process_model_swap_with_nonlinear_mock_model` runs the unchanged estimator
with a test-only **nonlinear** model (constant acceleration + quadratic drag, id `Test`). The same
stand-in consumer (`fastDescentWarning(const NavigationEstimate&)`) works with both models, the engine
called the injected model more than 1000 times, and the estimates really differ.

### 6.3 If the state dimension changes (honest list)

A future aerodynamic model may need airspeed, wind, a barometer-bias state or wing parameters. A
3-state interface **cannot** represent those. What changes:

| Component | Change needed for N ≠ 3 |
|---|---|
| `NavMath.hpp`, `UnscentedKalmanFilter<N>` | **none**: templated on N (`nav_engine_reused_with_four_states` runs N = 4 with a barometer-bias state) |
| `IVerticalProcessModel` / `VerticalStateVec` / `VerticalCov` | new interface (or template it on N) |
| `VerticalNavigationEstimator` | state indices, initialisation (new P0 rows), measurement functions `h(x)` (e.g. baro = h + bias), late-sample inflation, validity rules |
| `NavigationEstimate` | stays valid for consumers if the new states are internal; add fields only if consumers need them (e.g. airspeed). The 3×3 `covariance` stays the marginal of h, vz, az |
| `NavigationConfig` | new initial uncertainties and noises |
| Health fields, tests | new fields and tests |
| Drivers, preprocessing, attitude, mailbox, existing consumers | **none** |

---

## 7. The UKF mathematics

Scaled unscented transform with `n = 3`, `2n + 1 = 7` sigma points:

```
c = α²(n + κ),  λ = c − n
X₀ = x,  Xᵢ = x + √c·Lᵢ,  Xᵢ₊ₙ = x − √c·Lᵢ      (P = L Lᵀ, Lᵢ = column i)
Wm₀ = λ/c,  Wc₀ = λ/c + (1 − α² + β),  Wmᵢ = Wcᵢ = 1/(2c)
PREDICT: Yᵢ = f(Xᵢ, dt);  x⁻ = Σ Wm Y;  P⁻ = Σ Wc (Y − x⁻)(Y − x⁻)ᵀ + Q(dt)
UPDATE:  redraw Xᵢ from (x⁻, P⁻);  Zᵢ = h(Xᵢ);  ẑ = Σ Wm Z
         S = Σ Wc (Z − ẑ)(Z − ẑ)ᵀ + R;  Pxz = Σ Wc (X − x⁻)(Z − ẑ)ᵀ
         K = Pxz S⁻¹;  x⁺ = x⁻ + K (z − ẑ);  P⁺ = P⁻ − K S Kᵀ
```

**Parameters: α = 1, β = 2, κ = 0** (configurable in `NavigationConfig::ukf`; this β is **not** the
Madgwick gain):

* κ = 0 makes `n + κ = 3`, which matches the fourth moment (kurtosis) of a Gaussian for n = 3
  (Julier's rule `n + κ = 3`).
* β = 2 is optimal for Gaussian priors.
* α = 1 puts the sigma points at ±√3 σ, and all weights are non-negative except `Wm₀ = 0`. The
  textbook α = 10⁻³ gives `Wm₀ ≈ −10⁶`, and each mean becomes a difference of huge numbers. In float
  that destroys precision. **Measured:** after 20 000 mixed steps the deviation from the
  double-precision reference grows with |Wm₀|: 1.5·10⁻⁴ σ (α = 1), 3.7·10⁻⁴ σ (α = 0.5),
  8.6·10⁻³ σ (α = 0.1), **19 σ (α = 10⁻³, unusable)**. For the linear placeholder, α does not change
  the exact result at all.

**Numerical policy** (`UnscentedKalmanFilter.hpp` header):

* Every candidate result (sigma points, mean, covariance) is checked before it replaces the current
  state: it must be finite, and the new covariance must pass an actual **Cholesky factorisation**.
* On failure nothing is committed, and the call returns a `UkfStatus` (`NonFiniteSigmaPoint`,
  `ResultNotPositiveDefinite`, `InnovationNotPositiveDefinite`, `CovarianceNotPositiveDefinite`, ...).
  The estimator reports `NumericalFault` (section 10.3).
* The only correction applied is symmetrisation `(P + Pᵀ)/2`, which removes float round-off asymmetry
  and cannot hide a real fault. There is no silent reset, no eigenvalue clipping and no "repair".
* Means are accumulated as `Y₀ + Σ Wᵢ (Yᵢ − Y₀)` to avoid cancellation at large altitudes.
* `evaluate()` (compute innovation, NIS, gain inputs) and `apply()` (commit) are separate, so the
  estimator can gate before anything changes.
* Memory: the `UnscentedKalmanFilter<3>` object is 84 bytes. Stack per `predict()` is 360 B (Release
  `.su`). No heap, no exceptions.

---

## 8. Measurement models and sample selection

Every measurement is scalar and linear, `z = eᵢᵀ x + noise`, with i = h, vz or az. Sequential scalar
updates are exact for independent measurements.

### 8.1 Barometric altitude: `z = h`

* **Input:** the preprocessing **vote** `baroAltitude_m` (mean of the agreeing barometers), its status,
  contributing mask, per-channel timestamps and reference pressures.
* **Each sample once (`BaroSampleTracker`):** the vote is republished in every 10 ms frame, the
  barometers run at 50 Hz, and the two are read in different 10 ms cycles. A vote is a **new
  observation only if every contributing barometer's sample is newer than the one used last time**.
  Two healthy barometers therefore give one observation per 20 ms containing two fresh samples (one
  vote = one observation, **never two independent observations**). One barometer gives its own 50 Hz.
  Test: exactly 100 observations in 2 s; end to end with the real voter, 1500 in 30 s.
* **Observation time:** the midpoint of the contributing samples (`newest − spread/2`), because the
  vote is their mean.
* **Noise:** `baroSigma_m` (0.7 m), × 1.2 when only one barometer contributes, × 2 on the first sample
  after the contributing set changed (`BaroSourceChanged`).
* **Ambiguous** (barometers disagree, faulty one unknown): no output from preprocessing, so the update
  is skipped (`MeasurementOutcome::Ambiguous`).
* **Gate:** reject if `|ν| > 4·√S` (`baroGateSigma`).
* **Persistent disagreement recovery:** after ≥ 2 s **and** ≥ 20 consecutive rejections, the altitude
  variance is inflated by ν² and the velocity variance by (ν/T)² (T = time since the last accepted
  sample, ≥ 1 s), then the sample is accepted (`AcceptedInflated`, counted in `recoveries`, state
  `Recovering`). After an outage ≥ 1 s the same happens at once. **Trade-off:** a fault that shifts
  both barometers equally (which the vote cannot detect) is followed after 2 s (S14: 2.02 s).
* **Reference change:** if a contributing barometer's reference pressure changes (new calibration
  session), altitudes before and after refer to different zeros. The altitude variance is inflated by
  (100 m)² so the next sample re-anchors `h`. `vz` is untouched (no fake velocity spike), and the
  change is counted in `referenceResets`.

### 8.2 Vertical acceleration: `z = az`

* **Input:** `AttitudeEstimate::linearAccel.vertical_mps2` at the accelerometer sample time. Valid only
  if `linearAccel.valid` **and** `orientationValid`. Invalid acceleration is never fed as zero (test
  `nav_est_initialises_during_climb_with_honest_velocity_uncertainty`).
* **Acceleration is a measurement of the `az` state, not a deterministic input** to the prediction.
  It is therefore used once, in one place. The brief's correlation concern (the same signal as input
  and as measurement) does not arise.
* **Noise:** `accelSigma_mps2` (0.35), multiplied by **conservative, not statistically calibrated**
  factors:

  | Condition (from `AttitudeEstimate`) | Factor |
  |---|---|
  | attitude not reliable (Degraded / Initializing) or Madgwick `Recovering` | × 3 |
  | Madgwick skipped its accel correction (turn gate / direction / magnitude) | × 1.5 |
  | Madgwick has had no accel correction for long (`AccelRejectedProlonged`) | × 3 |
  | IMU accel or gyro vote Degraded | × 1.5 |
  | product > `accelMaxInflation` (10) | **rejected** (`RejectedQuality`) |
  | `attitude::GapDetected` or `attitude::NumericalFault` | **rejected**: after a gyro gap the orientation can be tens of degrees wrong |

* **Gate:** 6σ. After 5 consecutive rejections, the `az` variance is inflated by ν² and the sample is
  accepted. Real abrupt acceleration changes (steps) must not be gated out forever.

### 8.3 GNSS vertical velocity: `z = vz` (optional; works without it)

* Used only if `useGnssVelocity`, fix 3D (or GNSS + dead reckoning), ≥ 6 satellites, and `sAcc` finite,
  > 0 and ≤ 1.5 m/s. `σ = max(1.5·sAcc, 0.15 m/s)`, because vertical velocity is usually worse than
  NAV-PVT's 3-D `sAcc`.
* **`sAcc` pass-through added:** the driver already decoded `speedAccuracy_mps`, but preprocessing did
  not copy it into `GnssInfo`. This is now one additive field + one line in
  `PreprocessingPipeline::buildFrame()`. No other preprocessing change.
* Duplicates are rejected by receiver time-of-week **and** MCU timestamp. Gate 4σ (outliers are
  rejected: S21). There is no persistent-rejection recovery for GNSS.
* **GNSS altitude is not used**: it is MSL/ellipsoid, while `h` is relative to the barometric
  reference. Fusing it needs a datum alignment (offset state). This is left for later.

---

## 9. Time handling

* **Filter time `t_f`.** Each new sample is applied **at its own timestamp**: the state is predicted
  from `t_f` to the sample time, then updated. Samples of one update are sorted by time first. The
  output's `timestampMs` is `t_f`; `predictionAgeMs = now − t_f`. Consumers that need "now" extrapolate
  with `vz`/`az`.
* **Late samples** (older than `t_f`, newer than that source's previous sample): applied **at `t_f`**
  with the noise **inflated** by how much the measured state can change in the interval:
  * barometer: `Q_hh(δ) + ((|vz| + 2σ_vz)·δ)²`;
  * GNSS: `Q_vv(δ) + ((|az| + 2σ_az)·δ)²`;
  * acceleration: `Q_aa(δ)`.

  Limits: `maxLateMs` (30 ms), GNSS `gnssMaxLateMs` (300 ms). Beyond that the sample is `TooLate`.
  Delayed data are never silently treated as current: the output flags `LateMeasurement`.
* **Duplicates** (same sample again, e.g. republished frames) → `NotNew`, nothing changes (S24:
  bit-identical results with every frame delivered three times).
* **Out of order** (older than that source's previous sample) → `OutOfOrder`, skipped (S25).
* **Future timestamps** (after the processing time) → `FutureTimestamp`, skipped. This was found while
  testing wrap-around (validation §6.2).
* **Stale:** older than `baroMaxAgeMs` 100 ms, `accelMaxAgeMs` 50 ms, `gnssMaxAgeMs` 400 ms at
  processing time.
* **Wrap-around:** only signed differences are used. A run crossing `0xFFFFFFFF` gives bit-identical
  results to the same run at normal times (unit test and S26).
* **Mixed rates:** each source is assimilated at **its own** rate. The barometer runs at 50 Hz (or
  25 Hz) even though frames come at 100 Hz, and GNSS at 5–10 Hz (S22).
* **Outages:** with nothing new for `idlePredictMs` (50 ms), the state is predicted to "now", so the
  covariance grows. Prediction is bounded: nothing at all for `maxPredictionMs` (3 s), no barometer for
  `maxBaroOutageMs` (20 s), or σ_h > 50 m → `Unavailable`.

---

## 10. Initialisation, state machine and recovery

### 10.1 Initialisation (start-up in flight supported)

1. Collect barometer observations for ≥ 500 ms and ≥ 15 samples. A gap > 200 ms restarts the window.
2. Least-squares line fit `h(t) = b + v·t` (times relative to the newest sample).
3. Set `h₀ = b` and `vz₀ = v`. Velocity is **not** assumed zero.
4. Variances from the regression standard errors, using max(residual variance, `baroSigma²`), with
   floors of 0.3 m and 0.3 m/s. The h–v cross-covariance comes from the regression.
5. RMS residual > 1.5 m (strong acceleration or outliers) → keep the newer half and continue.
6. `az₀` = the current acceleration measurement with its σ if valid, else 0 with σ = 2 m/s².

S27 (start at −3 m/s while accelerating): valid after 0.52 s, initial vz error +0.29 m/s with
σ = 0.87 m/s.

### 10.2 States

| State | Meaning | Output |
|---|---|---|
| `Uninitialized` | no barometer yet (e.g. preprocessing has no reference during calibration) | NaN, invalid |
| `Initializing` | collecting the window | NaN, invalid |
| `Tracking` | barometer and acceleration accepted recently | valid per σ limits |
| `Degraded` | no barometer accepted for > 500 ms, or no acceleration for > 500 ms (e.g. the first ~6 s after power-up, while the attitude calibrates) | numbers + σ; per-field validity by σ |
| `Recovering` | barometer back after an outage, or a recovery inflation; leaves after 10 accepted samples | numbers + σ |
| `Unavailable` | > 3 s with no measurement, > 20 s without barometer, or σ_h > 50 m | NaN; re-initialises when the barometer returns |
| `NumericalFault` | the engine refused a result this update | NaN for this update; fault code kept; re-initialises |

Per-field validity: `altitudeValid` if σ_h ≤ 15 m; `velocityValid` if σ_v ≤ 2 m/s; `accelerationValid`
if σ_a ≤ 2 m/s² **and** an acceleration was accepted within 500 ms. Otherwise `UncertaintyLarge`.

### 10.3 Numerical faults

The fault is reported (state, `navflag::NumericalFault`, `numericalFaults`, `lastFaultCode` =
`UkfStatus`, `lastFaultAtMs`). The bad numbers are **not** published. The filter re-initialises from
the next barometer window (documented policy, counted in `reinitializations`), and the counters
survive. Test: `nav_est_numerical_fault_reported_and_recovered` (model starts returning NaN).

---

## 11. `NavigationEstimate`: the safety-facing API

Plain data: trivially copyable, 312 bytes on the Cortex-M7. The main fields:

* **Identity:** `sequence`, `timestampMs` (filter time), `computedAtMs`, `predictionAgeMs`,
  `estimator` (`VerticalUkf` / `BaroAlphaBeta`), `processModel` {id, version}, `state`.
* **Values:** `altitude_m`, `verticalVelocity_mps`, `verticalAccel_mps2` (positive up), each with a
  `*Valid` flag. Invalid values are NaN.
* **Uncertainty:** the 3×3 `covariance`, `altitudeSigma_m`, `velocitySigma_mps`, `accelSigma_mps2`.
* **Per source** (`baro`, `accel`, `gnss`), as `MeasurementDiagnostics`:
  * this update: outcome, innovation, innovation σ, NIS, R used, sample age, time since accepted;
  * lifetime counters: accepted, gate rejections, quality rejections, not-new, out-of-order, too-late,
    inflated, recoveries.
* **Outage and quality:** `sinceAnyUpdateMs`, `outageMs`, `qualityFlags` (`navflag::*`).
* **Lifetime counters:** initialisations, re-initialisations, reference resets, predictions, numerical
  faults (+ last code and time), unavailable episodes.

**Rules for consumers** (flight state, landing, crash, SOS):

1. Read through `SensorProcessingTask::Inst().navigation().read(e)` and use `e.sequence` for newness.
2. Use only fields whose `*Valid` flag is set. Check `state` and `qualityFlags`.
3. Use the σ fields for thresholds (e.g. "descending faster than X with 2σ confidence").
4. Remember the known over-confidence of `altitudeSigma_m` under correlated barometric errors
   (validation §8). Altitude is relative to the barometric reference, not above ground.
5. Never depend on `processModel` for logic. It is for logging and diagnosis only.

---

## 12. RTOS integration

* **Placement:** inside `SensorProcessingTask` (priority 32, unchanged), after attitude, once per frame:
  ```cpp
  navigation_.update(pg::navigationInputFrom(work_, attitude_.latest()));
  navigationOut_.publish(navigation_.latest());
  ```
  No new task, queue or priority change. Acquisition (40), sampling rates and queue overflow policies
  are unchanged. A separate task would need its own stack (≥ 1 KB), a queue and a copy of every frame
  and attitude estimate, and would see the data later.
* **Ownership:** this task is the only writer of the estimator and the mailbox. Readers copy under
  `RtosCriticalSection` (`taskENTER_CRITICAL`). The copy is 312 bytes, estimated at about 80 word
  moves, i.e. a few µs at 64 MHz. `HealthMonitorTask` copies it into a member (not its 2 KiB stack).
* **Start-up check:** `SOAR_ASSERT(navigation_.configValid())` in `InitTask`, next to the attitude
  check.
* **Health (`g_systemHealth`, 24 new fields):**
  * state, model id and per-source outcomes;
  * altitude (cm), vertical velocity (mm/s), acceleration (mm/s²); `INT32_MIN` = invalid;
  * σ_h and σ_v; prediction age; outage;
  * accepted/rejected counters, recoveries, initialisations, unavailable episodes, numerical faults and
    last fault code.

### 12.1 Resources (Release, `-Os`, CubeIDE build)

| Item | Value | Basis |
|---|---|---|
| Flash (text) | +9 852 B total (navigation code 8 088 B; the rest is the adapter, the processing-task integration and health reporting) | `arm-none-eabi-size` / `nm`, measured |
| Static RAM | +2 008 B bss: SensorProcessingTask object +1 608 (estimator 1 264, mailbox 320, model 12), HealthMonitorTask +316, `g_systemHealth` +84 | `nm -S`, measured |
| Heap | **0**: no `new`, `malloc`, containers or exceptions in `Algorithms/Navigation` | code + link map |
| Stack, deepest chain | `Run` 256 + `update` 120 + `processCandidate` 120 + `predictTo` 40 + `predict` 360 + `propagate` 32 = **928 B** of the 4 KiB SensorProc stack | Release `.su`, measured |
| Host time | 152 ns per `update()` (Apple M4, `-O2`, 60 000 frames with GNSS) | measured |
| Target time | **estimate 50–150 µs per update** at 64 MHz (0.5–1.5 % of the 10 ms period) | host time × the Milestone 3 host→target ratio; **not measured** |
| Max update rate | the estimator does bounded work per frame: ≤ 3 sample updates + ≤ 30 prediction steps (worst case after a 3 s gap) | code |

Measure on hardware: toggle a GPIO or read DWT `CYCCNT` around `navigation_.update()`
(HARDWARE_INTEGRATION_TODO.md §14).

---

## 13. Baseline estimator (`BaroAlphaBetaEstimator`)

* **Algorithm:** barometer only, α-β filter, no accelerometer. Same `BaroSampleTracker` rule, so each
  barometer sample is used once.
* **Gains:** α and β are the **steady-state Kalman gains** of a constant-velocity model with
  piecewise-constant white acceleration (Kalata), **recomputed for the actual dt**. A test proves they
  equal a Riccati iteration of the 2-state KF.
* **Reported σ:** the steady-state posterior covariance.
* **Tuning:** tuned as fairly as the UKF. The manoeuvre parameter σa was swept on the same tuning runs
  and the best value (3 m/s²) is the default.
* **Fallback:** it outputs a `NavigationEstimate` with `estimator = BaroAlphaBeta`, acceleration invalid
  and `processModel = None`. It is **not** switched in automatically in this milestone (and, unused by
  the firmware, it is not linked into the image). Fallback policy belongs to the future safety layer;
  the estimator id field already makes the source explicit.

---

## 14. Configuration (`NavigationConfig`, defaults)

| Parameter | Default | Meaning / basis |
|---|---|---|
| `ukf` α, β, κ | 1, 2, 0 | §7 |
| `jerkPsd_m2ps5` (qj) | 3.0 | sweep 0.1–10: 3 lowers acceleration RMSE and NEES vs 1, with no h or v cost |
| `accelErrorPsd_m2ps3` (qa) | 0.02 | sweep 0–0.2: 0 gives the over-confident velocity of §5.1; larger values increase noise |
| `baroSigma_m` | 0.7 | sweep 0.25–1.0: 0.25 rejects good samples; 0.7 balances v error and NEES |
| `baroSingleSensorFactor` / `baroSourceChangeFactor` | 1.2 / 2.0 | conservative |
| `baroGateSigma` | 4 | sweep 3–6: no difference without faults; 4 rejects a 15 m step |
| `baroRejectRecoveryMs` / `Count` | 2000 ms / 20 | trade-off: false re-anchor vs time to follow a common fault |
| `baroOutageReanchorMs` | 1000 ms | after an outage, re-anchor at once |
| `accelSigma_mps2` | 0.35 | noise + ~1° tilt + part of the 0.2 m/s² possible bias; sweep 0.15–0.7: small effect |
| accel factors | 3 / 1.5 / 3 / 1.5, max 10 | §8.2; heuristic |
| `accelGateSigma` / recovery count | 6σ / 5 | steps must not be rejected forever (S8) |
| GNSS | on; ≥ 6 sats; sAcc ≤ 1.5; σ = max(1.5·sAcc, 0.15); gate 4σ | §8.3 |
| `maxLateMs` / `gnssMaxLateMs` | 30 / 300 ms | §9 |
| `maxPredictStepMs` / `idlePredictMs` / `maxPredictionMs` | 100 / 50 / 3000 ms | §9 |
| `baroDegradedMs` / `accelDegradedMs` / `maxBaroOutageMs` | 500 / 500 / 20 000 ms | §10 |
| init window / min samples / max gap / max residual | 500 ms / 15 / 200 ms / 1.5 m | §10.1 |
| init σ floors | 0.3 m, 0.3 m/s; az 2 m/s² if unmeasured | §10.1 |
| validity limits | σ_h 15 m, σ_v 2 m/s, σ_a 2 m/s²; Unavailable at σ_h 50 m | §10.2 |

None of these values has been tuned on flight data.

---

## 15. Known limitations

1. **Simulation only.** No sensor, timing or flight data exist. The scenario motions are classes of
   vertical movement, not validated paraglider aerodynamics.
2. **Placeholder model:** it does not use any knowledge of the wing.
3. **Accelerometer bias** is not a state: velocity bias ≈ bias × 1.3 s. The UKF is worse than the
   barometer-only baseline when the bias is large (S17). Remedy: accelerometer calibration (hardware
   TODO) or a bias state (N = 4; the engine already supports it).
4. **Correlated barometric errors** (airflow, weather) are treated as white, so altitude σ is
   over-confident under realistic errors (altitude 2σ coverage 72 % instead of 95 %). Weather drift is
   indistinguishable from climb for a relative barometer (S13: the 3 m drift over 5 min is followed).
5. **A fault common to both barometers** (equal step) is followed after 2 s by design (S14). A step on
   one barometer makes the vote Ambiguous: accelerometer-only Degraded, then Unavailable after 20 s.
6. **Measurement timing** uses read-completion timestamps. BMP585 conversion latency and NEO-M9N
   navigation latency are unknown (no time-pulse pin), and GNSS latency is only partly covered by the
   late-sample inflation.
7. **Acceleration-noise inflation factors** are heuristic, not statistically calibrated.
8. **Target execution time is estimated, not measured.**
