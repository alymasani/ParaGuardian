# Vertical UKF: Validation Report (Milestone 4)

**Scope:** host simulation, independent mathematical checks, sanitizers, and native STM32CubeIDE firmware
builds. **No hardware, no sensor data, no flight data.** Nothing in this report validates
the system for flight or safety use. Every number below comes from the final run of the test suite
(`sh Tests/run_host_tests.sh`; its `[nav]` and `[metric]` lines) or from the final CubeIDE build, unless it
is marked **estimate**.

---

## 1. Summary

| Item | Result |
|---|---|
| Existing tests before any change | **278 / 278 passed** |
| Final host tests | **338 / 338 passed** (278 existing, unchanged + 60 new); also 338 / 338 under ASan+UBSan and under TSan, **0 sanitizer reports** |
| UKF vs independent double-precision linear KF | single updates agree to ≤ 2.3·10⁻⁶ (relative); 20 000 mixed steps to ≤ 1.5·10⁻⁴ σ (α = 1) |
| 30 required scenarios (+ one-sensor step variant) | all run; 0 numerical faults; covariance symmetric (asymmetry 0) and positive in every run |
| UKF vs barometer-only baseline (vertical velocity RMSE) | UKF lower in **29 of 31** rows (e.g. thermals 0.129 vs 0.546 m/s; climb→descent delay 40 vs 410 ms). **UKF worse in 2:** S17 accelerometer bias 0.2 m/s² (0.297 vs 0.222) and S29 barometer loss with bias (0.292 vs 0.189; the baseline's figure excludes the loss period). Nearly equal in S19 (0.216 vs 0.222). See §7.3 |
| Uncertainty, matched assumptions | approximately consistent, slightly conservative: NEES 2.25 (ideal 3), NIS 1.08 / 1.00 |
| Uncertainty, realistic errors | **over-confident in altitude**: NEES 6.49, altitude 2σ coverage 72 %; velocity coverage 99 % |
| CubeIDE Debug / Release | 0 errors, 0 warnings each; Release text +9 852 B, bss +2 008 B |
| Heap | none added; stack chain 928 B of 4 KiB (Release `.su`) |
| SoarOS | unchanged (`426c1ef`, clean) |

---

## 2. Baseline before changes (recorded first)

| Check | Result |
|---|---|
| Git | branch `Pre-processing`, HEAD `c26ff8c`, Milestone 3 work uncommitted (11 modified, 10 untracked entries), no stash |
| SoarOS | `426c1efe895bb58ba2a57ea128c9aad642e9be9f (heads/master)`, clean |
| Host tests | 278 tests, 278 passed, 0 failed |
| CubeIDE Debug | 0 errors / 0 warnings; text 80 916, data 104, bss 46 024 |
| CubeIDE Release | 0 errors / 0 warnings; text 49 484, data 104, bss 46 016 |

The Milestone 3 claims (278 tests, both builds) were confirmed against the repository before editing.

---

## 3. Test inventory (60 new tests)

| File | Tests | Content |
|---|---|---|
| `Tests/unit/test_navigation_math.cpp` | 12 | Cholesky / SPD inverse, white-jerk + unmodelled-acceleration Q vs independent numerical integration, CA propagation, UT weights, sigma-point moments, **UKF ≡ linear KF** (prediction, each measurement type, 20 000-step sequences for 5 parameter sets), numerical-fault detection, α-β gains vs Riccati |
| `Tests/unit/test_navigation_estimator.cpp` | 17 | configuration, NaN-when-invalid, in-flight initialisation, one barometer observation per fresh pair, duplicates / out-of-order / future / late / too-late, wrap-around bit-identity, reference change, Ambiguous vote, GNSS rules and sign, adapter mapping, acceleration quality inflation, outage → Unavailable → re-init, numerical fault → recovery, **model swap (nonlinear mock)**, **N = 4 engine**, persistent barometer disagreement |
| `Tests/unit/test_navigation_scenarios.cpp` | 27 | scenarios 1–30 (some tests cover two), consistency (NEES / NIS / coverage over seeds), sweeps (UKF and baseline) |
| `Tests/unit/test_navigation_integration.cpp` | 4 | end to end through the real preprocessing + Madgwick + UKF, GNSS through the pipeline, barometer reference start-up, mailbox concurrency (TSan) |
| `Tests/sim/VerticalSimulator.hpp`, `Tests/sim/LinearKalmanReference.hpp` | — | independent truth / sensor simulation; independent reference filter |

---

## 4. UKF ≡ linear Kalman filter (mandatory verification)

The placeholder model and every measurement model are linear, so a correct UKF must reproduce the
linear KF. Reference: `Tests/sim/LinearKalmanReference.hpp`. It is double precision, has its own matrix
code, writes F and Q out explicitly, and uses the Joseph-form update. It shares no code with the engine.
Deviations are scaled by the reference's σ (state) or `√(PᵢᵢPⱼⱼ)` (covariance).

| Check | Max state deviation | Max covariance deviation |
|---|---|---|
| Prediction, dt = 1, 10, 20, 100, 500 ms | < 10⁻⁵ σ (asserted) | < 10⁻⁵ (asserted) |
| Barometer update (z = h) | 3.3·10⁻⁷ σ | 2.3·10⁻⁶ |
| Acceleration update (z = az) | 6.8·10⁻⁸ σ | 7.9·10⁻⁷ |
| GNSS velocity update (z = vz) | 2.5·10⁻⁷ σ | 7.3·10⁻⁷ |
| 20 000 steps, random dt 1–40 ms, barometer 50 Hz + acceleration 100 Hz + GNSS 5 Hz, α=1 β=2 κ=0 (**default**) | 1.5·10⁻⁴ σ | 1.2·10⁻⁴ |
| same, α = 0.5 | 3.7·10⁻⁴ σ | 2.1·10⁻⁴ |
| same, α = 1, β = 0, κ = 1 | 1.7·10⁻⁴ σ | 1.1·10⁻⁴ |
| same, α = 0.1 (Wm₀ = −99) | 8.6·10⁻³ σ | 1.0·10⁻³ |
| same, α = 0.001 (Wm₀ ≈ −10⁶) | **19 σ: unusable in float** | 2.9·10² |

**Interpretation, not explained away:** the mathematics is exact for every parameter set. The growth
with smaller α follows |Wm₀|, because each weighted mean becomes a difference of huge terms in float.
The test asserts that the deviation **grows** from α = 0.5 to α = 0.1, which separates a precision
effect from a formula error. The default α = 1 has `Wm₀ = 0` and is the best-conditioned choice. The
long-run deviation of 1.5·10⁻⁴ σ is accumulated float round-off over 20 000 steps.

Also verified:
* white-jerk Q (and the added qa term) against Simpson integration of the defining integral, dt 1 ms – 1 s;
* the sigma points reproduce the mean and covariance exactly;
* weights sum to 1.

---

## 5. Estimator behaviour (unit level)

| Property | Evidence |
|---|---|
| Each barometer vote assimilated once; two barometers = one observation | exactly 100 observations in 2 s (2 × 50 Hz samples, 100 Hz frames); end to end with the **real** voter: 1 500 in 30 s |
| Each acceleration sample once | exactly 200 in 2 s; republished frames change nothing (bit-identical) |
| Invalid acceleration never used as zero | 0 accepted while `accelValid = false`; state Degraded + `AccelUnavailable` |
| In-flight start, velocity not assumed zero | climbing at 3 m/s: |vz error| < 0.3 m/s at initialisation, σ_v > 0.05 |
| Late GNSS (150 ms) inflated; 340 ms → `TooLate` | `AcceptedInflated`, σ > 0.3; `TooLate` |
| Out-of-order / future timestamps | `OutOfOrder` (state unchanged), `FutureTimestamp` |
| uint32 wrap-around | bit-identical state, covariance and counters vs the same run at normal times |
| Barometer reference change | `referenceResets` 1; altitude re-anchored within 0.5 s; 0 gate rejections; no velocity spike |
| Ambiguous barometers | skipped; estimate stays valid |
| GNSS quality | 2-D fix, 4 satellites, sAcc 3 m/s or NaN rejected; repeated iTOW `NotNew`; disabled → `Disabled` |
| Acceleration quality factors | ×1.5 turn gate, ×3 unreliable, ×1.5 IMU Degraded, `GapDetected` rejected, product 13.5 > 10 rejected |
| Outage → Unavailable → re-initialisation | σ_v grew > 1 m/s while Degraded; Unavailable; 2nd initialisation after data return |
| Numerical fault | model returning NaN → `NumericalFault`, code `NonFiniteSigmaPoint`, NaN output, re-initialised, counters kept |
| Persistent common barometer offset (+30 m) | 100 samples rejected, re-anchored after exactly 2 000 ms |
| Process-model replacement | §9 |

---

## 6. Findings during development (and what was changed)

### 6.1 Over-confident velocity with an imperfect model or accelerometer (design change)
The first version used only the white-jerk Q. With `az` measured at 100 Hz, the filter believed `vz` to
σ ≈ 0.04 m/s. The model-swap test used a deliberately wrong nonlinear model (drag 2 m/s² at 10 m/s).
`vz` ended **8 m/s wrong** while σ stayed 0.04 m/s, and the barometer was gated out (1 025 rejections).
Re-anchoring altitude alone could not repair the velocity, so the cycle repeated. The cause is that
accelerometer and model errors are not white. Fixes:
* the **unmodelled-acceleration term qa** in the process noise (design §5.1);
* barometer recovery that also re-opens the velocity variance by (ν/T)².

After the fix, the same grossly wrong model tracks with **0** rejections, at a biased velocity (−8.4 vs −10
m/s, σ 0.12). This is what a wrong model does, and it is now visible to the barometer gate rather than
silent. The qa sweep (§10) confirms qa = 0 is the worst setting: NEES 28.4 and the largest altitude error.

### 6.2 Future timestamps (code change)
A test harness bug produced a barometer timestamp 5 s in the future near the uint32 wrap. The estimator
accepted it. Preprocessing already rejects future samples, but the estimator now also refuses them
(`FutureTimestamp`) instead of relying on that.

### 6.3 Simulator artifacts (test-only changes, recorded for transparency)
* With irregular frame times, the simulator first produced barometer samples only when a frame happened
  to land on the barometer schedule, giving 90 % usable in S23. Sensors now sample on their own clocks.
  S23 is 100 % usable.
* Two first-draft test expectations were wrong and were corrected:
  * cumulative vs windowed acceleration count;
  * a GNSS sample meant to be "too late" was correctly classified `Stale` first.
* The single-barometer step (S14b) correctly ends `Unavailable`, not `Degraded`: the vote stays
  Ambiguous for 20 s, which exceeds `maxBaroOutageMs`.

### 6.4 Baseline tuned fairly (configuration change)
The baseline's σa was swept on the tuning runs. 3 m/s² is best (velocity RMSE 0.353 vs 0.432 at the
initial 1 m/s²) and became its default. All comparisons below use the tuned baseline.

---

## 7. Scenario results (UKF vs baseline, identical inputs)

**Setup:**
* Held-out seed 101; 60 s per run unless stated; metrics after a 2 s warm-up.
* Errors are evaluated at the state's own time (`timestampMs`), against the RK4 double-precision truth.
* **Realistic sensor errors in every run:**
  * barometer: 0.4 m white noise per sensor, plus 0.25 m **correlated** airflow error (τ = 3 s) common
    to both;
  * accelerometer: 0.15 m/s² white noise plus 0.05 m/s² correlated attitude error (τ = 5 s);
  * GNSS (where present): 5 Hz, σ 0.15 m/s.
* The truth is never the filter's white-jerk constant-acceleration process. Every run therefore
  includes **process-model mismatch**.
* "usable" = altitude **and** velocity valid. "p95" = 95th percentile of |error|.
* Baseline metrics only count frames where the baseline is valid. In outage scenarios (S16, S28, S29,
  S14b) its errors therefore **exclude** the outage, while the UKF's include it (see "usable").

### 7.1 RMSE, 95th percentile, worst case, usability, bias, rejections

| # | Scenario | UKF h RMSE / p95 / max (m) | UKF vz RMSE / p95 / max (m/s) | UKF az RMSE | BASE h RMSE / max | BASE vz RMSE / p95 / max | usable UKF / BASE | UKF bias h / vz | rejections b/a/g, recov. |
|---|---|---|---|---|---|---|---|---|---|
| S01 | stationary | 0.239 / 0.474 / 0.573 | 0.133 / 0.247 / 0.315 | 0.087 | 0.244 / 0.672 | 0.222 / 0.443 / 0.703 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S02 | constant climb +2 m/s | 0.238 / 0.474 / 0.573 | 0.133 / 0.246 / 0.315 | 0.087 | 0.244 / 0.672 | 0.222 / 0.443 / 0.703 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S03 | constant descent -1.2 m/s | 0.238 / 0.474 / 0.573 | 0.133 / 0.247 / 0.315 | 0.087 | 0.244 / 0.672 | 0.222 / 0.443 / 0.703 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S04 | increasing descent -1 -> -6 m/s | 0.238 / 0.475 / 0.573 | 0.133 / 0.247 / 0.315 | 0.087 | 0.254 / 0.681 | 0.257 / 0.549 / 0.952 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S05 | decreasing descent -6 -> -1.5 m/s | 0.238 / 0.475 / 0.573 | 0.133 / 0.247 / 0.315 | 0.086 | 0.239 / 0.672 | 0.255 / 0.560 / 0.889 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S06 | climb +2.5 -> descent -1.5 m/s in 3 s | 0.238 / 0.475 / 0.573 | 0.133 / 0.246 / 0.314 | 0.087 | 0.253 / 0.672 | 0.266 / 0.571 / 0.984 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S07 | smooth acceleration changes (0.8 m/s^2, 8 s) | 0.238 / 0.475 / 0.573 | 0.133 / 0.247 / 0.313 | 0.086 | 0.224 / 0.605 | 0.353 / 0.661 / 0.847 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S08 | abrupt acceleration steps (+-1.5 m/s^2) | 0.238 / 0.473 / 0.573 | 0.134 / 0.247 / 0.315 | 0.178 | 0.248 / 0.672 | 0.364 / 0.760 / 1.243 | 100.0 / 100.0 % | -0.001 / +0.037 | 0/11/0, 0 |
| S09 | thermal-like lift cycles | 0.238 / 0.475 / 0.594 | 0.129 / 0.245 / 0.341 | 0.090 | 0.278 / 0.919 | 0.546 / 1.365 / 1.823 | 100.0 / 100.0 % | +0.014 / +0.030 | 0/0/0, 0 |
| S10 | turbulence-like disturbances (1.5 m/s^2) | 0.239 / 0.475 / 0.572 | 0.133 / 0.248 / 0.314 | 0.270 | 0.264 / 0.751 | 0.468 / 0.896 / 1.653 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S11 | coordinated turn: Madgwick turn gate 20 s, attitude error | 0.233 / 0.465 / 0.573 | 0.170 / 0.298 / 0.480 | 0.132 | 0.244 / 0.672 | 0.222 / 0.444 / 0.703 | 100.0 / 100.0 % | +0.028 / +0.088 | 0/0/0, 0 |
| S12 | barometer noise 1.5 m per sensor | 0.337 / 0.658 / 0.950 | 0.197 / 0.432 / 0.688 | 0.087 | 0.410 / 1.309 | 0.506 / 1.001 / 1.702 | 100.0 / 100.0 % | -0.002 / +0.034 | 31/0/0, 0 |
| S13 | barometer weather drift 0.01 m/s | 1.748 / 2.944 / 3.346 | 0.124 / 0.240 / 0.382 | 0.086 | 1.752 / 3.368 | 0.201 / 0.387 / 0.693 | 100.0 / 100.0 % | +1.504 / +0.016 | 0/0/0, 0 |
| S14 | barometer step +15 m on BOTH sensors at 30 s | 10.401 / 15.198 / 15.460 | 0.128 / 0.234 / 0.575 | 0.087 | 10.713 / 15.490 | 0.225 / 0.448 / 0.703 | 99.5 / 100.0 % | +7.223 / +0.022 | 100/0/0, 1 |
| S14 | barometer step +15 m on ONE sensor at 30 s | 0.865 / 2.343 / 3.329 | 0.199 / 0.400 / 0.427 | 0.087 | 0.262 / 0.672 | 0.245 / 0.514 / 0.703 | 82.8 / 50.0 % | +0.207 / +0.096 | 0/0/0, 0 |
| S15 | one barometer lost at 20 s | 0.267 / 0.516 / 0.752 | 0.132 / 0.241 / 0.299 | 0.086 | 0.273 / 0.810 | 0.247 / 0.476 / 0.962 | 100.0 / 100.0 % | +0.056 / +0.018 | 0/0/0, 0 |
| S16 | both barometers lost 20-35 s | 0.463 / 0.882 / 0.914 | 0.129 / 0.243 / 0.342 | 0.089 | 0.304 / 0.837 | 0.248 / 0.388 / 1.987 | 100.0 / 75.0 % | +0.277 / +0.016 | 0/0/0, 0 |
| S17 | accelerometer bias +0.2 m/s^2 | 0.272 / 0.539 / 0.710 | 0.297 / 0.465 / 0.549 | 0.241 | 0.244 / 0.672 | 0.222 / 0.443 / 0.703 | 100.0 / 100.0 % | +0.132 / +0.268 | 0/0/0, 0 |
| S18 | acceleration saturation spikes (35 m/s^2, 0.3 s) | 0.311 / 0.666 / 1.033 | 0.421 / 1.090 / 2.650 | 1.586 | 0.356 / 1.575 | 1.095 / 2.480 / 6.419 | 100.0 / 100.0 % | -0.076 / -0.095 | 0/64/0, 16 |
| S19 | Madgwick degraded 15 s (attitude error 0.5 m/s^2) | 0.235 / 0.490 / 0.625 | 0.216 / 0.455 / 0.732 | 0.202 | 0.244 / 0.672 | 0.222 / 0.443 / 0.703 | 100.0 / 100.0 % | -0.008 / +0.024 | 0/100/0, 0 |
| S20 | GNSS dropout 20-40 s | 0.253 / 0.486 / 0.695 | 0.122 / 0.246 / 0.317 | 0.214 | 0.271 / 0.717 | 0.244 / 0.490 / 0.735 | 100.0 / 100.0 % | +0.069 / +0.016 | 0/0/0, 0 |
| S21 | GNSS 5% outliers (+-5 m/s) | 0.235 / 0.439 / 0.642 | 0.097 / 0.180 / 0.310 | 0.088 | 0.251 / 0.669 | 0.197 / 0.388 / 0.613 | 100.0 / 100.0 % | -0.036 / +0.020 | 0/0/16, 0 |
| S22 | mixed rates: baro 50/25 Hz, GNSS 10 Hz + 100 ms latency | 0.203 / 0.433 / 0.554 | 0.064 / 0.131 / 0.190 | 0.086 | 0.262 / 0.733 | 0.396 / 0.709 / 0.955 | 100.0 / 100.0 % | +0.067 / +0.003 | 0/0/0, 0 |
| S23 | irregular frames 5-15 ms + 3 ms accel jitter | 0.301 / 0.609 / 0.738 | 0.140 / 0.294 / 0.445 | 0.085 | 0.293 / 0.895 | 0.343 / 0.645 / 0.899 | 100.0 / 100.0 % | +0.060 / -0.017 | 0/0/0, 0 |
| S24 | every frame republished twice (duplicates) | 0.238 / 0.475 / 0.573 | 0.133 / 0.247 / 0.313 | 0.086 | 0.224 / 0.605 | 0.353 / 0.661 / 0.847 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S25 | 2% of frames replayed 30 ms late (out of order) | 0.267 / 0.476 / 0.667 | 0.121 / 0.241 / 0.330 | 0.092 | 0.270 / 0.581 | 0.379 / 0.679 / 0.913 | 100.0 / 100.0 % | +0.144 / +0.021 | 0/0/0, 0 |
| S26 | uint32 ms wrap-around at 20 s | 0.238 / 0.475 / 0.573 | 0.133 / 0.247 / 0.313 | 0.086 | 0.224 / 0.605 | 0.353 / 0.661 / 0.847 | 100.0 / 100.0 % | -0.003 / +0.034 | 0/0/0, 0 |
| S27 | start-up in flight (-3 m/s, accelerating) | 0.235 / 0.474 / 0.573 | 0.132 / 0.247 / 0.316 | 0.087 | 0.251 / 0.685 | 0.315 / 0.557 / 1.680 | 99.1 / 99.2 % | -0.003 / +0.033 | 0/0/0, 0 |
| S28 | complete sensor outage 20-25 s | 0.270 / 0.517 / 0.692 | 0.139 / 0.259 / 0.433 | 0.093 | 0.251 / 0.695 | 0.212 / 0.372 / 1.246 | 93.2 / 92.3 % | +0.075 / +0.006 | 0/0/0, 0 |
| S29 | barometers lost 20-28 s with +0.1 m/s^2 accel bias, recovery | 1.001 / 2.625 / 5.058 | 0.292 / 0.707 / 0.972 | 0.139 | 0.255 / 0.699 | 0.189 / 0.352 / 0.703 | 100.0 / 87.1 % | +0.434 / +0.190 | 0/0/0, 0 |
| S30 | long duration 30 min (thermals + turbulence) | 0.252 / 0.487 / 1.007 | 0.127 / 0.250 / 0.507 | 0.142 | 0.279 / 0.993 | 0.534 / 1.329 / 2.131 | 100.0 / 100.0 % | +0.009 / -0.004 | 0/0/0, 0 |

(b/a/g = barometer / acceleration / GNSS gate or quality rejections; recov. = covariance-inflation recoveries.)

### 7.2 Mean absolute error

| # | UKF h MAE | UKF vz MAE | BASE h MAE | BASE vz MAE |
|---|---|---|---|---|
| S01 stationary | 0.191 | 0.108 | 0.194 | 0.178 |
| S02 constant climb +2 m/s | 0.190 | 0.108 | 0.194 | 0.178 |
| S03 constant descent -1.2 m/s | 0.191 | 0.108 | 0.194 | 0.177 |
| S04 increasing descent -1 -> -6 m/s | 0.191 | 0.108 | 0.201 | 0.200 |
| S05 decreasing descent -6 -> -1.5 m/s | 0.190 | 0.108 | 0.186 | 0.198 |
| S06 climb +2.5 -> descent -1.5 m/s in 3 s | 0.190 | 0.108 | 0.200 | 0.202 |
| S07 smooth acceleration changes (0.8 m/s^2 | 0.191 | 0.108 | 0.175 | 0.296 |
| S08 abrupt acceleration steps (+-1.5 m/s^2 | 0.191 | 0.109 | 0.199 | 0.274 |
| S09 thermal-like lift cycles | 0.195 | 0.105 | 0.226 | 0.384 |
| S10 turbulence-like disturbances (1.5 m/s^ | 0.191 | 0.109 | 0.203 | 0.373 |
| S11 coordinated turn: Madgwick turn gate 2 | 0.189 | 0.139 | 0.193 | 0.177 |
| S12 barometer noise 1.5 m per sensor | 0.273 | 0.153 | 0.336 | 0.407 |
| S13 barometer weather drift 0.01 m/s | 1.509 | 0.100 | 1.508 | 0.161 |
| S14 barometer step +15 m on BOTH sensors a | 7.334 | 0.105 | 7.776 | 0.180 |
| S14 barometer step +15 m on ONE sensor at  | 0.525 | 0.157 | 0.209 | 0.195 |
| S15 one barometer lost at 20 s | 0.222 | 0.110 | 0.220 | 0.195 |
| S16 both barometers lost 20-35 s | 0.369 | 0.103 | 0.243 | 0.170 |
| S17 accelerometer bias +0.2 m/s^2 | 0.230 | 0.269 | 0.194 | 0.177 |
| S18 acceleration saturation spikes (35 m/s | 0.233 | 0.228 | 0.243 | 0.438 |
| S19 Madgwick degraded 15 s (attitude error | 0.185 | 0.163 | 0.194 | 0.177 |
| S20 GNSS dropout 20-40 s | 0.203 | 0.099 | 0.221 | 0.192 |
| S21 GNSS 5% outliers (+-5 m/s) | 0.193 | 0.079 | 0.201 | 0.157 |
| S22 mixed rates: baro 50/25 Hz, GNSS 10 Hz | 0.153 | 0.051 | 0.212 | 0.335 |
| S23 irregular frames 5-15 ms + 3 ms accel  | 0.234 | 0.105 | 0.235 | 0.278 |
| S24 every frame republished twice (duplica | 0.191 | 0.108 | 0.175 | 0.296 |
| S25 2% of frames replayed 30 ms late (out  | 0.220 | 0.099 | 0.233 | 0.326 |
| S26 uint32 ms wrap-around at 20 s | 0.191 | 0.108 | 0.175 | 0.296 |
| S27 start-up in flight (-3 m/s, accelerati | 0.187 | 0.107 | 0.199 | 0.258 |
| S28 complete sensor outage 20-25 s | 0.220 | 0.115 | 0.202 | 0.160 |
| S29 barometers lost 20-28 s with +0.1 m/s^ | 0.509 | 0.210 | 0.206 | 0.150 |
| S30 long duration 30 min (thermals + turbu | 0.203 | 0.101 | 0.224 | 0.367 |

### 7.3 Reading the table honestly

* **Rows S01–S07 and S24/S26 are nearly identical for the UKF.** This is expected, not a copy error. The
  filter is linear and `az` is *measured*, so its error depends on the sensor noise (the same seed) and
  only weakly on the smooth truth trajectory. The baseline, which only predicts, does depend on the
  trajectory.
* **Where the UKF helps:** any acceleration change.

  | Scenario | UKF vz RMSE (m/s) | Baseline vz RMSE (m/s) |
  |---|---|---|
  | thermals (S9) | 0.129 | 0.546 |
  | turbulence (S10) | 0.133 | 0.468 |
  | 30 min (S30) | 0.127 | 0.534 |
  | saturation spikes (S18) | 0.421 | 1.095 |

  **Response delay** at the climb → descent crossing (S6): **UKF 40 ms, baseline 410 ms**. With GNSS
  (S22) the UKF reaches 0.064 m/s.
* **Where the UKF is worse or not better (not hidden):**
  * **S17, accelerometer bias +0.2 m/s²:** UKF vz RMSE 0.297 vs baseline 0.222, with a vz bias of
    +0.268 m/s. A constant bias is not a state in a 3-state filter. Remedies: accelerometer calibration
    (hardware), or a bias state (N = 4; the engine supports it).
  * **S11 (turn) and S19 (Madgwick degraded):** UKF 0.170 / 0.216 vs baseline 0.222. The UKF still wins
    on RMSE, but its velocity worst case and bias grow (S11 vz bias +0.088). Attitude-related
    acceleration errors pass into velocity, damped by the inflation factors.
  * **Altitude:** the two are within about ±10 % in most rows. The baseline is slightly better in S7,
    S24 and S26 (RMSE 0.224 vs 0.238 m). The UKF's altitude is pulled by its velocity.
  * **S29 (8 s barometer loss with 0.1 m/s² bias):** UKF vz RMSE 0.292 vs baseline 0.189. Not a like
    for like comparison: the UKF keeps estimating through the loss, and its error there is counted; the
    baseline has no output during the loss (87 % usable). The UKF's altitude error grows to 5.06 m
    before the barometer returns. It is back below 1 m and Tracking 0.18 s after the return.
* **Faults:**
  * **S14, common +15 m step on both barometers:** a fault the vote cannot detect. Rejected for
    2.02 s (100 samples), then followed **by design**. Velocity disturbance ≤ 0.57 m/s.
  * **S14b, one barometer stepped:** the vote is Ambiguous, so the UKF continues on acceleration only
    (Degraded) and becomes Unavailable at 50.0 s (20 s limit). It never jumped to the faulty value
    (max error 3.3 m during accelerometer-only).
  * **S13, weather drift:** followed, as for any relative barometer: 3.18 m error after 5 min of
    0.01 m/s drift; velocity bias +0.016 m/s.
* **GNSS:** with GNSS, S21 vz RMSE is 0.097 (0.133 without). 16 of the ±5 m/s outliers were gated (the
  rest fell inside the gate when σ was large, or were small). Max error 0.31 m/s.
* **Timing:**
  * S24: 12 002 repeated samples skipped, results bit-identical to the clean run;
  * S25: every replayed old frame classified `OutOfOrder`;
  * S26: wrap-around bit-identical to normal time;
  * S23: irregular frames 5–15 ms plus 3 ms jitter, 100 % usable.
* **Start-up and recovery:**
  * S27: valid 0.52 s after start while descending at −3 m/s and accelerating; initial vz error
    +0.29 m/s (σ 0.87);
  * S28: 5 s total outage → Unavailable → valid again 0.52 s after the data return;
  * S16: 15 s barometer loss → Degraded on acceleration only, max |h error| 0.91 m with σ_h 5.47 m
    (conservative) → Tracking again.

### 7.4 End to end (real preprocessing, real voter, real Madgwick)

* 1 m/s² sinusoidal vertical motion after the 5 s calibration: max |h error| 0.134 m, max |vz error|
  0.053 m/s, az RMSE 0.013 m/s². 1 500 barometer observations in 30 s; state Tracking.
* GNSS NAV-PVT-derived samples (positive up) through the pipeline at 5 Hz: 100 velocities assimilated in
  20 s, 2 470 republished copies skipped.
* During calibration there is no barometer reference, so navigation stays `Uninitialized` with NaN
  output (nothing fabricated). Tracking starts at ≈ 0 m relative altitude.

---

## 8. Uncertainty validation (NEES / NIS / coverage)

**Method:** held-out seeds 101–110; the sweeps used seeds 1–5.
* NEES = eᵀP⁻¹e over (h, vz, az); ideal mean = 3. The baseline's is over (h, vz); ideal 2.
* "NEES < χ²₉₅" should be ≈ 95 %.
* 2σ coverage should be ≈ 95 %.
* NIS = ν²/S per scalar update; ideal mean = 1.

| Case | Mean NEES (ideal) | NEES < χ²₉₅ | 2σ coverage h / vz / az | Mean NIS baro / accel / GNSS | Verdict |
|---|---|---|---|---|---|
| UKF, **matched** assumptions (white-jerk truth, configured white noise), 10 seeds × 120 s | 2.25 (3) | 97.4 % | 96.2 / 99.9 / 95.5 % | 1.08 / 1.00 / – | **approximately consistent, slightly conservative** (qa adds velocity noise the matched truth does not have) |
| Baseline, matched | 10.85 (2) | 46.5 % | 62.0 / 51.2 / – | – | over-confident: a white-acceleration α-β model cannot represent a random-jerk truth |
| UKF, **realistic** errors (S7, S9, S10), 30 runs | 6.49 (3) | 72.1 % | **71.6** / 98.7 / 96.5 % | 0.19 / 0.43 / – | **over-confident in altitude**, consistent-to-conservative in velocity and acceleration |
| Baseline, realistic | 8.06 (2) | 59.0 % | 68.9 / 87.6 / – | – | over-confident |
| UKF with GNSS velocity (S21, no outliers), 10 seeds | 8.84 (3) | 61.3 % | **58.4** / 99.8 / 100 % | 0.19 / 0.14 / 0.16 | altitude over-confident; velocity conservative |

**Why the altitude is over-confident under realistic errors:** the simulated barometer error contains a
0.25 m component **correlated over seconds**, common to both sensors. Sequential white-noise updates
cannot average out an error that is correlated in time, and a 3-state filter cannot separate it from
altitude. The low barometer NIS (0.19) shows the white part of R is generous, while the altitude error
itself is dominated by the correlated part. GNSS velocity makes this worse: it shrinks σ_h further
through the h–v correlation. **Consumers must not treat `altitudeSigma_m` as calibrated** (design §11).
Options for later:
* a barometer-error state (N = 4);
* an output σ floor for unobservable barometric error;
* tuning on real flight data.

No claim of statistical consistency is made beyond the matched case.

---

## 9. Process-model replacement (acceptance criterion 25)

* `nav_process_model_swap_with_nonlinear_mock_model`: the **unchanged** `VerticalNavigationEstimator`
  runs with `MockDragModel`, a test-only **nonlinear** model: CA + quadratic drag 0.5 m/s² at
  10 m/s, its own id `Test` v7. Results:
  * the engine called it > 1 000 times;
  * the output reports `processModel = {Test, 7}` and no `ModelPlaceholder` flag;
  * the same stand-in safety consumer `fastDescentWarning(const NavigationEstimate&)` fires for both
    models;
  * the models' velocity estimates differ by up to 0.588 m/s (the mock is deliberately not the truth's
    physics);
  * final mock vz −9.43 m/s vs the true −10 m/s.
* `nav_engine_reused_with_four_states`: `UnscentedKalmanFilter<4>` with a barometer-bias state (κ = −1,
  so n + κ = 3) separates a 3 m barometer bias from height, using an unbiased reference every 10th
  step. The engine supports a larger state with no code change. The 3-state interface itself does not
  (design §6.3).

---

## 10. Parameter sweeps (tuning seeds 1–5; scenarios 4, 6, 7, 8, 9, 10, 17; 35 runs per value)

Each row changes one parameter from the final default. "DEFAULT" = qj 3, qa 0.02, accel σ 0.35, baro σ 0.7,
gate 4.

| Parameter | Value | h RMSE | vz RMSE | az RMSE | mean NEES | baro rejections |
|---|---|---|---|---|---|---|
| qj (m²/s⁵) | 0.1 / 0.3 / 1 / **3** / 10 | 0.277 / 0.276 / 0.276 / **0.276** / 0.276 | 0.158 / 0.158 / 0.157 / **0.156** / 0.156 | 0.248 / 0.209 / 0.169 / **0.144** / 0.132 | 21.5 / 13.2 / 9.5 / **8.6** / 8.3 | 0 |
| qa (m²/s³) | 0 / 0.005 / **0.02** / 0.05 / 0.2 | 0.336 / 0.287 / **0.276** / 0.274 / 0.273 | 0.180 / 0.155 / **0.156** / 0.167 / 0.198 | 0.144 | 28.4 / 12.4 / **8.6** / 6.9 / 5.1 | 0 |
| accel σ (m/s²) | 0.15 / **0.35** / 0.7 | 0.276 / **0.276** / 0.275 | 0.156 / **0.156** / 0.157 | 0.151 / **0.144** / 0.169 | 9.5 / **8.6** / 8.2 | 0 |
| baro σ (m) | 0.25 / 0.5 / **0.7** / 1.0 | 0.273 / 0.274 / **0.276** / 0.281 | 0.193 / 0.164 / **0.156** / 0.153 | 0.144 | 38.4 / 13.7 / **8.6** / 5.4 | 79 / 0 / **0** / 0 |
| baro gate (σ) | 3 / **4** / 6 | identical (no faults in the tuning set) | | | | 0 |
| Baseline σa (m/s²) | 0.3 / 1 / **3** / 10 | 0.519 / 0.324 / **0.279** / 0.274 | 0.582 / 0.432 / **0.353** / 0.388 | – | 203 / 25.9 / **9.1** / 4.7 | 0 |

**Choices** (conservative, not optimised on the evaluation seeds):
* **qj = 3:** better acceleration and NEES than 1, no h/v cost.
* **qa = 0.02:** 0.005 has a marginally lower vz RMSE but NEES 12.4; larger values add velocity noise.
* **Barometer σ = 0.7:** 0.25 rejects good samples; 1.0 is slightly better in velocity and NEES but worse
  in altitude and widens the fault gate.
* **Accelerometer σ = 0.35:** almost no effect; keeps room for the uncalibrated bias.
* The initial configuration (qj 1, baro σ 0.5) gave vz RMSE 0.164 and NEES 14.7. The final one gives
  0.156 and 8.6. The held-out scenario table (§7) and consistency (§8) use **different** seeds from
  this sweep.

---

## 11. Numerical stability

* 0 numerical faults in every scenario run (including 30 min), every sweep run and the
  consistency runs.
* Covariance symmetry: maximum |Pᵢⱼ − Pⱼᵢ| = 0 in the published estimate (it is symmetrised), and the
  minimum az variance stays > 0.
* **S30 (30 min, 269 900 prediction steps):** σ_h 0.130 → 0.131 m and σ_v 0.158 → 0.158 m/s from
  t = 20 s to the end. Bounded, neither growing nor collapsing.
* Fault injection: NaN from the model, non-positive-definite innovation and result, singular or
  indefinite initial covariance, NaN measurement. All detected, nothing committed, and the estimator
  re-initialises with its counters kept.

---

## 12. Resources, timing, builds

| Item | Result | Basis |
|---|---|---|
| CubeIDE Debug | 0 errors / 0 warnings; text 102 308 (+21 392), data 104, bss 48 032 (+2 008) | headless build of a copy of the tree |
| CubeIDE Release (`-Os`) | 0 errors / 0 warnings; text 59 336 (+9 852), data 104, bss 48 024 (+2 008) | same |
| Release navigation code | 8 088 B (estimator 5 378, engine template instances 1 834, `NavigationConfig::isValid` 560, model 316); baseline estimator not linked (unused by the firmware) | `arm-none-eabi-nm -S`, unique addresses |
| Static RAM | SensorProcessingTask object 16 744 → 18 352 B; HealthMonitorTask 2 788 → 3 104 B; `g_systemHealth` 588 → 672 B | `nm -S` |
| Object sizes (Cortex-M7) | `NavigationEstimate` 312 B, `NavigationInput` 100 B, `VerticalNavigationEstimator` 1 264 B, `NavigationMailbox` 320 B, `UnscentedKalmanFilter<3>` 84 B, model 12 B | `sizeof` compiled for ARM |
| Heap | none (no allocation in `Algorithms/Navigation`; FreeRTOS heap untouched) | code review |
| Stack | `SensorProcessingTask::Run` 208 → 256 B; deepest navigation chain **928 B** of 4 096 B | Release `.su` |
| Strict ARM compile (`-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wdouble-promotion -Wfloat-conversion -Wcast-align`, O0 and O2) | **0 warnings** in navigation code | arm-none-eabi-g++ 14.3 |
| Host execution time | 152 ns per `update()` (best of 5 × 60 000 frames, GNSS on, Apple M4, `-O2`); baseline 8 ns | measured |
| Target execution time | **estimate 50–150 µs** per update at 64 MHz (≤ 1.5 % of 10 ms) | host time × Milestone 3 host→target ratio; **not measured** |
| Mailbox critical section | 312-byte copy (≈ 80 word moves, a few µs, **estimate**) | code |
| Sanitizers | ASan+UBSan 338/338, TSan 338/338, 0 reports | `SANITIZE=1`, `SANITIZE=thread` |

---

## 13. Not validated / needs hardware

1. **No real data of any kind.** Barometer noise and its correlation on a worn device, accelerometer
   bias, attitude error in real turns, GNSS latency and accuracy are all assumptions.
2. **Execution time, stack high-water mark and critical-section duration on the STM32**
   (HARDWARE_INTEGRATION_TODO.md §14).
3. **Measurement latencies** (BMP585 conversion, NEO-M9N navigation) and MCU↔GNSS time alignment.
4. **Tuning on recorded flights** (qj, qa, barometer σ, gates, inflation factors). The current values
   are simulation-based starting points.
5. **The placeholder process model** has no wing knowledge. An aerodynamic model, an accelerometer-bias
   state and a barometer-error state are the expected next improvements.
6. Safety use: none of this has been reviewed or validated as a safety function.
