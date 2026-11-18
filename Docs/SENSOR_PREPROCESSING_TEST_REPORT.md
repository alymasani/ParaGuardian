# Sensor Preprocessing: Test Report

Date: 2026-11-17 · **Hardware verification: none.** Everything below was run on a laptop (host tests) or is a firmware *build* result. No sensor, PCB or STM32 board was used.

## 1. Summary

| Item | Result |
|---|---|
| Host tests, AddressSanitizer + UndefinedBehaviorSanitizer (`sh Tests/run_host_tests.sh`) | **205 tests, 205 passed, 0 failed** |
| Host tests, ThreadSanitizer (`SANITIZE=thread sh Tests/run_host_tests.sh`) | **205 passed, 0 ThreadSanitizer warnings** |
| Host tests, no sanitizers (`SANITIZE=0`) | 205 passed |
| Existing regression tests (milestone 1 + hardening) | **120 of 120 still pass**, unmodified |
| New preprocessing tests | 85 (5 files) |
| STM32CubeIDE 2.2.0 headless build, Debug | **0 errors, 0 warnings** |
| STM32CubeIDE 2.2.0 headless build, Release | **0 errors, 0 warnings** |
| Strict extra pass, arm-none-eabi-g++ 14.3 `-O2 -Wall -Wextra -Wshadow -Wpedantic` on all new/changed .cpp | 0 warnings in ParaGuardian code; only the two existing SoarOS header warnings (`Command.hpp` -Wshadow, `CubeDefines.hpp` -Wsized-deallocation) |
| Link check | no undefined symbols; no `malloc`/`new`/`pvPortMalloc` references from any new object file |
| SoarOS | unchanged (`426c1efe…`, clean) |

Host compiler: Apple clang 17.0.0, `-std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Wshadow -Werror -pthread` plus the sanitizer flags.

## 2. Tests by module

| File | Tests | Covers |
|---|---|---|
| `test_preprocessing_validation.cpp` | 15 | valid sample; invalid flag; NaN/Inf; per-channel saturation; out-of-range and "not new"; **free fall accepted**; per-sensor staleness; future and backwards timestamps; duplicates, out-of-order and gaps; id/type mismatch and empty message; barometer/LiDAR/magnetometer/GNSS checks; wraparound arithmetic; validator across the wrap; latency model; `SequenceTracker` classification and backward compatibility |
| `test_preprocessing_calibration.cpp` | 27 | **Stationary:** perfectly still; realistic noise with uncalibrated bias; constant rotation; **slow constant rotation (0.05 and 0.005 rad/s)**; translational acceleration; short spike; intermittent missing data and gaps; slowly drifting gyro (accepted) vs fast drift (rejected); noisy window. **Gyro:** known offsets with noise (error metric); insufficient samples keeps the previous result; too noisy or implausible fails; accelerometer interface is identity. **Session:** 3 IMUs with different signed biases; missing IMU; interrupted by motion; timeout with and without retry; barometer alone never calibrates. **Barometer:** formula zero and known points; high take-off scale error and ISA option; zero, negative and NaN; mean, count and variance; unstable/drifting vs weather drift; insufficient and invalid inputs; two barometers with an offset; one barometer missing; samples before motion discarded |
| `test_preprocessing_alignment.cpp` | 9 | identity placeholder flagged unverified; 90° rotations; 180° and two-axis sign change; single-axis sign change rejected as a mirror; axis permutation; magnitude preservation (200 random vectors); invalid matrices (scaled, sheared, singular, NaN, sensor with no vector output); three differently mounted IMUs agree in the body frame; NED→ENU and the FRD gravity sign |
| `test_preprocessing_redundancy.cpp` | 16 | all agree; one invalid; biased → outlier → excluded after 5; isolated spikes don't change selection; two disagree; all disagree; chain partial agreement (median); recovery after 20; no persistence without new data; time skew; no candidates; relative tolerance; barometers agree, disagree (neither blamed), one unavailable, returning barometer re-qualifies or conflicts |
| `test_preprocessing_pipeline.cpp` | 18 | end to end from simulated `SensorMessage` to `ProcessedSensorFrame`: start-up calibration then calibrated body outputs; climb tracking with offset barometers; weather drift; biased IMU excluded and recovered; common-mode failure (limit); all IMUs disagree; barometer disagreement and loss/recovery; all sensors stop; delayed (skewed vs stale); NaN/invalid with reasons; random outliers; mixed rates with missing reads and GNSS time domain; **timestamp jitter**; **frame mailbox concurrency (ThreadSanitizer)**; **queue loss with the real hub using DropOldest**; repeated calibration request; **run across the 2³² ms wrap**; memory footprint and host timing |

### Simulation harness (`Tests/sim/SensorSimulator.hpp`)

The harness provides:
- deterministic noise (its own xorshift64* and Box–Muller, not `<random>` distributions, whose output varies between standard libraries);
- per-sensor rates and phases, and per-sensor sequence numbers;
- known gyro bias, accelerometer offset and mounting rotation per IMU;
- truth in the body frame, with each chip seeing it through the transpose of its mounting;
- pressure from a true altitude via the **inverse** ISA formula, which independently checks the pipeline's forward formula;
- weather drift, noise, and these faults: invalid flag, NaN, saturation, step bias, missing reads (no sequence step), lost samples (sequence gap), delayed and jittered timestamps, outliers, absent sensors and recovery.

Scheduling is relative to the first step so that runs across the 2³² wrap stay correct.

## 3. Calibration accuracy (simulated)

| Scenario | Truth | Result |
|---|---|---|
| Single IMU, 500 samples, σ = 0.0015 rad/s, bias (0.0123, −0.0311, 0.0047) rad/s | — | bias error **1.99e-4 rad/s (0.011 dps)**; reported standard error 6.5e-5 rad/s per axis (theory σ/√n = 6.7e-5); noise std estimate within 0.0002 of σ |
| Three IMUs, different signed biases, σ = 0.0015 rad/s, full session from `SensorSimulator` | — | worst bias error **1.34e-4 rad/s**; session complete at 5.0 s |
| Fused angular rate after calibration, rotating at (0.20, −0.10, 0.30) rad/s, three IMUs with biases up to 0.031 rad/s and noise 0.0012 rad/s | — | worst error **2.23e-3 rad/s** over 100 frames |
| Barometer reference, 250 samples, σ = 1.5 Pa | 95 000 Pa | within 0.4 Pa; std estimate within 0.25 Pa |
| Two barometers with a 35 Pa offset | 35 Pa | reference difference within 0.6 Pa |
| 100 m climb at 5 m/s, offset barometers | climb | worst altitude error **0.22 m**; status Healthy throughout |
| Weather: −1 hPa over 60 s, device not moving | 0 m | reads **+8.3 m** (expected and documented limitation) |
| Take-off at 1500 m, climb 300 m, specified formula | 300 m | 310.5 m (+3.5 %); `IsaAtReference` option gives 300.0 m |

## 4. Fault detection results

| Fault | Detected as | Time to detect / effect |
|---|---|---|
| IMU gyro step fault of 0.3 rad/s | outlier from the first frame; **excluded after 5 frames (50 ms)** | output never moved more than 0.005 rad/s from truth; re-included about 200 ms after the fault cleared |
| Random 6 m/s² outliers, 3 % of samples | per-sample outlier | worst output error < 0.05 m/s² over 10 s; never excluded |
| Two IMUs with the same fault (common mode) | healthy IMU voted out | output follows the faulty pair. **Limit demonstrated, not solved** |
| Two IMUs disagree with each other and the third | `Ambiguous` | no output |
| Barometer +60 Pa (about 5 m) | `Ambiguous` | altitude invalid; neither barometer blamed |
| Barometer missing / returns | `Degraded` (1 contributor) / re-qualifies | `BaroSourceChanged` flagged exactly once on rejoin |
| All sensors stop | `Unavailable` | frames still produced; `ImuUnavailable`, `BaroUnavailable`, `NoNewImuData` |
| NaN on IMU0, invalid flag on Baro0 | rejected with `NonFinite` / `FlagInvalid` | the other sensors carry the output |
| Timestamp 20 ms behind / 45 ms behind | skewed (not compared) / stale (rejected) | `Degraded` with two contributors |
| Device never still (60 s) | `TimedOut` | session retried; previous results kept |
| Slow constant rotation 0.05 / 0.005 rad/s | `AccelDrift` | **not** accepted as stillness (see §6) |

## 5. Queue loss interaction

The real `SensorDataHub` feeds a 16-deep `BoundedSampleQueue` with DropOldest. The consumer drains only every 50 ms, simulating a stalled processing task, for 8 s.
- **Hub:** 640 overwrites.
- **Pipeline:** saw 636 lost samples as sequence gaps and rejected 480 as stale.
- **Output:** the specific force was valid after **every** drain, because the newest data always survived.
- **Calibration** still completed, at 6.69 s instead of 5.0 s: the window stretches until 400 samples are present rather than accepting a sparse one.

## 6. Defects found by these tests and fixed during the milestone (honest record)

1. **Stationary detector accepted a slow constant rotation as "still"**, found while writing `prep_stationary_slow_constant_rotation_not_mistaken_for_bias`.
   - A constant rate below 0.1 rad/s passes every gyro test (it looks like a bias), and `|a|` stays at g.
   - Fix: segment-drift test on **each accelerometer axis**. Gravity's direction in sensor axes moves.
2. **Wrong claim in a comment**: NED→ENU has determinant **+1**, not −1. The comment was corrected; the test now also asserts it is a proper rotation.
3. **Barometric formula scale at high take-off sites** (+3.5 % at 1500 m), found by the test with an independent expected value. The specified formula is kept as the default; the `IsaAtReference` option was added and documented.
4. Calibration window restart: the new window now begins the millisecond *after* the moving sample, so a barometer sample read at the same millisecond is excluded.
5. **Simulator bug**, not production code: `now % period` scheduling broke across the 2³² wrap.
6. Test expectations corrected, not production code:
   - an excluded IMU appears in `excludedMask`, not `outlierMask`;
   - under heavy loss, calibration completes *later* rather than never.

No production code was changed only to make a test pass. Each change above follows from a stated requirement.

## 7. Performance

| Measurement | Value | Notes |
|---|---|---|
| Host, no sanitizers | **0.24 µs per 10 ms cycle** (≈5.5 messages + 1 frame), 0.043 µs per message | Apple M-series, -O1, 2000 cycles |
| Host, ASan+UBSan | 0.77 µs per cycle | instrumentation overhead |
| Target estimate | 0.1–0.4 ms per cycle (1–4 % CPU) | 64 MHz HSI, caches not enabled in `main.c`, flash latency 1; **not measured** |
| `sizeof` (host, close to target) | pipeline 5 328 B, frame 420 B, validator 548 B, calibration controller 3 408 B, voter 88 B | |
| Target stack (`-fstack-usage`) | about 0.65 KB worst chain of 4 KB | `Run` 152 → `fillImu` 320 → `RedundancyVoter::update` 104 B |

## 8. Build status and size

| Config | Before (hardening) | After | Δ |
|---|---|---|---|
| Debug | text 28 304, data 24, bss 28 264 | **text 61 552, data 104, bss 45 208** | +33 248 / +80 / +16 944 |
| Release | text 16 540, data 24, bss 28 264 | **text 37 744, data 104, bss 45 200** | +21 204 / +80 / +16 936 |

Release text growth breaks down as preprocessing objects 13.6 KB plus `pow` and its tables about 7.8 KB. The bss growth is the processing task object (16.1 KB): stack 4 KB, queue 5.5 KB, pipeline and frames. The FreeRTOS heap is not used by any new code. The +80 B of data comes from the C library pulled in with the math functions.

## 9. Known limitations

- **No hardware:** every threshold is a documented starting point, not a tuned value. Sensor noise, biases, the real cycle time, stack high-water marks and the critical-section length are all unmeasured.
- **Placeholder mountings:** every mounting is an identity placeholder, and every frame says so (`MountingUnverified`).
- **Accelerometer:** offsets and scale are not calibrated (single-pose limitation, see the walkthrough §E).
- **Magnetometer:** no hard- or soft-iron calibration.
- **Barometric altitude:** drifts with weather and depends on air temperature (±7 % scale for ±20 K).
- **Voting:** majority voting cannot handle common-mode failures, and two barometers cannot isolate a faulty one.
- **Time model:** timestamps are read-completion times with 1 ms resolution, latencies are assumed to be 0, and there is no MCU↔GNSS time mapping.
- **Start-up calibration:** needs about 5 s of stillness. Until then there is no gyro output, and the frame says so (`GyroNotCalibrated`). The session completes at the first 5 s still window; a session that finds none within 60 s is counted as failed and immediately restarted.
- **Host timing tests** model time only through timestamps, not execution time on the MCU.
