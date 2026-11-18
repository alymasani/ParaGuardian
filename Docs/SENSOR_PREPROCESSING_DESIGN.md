# Sensor Preprocessing, Calibration and Redundancy: Design

Milestone 2 · Date: 2026-11-17 · Status: implemented, host-tested, and building in STM32CubeIDE.
**Nothing here has run on ParaGuardian hardware.** No PCB or sensors exist yet.
AI assistance: the design, code and this document were produced with Claude Code (Anthropic) and reviewed by the author. See the Git summary.

---

## 1. Architecture

```
 SensorAcquisitionTask (prio 40, 10 ms)          drivers -> SI-unit samples, sensor axes
        |  hub.publish(SensorMessage)  (never blocks)
        v
 SensorDataHub ──► LatestSampleMailbox (existing, health)
        |
        └──► RtosSampleQueue<64>  OverflowPolicy::DropOldest   (new subscriber)
                 |  popWait(50 ms)  -> drain <= 64
                 v
 SensorProcessingTask (prio 32)  ── owns ──  pg::PreprocessingPipeline (portable C++)
                 |
   ingest(msg):  SampleValidator ─► CalibrationController ─► correction ─► AxisAlignment ─► latest-valid store
                 (validity, age,      (StationaryDetector x3,   (gyro - bias,   (sensor -> body FRD)
                  ranges, sequence,    GyroBiasCalibrator x3,    accel cal I/F,
                  timestamps)          BaroReferenceCalibrator   p -> h_rel)
                                       x2)
   buildFrame(): freshness at frame time ─► RedundancyVoter (accel x3, gyro x3, altitude x2) ─► ProcessedSensorFrame
                 |
                 v
 ProcessedFrameMailbox (latest frame, critical section)  ──►  HealthMonitorTask (prio 16) -> g_systemHealth
                                                         ──►  future Madgwick / UKF / safety consumers
```

Layering rule: everything in `Components/Processing/` is plain C++17 with no HAL or FreeRTOS includes and no heap. Host tests compile it unchanged. Only `Components/Tasks/SensorProcessingTask.*` touches FreeRTOS.

## 2. Module responsibilities and public interfaces

| File | Responsibility | Key interface |
|---|---|---|
| `Processing/VectorMath.hpp` | `Mat3f`, vector/matrix helpers; reuses `pg::Vec3f` | `vec::add/sub/norm/median3`, `mat::mul/transpose/determinant/rotationX/Y/Z` |
| `Processing/ProcessingTypes.hpp` | rejection reason bits, `RedundancyStatus`, `CalibrationState`, `CalibrationFailure`, `StationaryStatus`, `TimeDomain`, `BodyFrame` | `reject::*`, index helpers |
| `Processing/ProcessingConfig.hpp` | every tunable value, with its basis | `PreprocessingConfig` and sub-configs |
| `Processing/TimeBase.hpp` | timestamp convention; wrap-safe arithmetic; future sync hook | `timebase::diffMs/isAfter/ageMs`, `IMeasurementTimeModel`, `FixedLatencyTimeModel` |
| `Acquisition/SampleFreshness.hpp` (extended) | `SequenceTracker::observeHeader()` classifies First / InOrder / Gap / Duplicate / OutOfOrder | `observe()` unchanged for existing users |
| `Processing/SampleValidator.*` | per-sample checks and per-reason statistics | `validate(msg, now) -> ValidationResult` |
| `Processing/RunningStats.hpp` | Welford mean/variance; per-segment means | `RunningStats`, `RunningStats3`, `SegmentMeans` |
| `Processing/StationaryDetector.*` | one IMU continuously still for a full window? | `add(t, gyro, accel) -> StationaryStatus`, `report()` |
| `Processing/Calibration.*` | gyro bias; barometer reference; ISA formula; accelerometer calibration interface | `GyroBiasCalibrator`, `BaroReferenceCalibrator`, `baro::relativeAltitude_m`, `AccelCalibration` |
| `Processing/CalibrationController.*` | calibration session state machine | `request`, `onImu`, `onBaro`, `poll`, `status` |
| `Processing/AxisAlignment.*` | sensor → body rotations with validity check; NED→ENU | `setMounting`, `toBody`, `isProperRotation`, `fromAxisMap`, `nedToEnu` |
| `Processing/RedundancyVoter.*` | 2 or 3 sensor consistency check, selection and hysteresis | `update(inputs, newData) -> VoteResult` |
| `Processing/ProcessedSensorFrame.hpp` | output type and `quality::*` flags | `ProcessedSensorFrame` (420 B, trivially copyable) |
| `Processing/ProcessedFrameMailbox.hpp` | bounded hand-over of the newest frame | `IProcessedFrameSink`, `ProcessedFrameMailbox::publish/read` |
| `Processing/PreprocessingPipeline.*` | wires all of the above together | `start`, `ingest`, `buildFrame`, `requestCalibration`, `alignment()` |
| `Tasks/SensorProcessingTask.*` | RTOS task, queue, mailbox | `InitTask(hub)`, `frames()`, `stats()`, `requestCalibration()` |

### Reused rather than duplicated
The design reuses the existing types instead of defining competing ones:
- `Vec3f`, `SensorId`, `SampleHeader` and `ImuSample`/`BaroSample`/… from `SensorTypes.hpp`
- `SensorMessage` and `headerOf()`
- `SensorDataHub` and `OverflowPolicy::DropOldest`
- `RtosSampleQueue`, `ICriticalSection`/`RtosCriticalSection`
- `SequenceTracker`, which was extended rather than replaced

## 3. Data types

- **`ValidationResult`** holds:
  - `reasons`, a bit mask of 23 possible reasons;
  - `ageMs`, `sequenceGap` and `sequence`;
  - `accepted`, plus per-channel `accelUsable` and `gyroUsable`.

  A saturated gyro therefore does not discard a good accelerometer reading.
- **`VoteResult`, `VectorEstimate` and `ScalarEstimate`** carry:
  - `value` and `valid`;
  - `status` (`Unavailable`, `Healthy`, `Degraded` or `Ambiguous`);
  - the masks `candidateMask`, `contributingMask`, `excludedMask` and `outlierMask`, one bit per sensor;
  - `timestampMs` of the newest contributing sample, plus `ageMs` and `sampleSpreadMs`;
  - `maxDisagreement`.
- **`ProcessedSensorFrame`** contains:
  - the fused specific force, angular rate (with `gyroDtMs` and `gyroDtValid`) and barometric altitude;
  - per-IMU and per-barometer channel information;
  - magnetometer, GNSS and LiDAR pass-through;
  - calibration state;
  - `qualityFlags`, the 19 bits in `quality::*`;
  - lost and rejected sample counters.

  **A value means nothing unless its `valid` flag is true. Missing data is never written as 0.**

## 4. Coordinate conventions

- **Sensor frame.** Each chip's own datasheet X/Y/Z axes. This is what drivers publish (unchanged).
- **Body frame: FRD**, right-handed. X points forward (the way the pilot faces), Y right, Z down. At rest and level, specific force = (0, 0, −g).
- **Mounting.** `v_body = R_body_from_sensor · v_sensor`. The same R is applied to an IMU's accelerometer and its gyro.
  - Every mounting is the identity placeholder with `verified = false`.
  - Every frame carries `quality::MountingUnverified` until real values are entered.
  - **The PCB axes are not known.**
- **Validity of R.** R must satisfy `R·Rᵀ = I` (tolerance 1e-4) and `det R = +1`.
  - A single axis sign flip has det −1, a mirror image; it is rejected.
  - Two sign flips make a 180° rotation, which is accepted.
- **Navigation frame (later): ENU.**
  - `v_NED = R_nb · v_body`, where `R_nb` comes from the future attitude filter.
  - `v_ENU = C · v_NED`, with `C = [[0,1,0],[1,0,0],[0,0,−1]]` (`kEnuFromNed`).
  - C is a proper rotation (det +1) and its own inverse.
  - FRD/NED was chosen because it is the standard aerospace pairing for `R_nb`. The only ENU step is this one explicit, tested matrix, so the two conventions are never mixed silently.
- **Rotation builders.** `mat::rotationX/Y/Z` are single-axis active rotations about fixed axes. `R = Rz·Ry·Rx` applies X first.
  - There is no Euler-angle API, to avoid ambiguity about rotation order.
  - `fromAxisMap` builds R from a "body axis k = ±sensor axis" table.

## 5. Calibration state machines

### Session (`CalibrationController`)

```
NotCalibrated --request--> Collecting --(>=1 IMU Stationary AND (all seen IMUs Stationary OR window+1 s))--> Valid
                              |   ^
          motion on ANY IMU   +---+  (all detectors, gyro and baro accumulators restart; window starts after the moving sample)
                              |
                              +--(session > 60 s)--> retryOnFailure ? new session (failedSessions++) : Failed
```

- The pipeline requests a session at start-up (`calibrateAtStartup`). Any task can request another through `SensorProcessingTask::requestCalibration()`.
- **Nothing is calibrated without inertial confirmation of stillness.** A steady barometer alone fails with `NoStationaryImu`.
- **Gyro:** a new result replaces the old one on success. On failure the previous result is kept, because a chip's bias is still the best correction available.
- **Barometers:** references are replaced per session. On failure a barometer's reference becomes **invalid**, never kept from an older session, because the two barometers must share one zero.
- A timed-out session keeps every previous result. They were never replaced, so they remain mutually consistent.

### Stationary detector (per IMU)

```
InsufficientData --(window >= 5 s, >= 400 samples, statistics quiet)--> Stationary
      ^   |
      |   +--(instantaneous limit violated / window statistics noisy or drifting)--> Moving --next sample--> InsufficientData
      +--(gap > 50 ms: restart, counted as gapRestarts)
```

## 6. Redundancy decision logic (`RedundancyVoter::update`)

1. **Candidates** are sensors that are fresh, valid, calibrated and body-aligned, excluding any more than `maxSkewMs` older than the newest candidate.
2. **Pairwise agreement:** two sensors agree when `|vᵢ − vⱼ| ≤ abs + rel · max(|vᵢ|,|vⱼ|)`.
3. **Voters** are the candidates that are not excluded. With 3 voters, the **outlier** is the one that disagrees with both others while those two agree with each other.
4. **Output:**

   | Contributors | Agreement | Output | Status |
   |---|---|---|---|
   | 3 | all 3 pairs | mean | Healthy |
   | 3 | 2 of 3 pairs (chain) | component-wise median | Degraded, `partialAgreement` |
   | 3 | no pair | none | Ambiguous |
   | 2 | agree | mean | Healthy if 2 is every configured sensor (barometers), otherwise Degraded |
   | 2 | disagree | none | Ambiguous: **neither is blamed** |
   | 1 | — | that sensor | Degraded; Ambiguous instead if a re-qualifying sensor disagrees with it |
   | 0 candidates | — | none | Unavailable |

5. **Persistence and hysteresis.** These counters advance only when there is new data.
   - A sensor that is the outlier 5 times in a row is **excluded**.
   - An excluded sensor that agrees with the output 20 times in a row is re-included.
   - A sensor that is absent for 10 updates (5 for barometers) must re-qualify the same way when it returns.
   - A single noisy sample is left out of *that* sample's value but never changes the selection.
6. **Barometers** are compared as altitude relative to their *own* reference from the *same* session. A constant offset between the two devices therefore cancels, and the zero is never switched silently. When the contributing set changes, `quality::BaroSourceChanged` is raised for that frame.

**Stated limits:**
- Majority voting assumes independent faults. Two IMUs that fail the same way out-vote the healthy one; the test `prep_e2e_two_imus_fail_same_way_common_mode_limit` demonstrates this.
- Two barometers can detect disagreement but cannot isolate the faulty one.
- Voting shows that sensors are consistent with each other, not that they are correct.

## 7. Timestamp assumptions

- `timestampMs` is the MCU time when the read completed (`HAL_GetTick()`, 1 ms resolution). It is not the physical measurement instant.
  - `IMeasurementTimeModel` / `FixedLatencyTimeModel` is the hook for adding per-sensor latency.
  - All latencies are 0 because none has been measured.
- Each sensor keeps its own timestamp. Samples from the same cycle are **not** assumed simultaneous: voting only compares samples within `maxSkewMs`.
- Nothing is interpolated, extrapolated or invented. A missing sample is reported as not valid.
- **Wraparound.** Every comparison uses `(int32)(a − b)`, which is valid for intervals below 2³¹ ms (24.8 days). All intervals compared here are seconds or less. The tests `prep_time_*` and `prep_e2e_runs_across_timestamp_wraparound` cover this.
- **GNSS.** `gnss.timeOfWeek_ms` (receiver time) and `gnss.timestampMs` (MCU arrival time) are both passed through and never combined. A mapping would need the receiver's time-pulse pin, which has not been verified.
- **Staleness.** The limits are set per sensor in `ValidationConfig::maxAgeMs` and applied twice: at ingest, and again when the frame is built.

## 8. Fault handling

| Fault | Where detected | Effect |
|---|---|---|
| Invalid flag, NaN/Inf, out of range, saturation, "not new" status | `SampleValidator` | sample or channel rejected; reason bit recorded per sensor and per reason |
| Stale / future / backwards timestamp; duplicate or out-of-order sequence | `SampleValidator` | rejected; counted |
| Lost samples (queue DropOldest, bus) | sequence gaps | `samplesLostSinceLastFrame`, `quality::SamplesLost` |
| Sensor silent | freshness at frame time | channel not fresh → not a candidate → Degraded/Unavailable |
| Sensor biased or failing | `RedundancyVoter` | outlier → excluded (persistence) → re-included after recovery (hysteresis) |
| Barometers disagree | `RedundancyVoter` (n=2) | altitude invalid, `Ambiguous` |
| Device never still | `CalibrationController` timeout | `TimedOut`, retry; previous results kept |
| Invalid mounting matrix | `AxisAlignment::setMounting` | rejected, previous kept, counted |
| No data at all | 50 ms idle timeout in the task | a frame is still produced, flagged `ImuUnavailable` / `BaroUnavailable` / `NoNewImuData` |

Free fall (near-zero specific force) is **not** a fault. Any attitude is accepted.

## 9. Parameter justification (none tuned on hardware)

| Parameter | Value | Basis | How to re-derive |
|---|---|---|---|
| max age IMU / baro / mag / GNSS / LiDAR | 30 / 60 / 60 / 2000 / 50 ms | about 3× the nominal period; GNSS uses the driver's stale timeout | measured cycle jitter (`maxCycleTimeMs`) |
| stationary window | 5 s | the start-up window proposed earlier | trade-off: bias error ∝ 1/√n |
| min samples | 80 % of window/period = 400 | allows intermittent loss without accepting a sparse window | — |
| max gap | 50 ms | 5 IMU periods; a longer gap was not observed | — |
| instantaneous \|ω\| | 0.10 rad/s (5.7 dps) | must exceed **uncalibrated** MEMS bias (of the order of 1 dps; datasheet to confirm) | measured zero-rate offsets of the batch |
| \|\|a\|−g\| | 0.6 m/s² | accelerometer offset (tens of mg) plus local-gravity variation 9.78–9.83 | — |
| gyro std | 0.01 rad/s | about 10× expected MEMS white-noise RMS at this bandwidth; human sway exceeds it | record 60 s still, take 3–5× measured std |
| \|a\| std | 0.05 m/s² | about 5 mg | same |
| segment spreads | gyro 0.005 rad/s; accel per axis 0.05 m/s² (≈0.3° change in gravity direction) | catches warm-up drift, slow rotation, slow pushes | — |
| gyro bias plausibility | ≤ 10 dps | a larger "bias" is motion or a broken sensor | — |
| baro std / segment spread | 10 Pa / 12 Pa (≈0.85 m / 1 m) | rejects carrying the device or strong gusts; weather is about 0.05 Pa in 5 s | measured BMP585 noise with the chosen oversampling |
| accel agreement | 1.0 m/s² + 5 % | offsets plus lever arm (ω²r = 0.5 m/s² at 5 rad/s, r = 2 cm) plus skew | measured offsets and PCB IMU spacing |
| gyro agreement | 0.05 rad/s + 5 % | post-calibration residual plus sensitivity tolerance of a few % | — |
| baro agreement | 2.0 m | about 24 Pa; covers noise and temperature effects | — |
| exclude / include / requalify | 5 / 20 / 10 (baro 3 / 10 / 5) | 50 ms to drop a bad IMU, 200 ms to readmit | — |
| skew | 15 ms IMU, 30 ms baro | IMUs read in the same cycle; barometers can be one cycle apart | — |
| barometric model | 44330·(1−(P/P0)^0.1903) (specified); optional `IsaAtReference` | see §10 | — |

## 10. Barometric formula finding

The specified formula uses 44330 m, which is T₀/L for the ISA **sea-level** temperature (288.15 K / 0.0065 K/m). Relative to a reference at ISA altitude h₀, it overstates height differences by a factor of 44330/(44330 − h₀). That is **+3.5 % at a 1500 m take-off**: a 300 m climb reads 310.5 m. The test `prep_baro_formula_high_takeoff_scale_error_and_isa_option` shows this.

`BaroCalibrationConfig::altitudeModel = IsaAtReference` uses the scale height 44330·(P0/101325)^0.1903, which is exact within ISA. **The specified formula remains the default.** Both formulas ignore the real air temperature, and a ±20 K deviation from ISA is a ±7 % scale error in either one. That temperature error, together with weather drift (about 8 m per hPa), is the real limitation, and the future estimator must handle it with GNSS.

## 11. Memory and timing estimates

| Item | RAM (static .bss) |
|---|---|
| `SensorProcessingTask` object | 16 144 B: stack 4096, queue storage 64 × 88 = 5632, pipeline ≈5.3 KB, two frames 2 × 420, TCB and queue control |
| `HealthMonitorTask` frame copy | +420 B |
| `g_systemHealth` | 476 → 532 B |
| Total firmware .bss (Release) | 28 264 → 45 200 B of 512 KB RAM_D1 |
| FreeRTOS heap | unchanged: the new task, queue and frames are all static |

| Flash (Release text) | Bytes |
|---|---|
| Processing objects + task | 13 557 |
| newlib `pow` + its exp/log tables | ≈ 7 800 |
| Total firmware | 16 540 → 37 744 of 2 MB |

**Execution.** All loops are bounded by compile-time constants (3 IMUs, 2 barometers, 8 segments, 23 reason bits, queue depth 64). There is no allocation and no unbounded wait other than the blocking `popWait`. Measured host time: 0.24 µs per 10 ms cycle (5.5 messages plus one frame, Apple M-series, -O1). Target estimate: 0.1–0.4 ms per cycle, i.e. 1–4 % CPU at 64 MHz with caches off and flash latency 1 (as configured in `main.c`). **Not measured.**

**Stack.** Compiler `-fstack-usage`: the deepest chain is `Run` 152 B → `buildFrame` → `fillImu` 320 B → `RedundancyVoter::update` 104 B, about 0.65 KB plus the RTOS context frame, against a 4 KB stack.

## 12. Integration with current acquisition

- No driver, `SensorAcquisitionTask`, `AcquisitionScheduler` or `SensorDataHub` code changed. The processing task is one more hub subscriber, using `DropOldest`.
- `paraguardian_create_tasks()` now runs: acquisition sources → acquisition `InitTask` → `SensorProcessingTask::InitTask(acquisition.hub())` → health monitor, which also watches the `SensorProc` stack.
- Priorities: acquisition 40 > processing 32 > defaultTask 24 > health 16. This is checked by `static_assert` in `TaskConfig.hpp`.
- The only cost on the acquisition side is one 88-byte `xQueueSend` per message. When the queue is full it adds an `xQueueReceive` too (DropOldest), and neither ever blocks.
- `ProcessedFrameMailbox::publish/read` copies 420 B inside `taskENTER_CRITICAL`. That is estimated at a few µs; during it, interrupts at priority ≥ 5 (including the TIM6 tick) wait. Measure this on hardware; a double buffer with a sequence counter is the alternative if it matters.
