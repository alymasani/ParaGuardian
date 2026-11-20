# Components/Algorithms

Portable algorithm code (no HAL, no FreeRTOS, no heap). Compiled into the
firmware by the existing `Components` source entry and into the host tests by
`Tests/run_host_tests.sh`.

## Attitude/ (Milestone 3: implemented)

Madgwick attitude estimation + gravity compensation. See
`Docs/MADGWICK_ATTITUDE_DESIGN.md` and `Docs/MADGWICK_ATTITUDE_WALKTHROUGH.txt`.

| File | Role |
|---|---|
| `Quaternion.hpp` | scalar-first `Quaternion`, Hamilton product, `q_ENU_from_body` convention, Euler (display only) |
| `MadgwickFilter.hpp/.cpp` | the original Madgwick gradient-descent step (gyro + accel + optional mag) |
| `AttitudeEstimator.hpp/.cpp` | initialisation state machine, gyro-driven timing, gating, recovery, magnetometer policy |
| `GravityCompensator.hpp/.cpp` | `a_ENU = R(q) f_body + (0, 0, -g)` |
| `AttitudeConfig.hpp`, `AttitudeTypes.hpp` | parameters (with their basis) and input/output types |
| `AttitudeFrameAdapter.hpp` | `ProcessedSensorFrame` -> `AttitudeInput` |
| `AttitudeMailbox.hpp` | latest-value hand-over to other tasks |

The firmware runs it inside `SensorProcessingTask`; read the result with
`SensorProcessingTask::Inst().attitude().read(estimate)` and use only fields whose
validity flag is set (`orientationValid`, `linearAccel.valid` / `.reliable`,
`yawReferenced`), checking `qualityFlags`.

## Navigation/ (Milestone 4: implemented, simulation-validated only)

Vertical Unscented Kalman Filter on `x = [h, vz, az]` (positive up), fusing the barometric
altitude, the attitude estimator's gravity-compensated vertical acceleration and (optionally)
GNSS vertical velocity. See `Docs/VERTICAL_UKF_DESIGN.md` and `Docs/VERTICAL_UKF_WALKTHROUGH.txt`.

| File | Role |
|---|---|
| `NavMath.hpp` | fixed-size `Mat`/`Vec`, Cholesky, SPD inverse (no heap, templated on size) |
| `UnscentedKalmanFilter.hpp` | generic UKF engine `UnscentedKalmanFilter<N>`: sigma points, predict, evaluate/apply, numerical checks; no physics |
| `IVerticalProcessModel.hpp` | **the replacement boundary** for the process model (3 states) |
| `ConstantAccelerationModel.hpp/.cpp` | TEMPORARY placeholder model (white jerk + unmodelled acceleration noise) |
| `VerticalNavigationEstimator.hpp/.cpp` | sample selection, timing, initialisation, gating, recovery, state machine, output |
| `BaroSampleTracker.hpp` | "is this barometer vote a new observation?" (each sample used once) |
| `NavigationTypes.hpp`, `NavigationConfig.hpp` | `NavigationInput`, **`NavigationEstimate`** (the safety-facing output), parameters |
| `NavigationInputAdapter.hpp` | `ProcessedSensorFrame` + `AttitudeEstimate` -> `NavigationInput` |
| `NavigationMailbox.hpp` | latest-value hand-over to other tasks |
| `BaroAlphaBetaEstimator.hpp/.cpp` | barometer-only baseline (comparison / possible fallback; not used by the firmware) |

Read the result with `SensorProcessingTask::Inst().navigation().read(estimate)`. Use only fields
whose `*Valid` flag is set; check `state`, `qualityFlags` and the sigmas. Altitude is relative to
the barometric reference, not above ground. To replace the process model, implement
`IVerticalProcessModel` and change the type of `navModel_` in `SensorProcessingTask.hpp`.

## Flight/, Landing/, Safety/ (Milestone 5: implemented, simulation-validated only)

Flight-phase detection, LiDAR ground clearance and landing / descent warnings, plus the common
safety-event infrastructure. See `Docs/FLIGHT_LANDING_DESIGN.md` and `Docs/FLIGHT_LANDING_WALKTHROUGH.txt`.
All thresholds are provisional development values; the LiDAR mounting is an unverified placeholder.

| File | Role |
|---|---|
| `Safety/SafetyInput.hpp` | bounded input (`SafetyInput`) + `safetyInputFrom(frame, attitude, navigation)` |
| `Flight/FlightStateTypes.hpp`, `Flight/FlightStateDetector.hpp/.cpp` | evidence-based phase state machine (Unknown, Ground, TakeoffCandidate, Flying, Approach, LandingCandidate, Landed) |
| `Landing/GroundClearanceTypes.hpp`, `Landing/GroundClearanceEstimator.hpp/.cpp` | TF02-Pro slant range + attitude -> clearance (flat-terrain assumption flagged), closure rate, time to contact |
| `Landing/LandingSafetyTypes.hpp`, `Landing/LandingSafetyAnalyzer.hpp/.cpp` | warnings with persistence, hysteresis, cooldown, escalation; `SafetySnapshot` |
| `Safety/SafetyEvent.hpp`, `SafetyEventEmitter.hpp`, `SafetyEventQueue.hpp`, `SafetySnapshotMailbox.hpp` | transition-only events, numbering, bounded priority-aware queue, snapshot hand-over |
| `Safety/SafetyMonitor.hpp/.cpp` | runs clearance -> phase -> warnings each frame; `SafetyConfig` |

Read the state with `SensorProcessingTask::Inst().safety().read(snapshot)`. The ONE consumer of
`SensorProcessingTask::Inst().safetyEvents()` is EmergencyAlertTask (Milestone 6); the crash detector
emits into the same queue (`ImpactCandidate`, `CrashSuspected`).

## Crash/, Emergency/, Safety/ dispatcher (Milestone 6: implemented, simulation-validated only)

Possible-impact / suspected-crash detection, the incident / SOS lifecycle and the alert dispatcher.
See `Docs/CRASH_EMERGENCY_DESIGN.md` and `Docs/CRASH_EMERGENCY_WALKTHROUGH.txt`. A "suspected crash" is
NOT a confirmed crash and says nothing about injury; all thresholds and countdowns are placeholders.

| File | Role |
|---|---|
| `Crash/CrashTypes.hpp`, `Crash/CrashConfig.hpp` | states, evidence bits, `CrashInput`, `CrashStatus`, `IncidentContext`, parameters |
| `Crash/CrashDetector.hpp/.cpp` | corroborated IMU impulse (per-IMU peaks BEFORE smoothing) + bounded 8 s context + post-impact observation -> CrashSuspected / Rejected |
| `Crash/CrashInputAdapter.hpp` | `crashInputFrom(frame, attitude, snapshot, navigation)`, `updateIncidentContext()`, `CrashReportMailbox` |
| `Emergency/EmergencyTypes.hpp`, `EmergencyConfig.hpp` | emergency states, SOS delivery stages (never conflated), `EmergencyStatus` |
| `Emergency/EmergencyCommand.hpp` | integrity-checked commands (manual SOS, cancel, ack, reset), transport / phone reports, `BoundedInbox` |
| `Emergency/EmergencyManager.hpp/.cpp` | countdown, cancellation, manual SOS, request creation, retries / backoff, acknowledgment correlation, re-arm, persistence hooks |
| `Emergency/SosRequest.hpp/.cpp`, `SosRequestMailbox.hpp` | versioned 64-byte SOS request (CRC-32), legacy mapping, the reliable pending-request record |
| `Emergency/ISosTransport.hpp`, `IEmergencyStore.hpp` | transport and persistence interfaces (firmware: unconfigured / null) |
| `Safety/AlertTypes.hpp`, `AlertPriorityPolicy.hpp` | pilot alerts, the single priority / pattern table |
| `Safety/SafetyEventDispatcher.hpp/.cpp` | THE consumer of the safety events; active alerts, priorities, acknowledgment, audio / LED scheduling |
| `Safety/ValueMailbox.hpp` | generic latest-value mailbox |
| `../Output/Audio/AudioAlertScheduler.*`, `../Output/LED/LedPatternScheduler.*` | non-blocking audio / LED pattern schedulers over `IAudioAlert` / `IStatusLed` |

Runtime: the detector runs in `SensorProcessingTask`; the manager and dispatcher run in
`EmergencyAlertTask` (Components/Tasks), which advances countdowns and retries even without sensor frames.
Commands: `EmergencyAlertTask::Inst().submitCommand(makeEmergencyCommand(...))`.

## Not implemented (out of scope so far)

Status as of Milestone 8 (2026-11-19). The Android companion app (M7) and the portable flash persistence, flash
logging, power, PCM audio and GPIO LED code (M8) **are implemented** (see the M7 and M8 documents). What remains:
the paraglider aerodynamic process model, the physical backends that need the schematic (W25Q128 SPI wiring,
CYW20822 module transport, I2S/SAI audio, LED pins, battery ADC / charger / enable pins) and flare guidance.
Future algorithm work must:

- take their input from `pg::SafetySnapshot` / `pg::SafetyEvent`, `pg::CrashReport`, `pg::EmergencyStatus`,
  `pg::NavigationEstimate`,
  `pg::AttitudeEstimate` and (for raw data) `pg::ProcessedSensorFrame`; never from the internals of
  the UKF or the safety algorithms;
- convert body (FRD) quantities to ENU only through the attitude quaternion
  (`quat::rotate(q_ENU_from_body, v)`) or `pg::nedToEnu()`;
- never call HAL or driver functions directly;
- live in portable C++ so they can be unit-tested on a laptop.

## Milestone 7 communications

The BLE bridge transports the existing M6 emergency request and reports phone stages back to EmergencyManager. See the [BLE and Android architecture](../../Docs/BLE_ANDROID_ARCHITECTURE.md) and the [companion project](../../Android/ParaGuardian/README.md) for setup and validation.

## Milestone 8 persistence, logging and power

`EmergencyManager` now reports durability honestly (`EmergencyStatus::persist`: queued is not saved) and keeps
incident ids unique across resets through the `Components/Storage` journal (written by `StorageTask`). Safety
events, emergency transitions and navigation summaries are logged through `FlightRecorder`. The dispatcher has
three appended alert kinds (LowBattery, SystemFault, PhoneDisconnected); the M6 priorities are unchanged. See
[MILESTONE8_SOFTWARE_RELIABILITY_DESIGN.md](../../Docs/MILESTONE8_SOFTWARE_RELIABILITY_DESIGN.md).
