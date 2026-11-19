# Git Change Summary: Crash Detection, Emergency Manager and Alert Dispatch (Milestone 6)

**No commits, pushes, pull requests, branch changes, tags, stashes, resets, cleans, rebases or
identity changes were made in either repository. Nothing was staged.** All Milestone 6 changes are
uncommitted working-tree changes for your review. They sit on top of the (also uncommitted)
Milestone 3–5 work, which was preserved.

## Initial state (recorded before any edit)

| Command | Result |
|---|---|
| `git branch --show-current` | `Pre-processing` |
| `git rev-parse --short HEAD` | `c26ff8c` |
| `git diff --cached` | empty (nothing staged) |
| `git status --short` | 46 entries, all from Milestones 3–5: 13 modified tracked files, 33 untracked entries |
| `git submodule status` | ` 426c1efe895bb58ba2a57ea128c9aad642e9be9f SoarOS (heads/master)` |
| `git -C SoarOS status` | `## master...origin/master`, clean |
| Stash | empty |
| Host tests | 393 / 393 |
| CubeIDE builds | Not re-run at the start. The reference is the Milestone 5 final build of the same tree: Debug text 122 804 / bss 51 120, Release text 70 920 / bss 51 112, 0 errors / 0 warnings. See the validation report §10. |

## Final state

| Repo | State |
|---|---|
| ParaGuardian | Same branch `Pre-processing`, same HEAD `c26ff8c`, nothing staged, no stash; 67 status entries (16 modified tracked files, 51 untracked entries) |
| SoarOS | **Unchanged**: `426c1ef`, `master`, clean |

Not modified:
- `.ioc`, `.cproject`, `.project`, `.mxproject`, `Src/*`, `Inc/*`, linker scripts;
- HAL, CMSIS and FreeRTOS;
- every sensor driver, acquisition, calibration and voting;
- Madgwick, the UKF, flight-state, ground-clearance and landing-safety algorithms;
- clocks and pins;
- the SoarOS submodule.

`git diff -- Components/Sensors Components/Acquisition Components/BSP Src Inc Drivers Middlewares` is
empty. The new folders are compiled by the existing `Components` source entry, with no build-setting
change. No CMake or PlatformIO.

## Files created in Milestone 6 (36)

```
Components/Algorithms/Crash/CrashConfig.hpp             thresholds (provisional) + validity
Components/Algorithms/Crash/CrashTypes.hpp              states, evidence, CrashInput / CrashStatus / IncidentContext / CrashReport
Components/Algorithms/Crash/CrashDetector.hpp           detector (interface)
Components/Algorithms/Crash/CrashDetector.cpp           detector
Components/Algorithms/Crash/CrashInputAdapter.hpp       crashInputFrom(), updateIncidentContext(), CrashReportMailbox
Components/Algorithms/Emergency/EmergencyConfig.hpp     countdowns, retry policy (placeholders)
Components/Algorithms/Emergency/EmergencyTypes.hpp      states, delivery stages, EmergencyStatus
Components/Algorithms/Emergency/EmergencyCommand.hpp    commands (integrity), transport reports, BoundedInbox
Components/Algorithms/Emergency/EmergencyManager.hpp    incident lifecycle (interface)
Components/Algorithms/Emergency/EmergencyManager.cpp    incident lifecycle
Components/Algorithms/Emergency/SosRequest.hpp          versioned SOS request, CRC-32
Components/Algorithms/Emergency/SosRequest.cpp          encode / decode / legacy mapping
Components/Algorithms/Emergency/SosRequestMailbox.hpp   reliable pending-request record
Components/Algorithms/Emergency/ISosTransport.hpp       transport interface + UnconfiguredSosTransport
Components/Algorithms/Emergency/IEmergencyStore.hpp     persistence record + interface + NullEmergencyStore
Components/Algorithms/Safety/AlertTypes.hpp             alerts, patterns, DispatchStatus
Components/Algorithms/Safety/AlertPriorityPolicy.hpp    the priority / pattern table
Components/Algorithms/Safety/SafetyEventDispatcher.hpp  event consumer + alert arbitration (interface)
Components/Algorithms/Safety/SafetyEventDispatcher.cpp  event consumer + alert arbitration
Components/Algorithms/Safety/ValueMailbox.hpp           generic latest-value mailbox
Components/Output/Audio/AudioAlertScheduler.hpp         audio pattern scheduler + UnconfiguredAudioAlert
Components/Output/Audio/AudioAlertScheduler.cpp
Components/Output/LED/LedPatternScheduler.hpp           LED pattern scheduler + UnconfiguredStatusLed
Components/Output/LED/LedPatternScheduler.cpp
Components/Tasks/EmergencyAlertTask.hpp                 static task (priority 36, 20 ms / notification)
Components/Tasks/EmergencyAlertTask.cpp
Tests/mocks/MockEmergencyIo.hpp                         transport / store / audio / LED test doubles
Tests/sim/CrashSimulator.hpp                            incident simulator on top of fsim
Tests/unit/test_crash_detector.cpp                      19 tests
Tests/unit/test_emergency_manager.cpp                   25 tests
Tests/unit/test_alert_dispatch.cpp                      13 tests
Tests/unit/test_crash_emergency_integration.cpp         13 tests
Docs/CRASH_EMERGENCY_DESIGN.md
Docs/CRASH_EMERGENCY_VALIDATION_REPORT.md
Docs/CRASH_EMERGENCY_WALKTHROUGH.txt
Docs/CRASH_EMERGENCY_GIT_SUMMARY.md                     this file
```

## Files modified in Milestone 6 (20)

Three of these files were not modified before Milestone 6: `IAudioAlert.hpp`, `IStatusLed.hpp` and
`RunInterface.cpp`. The `Safety/` headers are untracked Milestone 5 files.

| File | Milestone 6 change | Why |
|---|---|---|
| `Components/Processing/ProcessedSensorFrame.hpp` | `ImuChannelInfo` + 6 impact-peak fields (additive) | IMU evidence before smoothing |
| `Components/Processing/PreprocessingPipeline.hpp` | `ImuState` + the same fields | same |
| `Components/Processing/PreprocessingPipeline.cpp` | `handleImu` keeps the max since the frame + saturation flags; `fillImu` copies; `buildFrame` resets | same; accepted-sample handling, voting and calibration unchanged |
| `Components/Algorithms/Safety/SafetyEvent.hpp` | new event types 34–41 (32 / 33 now produced), source comments, per-emitter id note | reuse the Milestone 5 event system |
| `Components/Algorithms/Safety/SafetyMonitor.hpp` | non-const `eventEmitter()` | crash events share the processing emitter |
| `Components/Algorithms/Safety/SafetyEventQueue.hpp` | comment only (the consumer now exists) | documentation |
| `Components/Output/Audio/IAudioAlert.hpp` | `AlertTone` values appended (existing values unchanged) | alert tones |
| `Components/Output/LED/IStatusLed.hpp` | `virtual bool available() const { return true; }` | report "no LED hardware" |
| `Components/Tasks/SensorProcessingTask.hpp/.cpp` | CrashDetector, IncidentContext, CrashReport mailbox, `crashReports()` accessor, `SOAR_ASSERT(crash_.configValid())`, crash update after the safety update | RTOS integration |
| `Components/Tasks/TaskConfig.hpp` | EmergencyAlert priority / stack / period / depths + static_asserts | task budget |
| `Components/Tasks/SystemHealth.hpp` | 45 crash / emergency / alert / timing fields | diagnostics |
| `Components/Tasks/HealthMonitorTask.hpp/.cpp` | copies (members) + collection of the new fields | diagnostics |
| `Components/SysCore/RunInterface.cpp` | creates EmergencyAlertTask; Health watches its stack | task creation |
| `Tests/run_host_tests.sh` | 6 new production `.cpp` in the host build | host tests |
| `Components/Algorithms/README.md` | Milestone 6 section; out-of-scope list | documentation |
| `Docs/RTOS_TIMING_ANALYSIS.md` | EmergencyAlert row; SensorProc row; new §4d | timing budget |
| `Docs/HARDWARE_INTEGRATION_TODO.md` | new §16 | hardware decisions |

## Verification after the changes

| Check | Result |
|---|---|
| Host tests | 463 / 463 (393 existing unchanged + 70 new) |
| ASan + UBSan | 463 / 463, 0 reports |
| ThreadSanitizer | 463 / 463, 0 reports |
| CubeIDE Debug | 0 errors / 0 warnings; text 156 636, data 104, bss 59 288 |
| CubeIDE Release | 0 errors / 0 warnings; text 89 080, data 104, bss 59 288 |
| Strict ARM warnings (new and changed `.cpp`) | 0 |
| SoarOS | unchanged |

## Proposed commit boundaries (for your manual review; run them yourself)

Commit Milestones 3–5 first (their git summaries list them). Then:

1. `feat(preprocessing): per-IMU impact peaks and saturation since the last frame`:
   `ProcessedSensorFrame.hpp`, `PreprocessingPipeline.hpp/.cpp` (IMU-peak hunks only).
2. `feat(crash): multi-evidence possible-impact / suspected-crash detector`:
   `Components/Algorithms/Crash/*`, `SafetyEvent.hpp`, `SafetyMonitor.hpp`.
3. `feat(emergency): incident lifecycle, countdown, manual SOS, versioned SOS request, retries`:
   `Components/Algorithms/Emergency/*`.
4. `feat(alerts): safety-event dispatcher, priority policy, audio / LED schedulers`:
   `Safety/AlertTypes.hpp`, `AlertPriorityPolicy.hpp`, `SafetyEventDispatcher.*`, `ValueMailbox.hpp`,
   `SafetyEventQueue.hpp`, `Components/Output/**`.
5. `feat(rtos): EmergencyAlertTask, crash detection in SensorProc, health fields`:
   `Components/Tasks/*`, `Components/SysCore/RunInterface.cpp`.
6. `test(crash): incident simulator, mocks and 70 tests`: `Tests/**`.
7. `docs(crash): design, validation, walkthrough, git summary; timing / hardware TODO / README`.

Files that carry hunks from several milestones (e.g. `SensorProcessingTask.cpp`, `SystemHealth.hpp`,
`HealthMonitorTask.cpp`, `run_host_tests.sh`) can be split with `git add -p`.

## AI-use disclosure

This work was produced with AI assistance (Claude Code) under the student's direction. State this in
the commit messages and project documentation as your university's policy requires. No
`Co-authored-by` trailer or other attribution was added to any file or commit (no commits were made).
