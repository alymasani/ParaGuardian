# Git Change Summary: Flight State, LiDAR Clearance and Landing Safety (Milestone 5)

**No commits, pushes, pull requests, branch changes, tags, stashes, resets, cleans, rebases or identity
changes were made in either repository. Nothing was staged.** All Milestone 5 changes are uncommitted
working-tree changes for your review, on top of the (also uncommitted) Milestone 3 and 4 work.

## Initial state (recorded before any edit)

| Command | Result |
|---|---|
| `git branch --show-current` | `Pre-processing` |
| `git rev-parse --short HEAD` | `c26ff8c` (`main` at `8602405`) |
| `git status --short` | 33 entries, all from Milestones 3–4: 12 modified tracked files, 21 untracked entries |
| `git diff --cached` | empty (nothing staged) |
| `git submodule status` | ` 426c1efe895bb58ba2a57ea128c9aad642e9be9f SoarOS (heads/master)` |
| `git -C SoarOS status` | `## master...origin/master`, clean |
| Stash | empty |
| Host tests | 338 / 338 |
| CubeIDE Debug / Release | 0 / 0 errors / warnings each; text 102 308 / 59 336, bss 48 032 / 48 024 |

## Final state

| Repo | State |
|---|---|
| ParaGuardian | same branch `Pre-processing`, same HEAD `c26ff8c`, nothing staged, no stash |
| SoarOS | **unchanged**: `426c1ef`, `master`, clean |

`git status --short` now shows 13 modified tracked files: the 12 from before plus
`PreprocessingPipeline.hpp`. Untracked entries are the earlier ones plus the Milestone 5 files below.

## Files created in Milestone 5 (25)

```
Components/Algorithms/Flight/FlightStateTypes.hpp               FlightPhase, evidence bits, config, status
Components/Algorithms/Flight/FlightStateDetector.hpp            phase state machine (interface)
Components/Algorithms/Flight/FlightStateDetector.cpp            phase state machine
Components/Algorithms/Landing/GroundClearanceTypes.hpp          ClearanceStatus, flags, config, output
Components/Algorithms/Landing/GroundClearanceEstimator.hpp      LiDAR clearance / rate / TTC (interface)
Components/Algorithms/Landing/GroundClearanceEstimator.cpp      LiDAR clearance / rate / TTC
Components/Algorithms/Landing/LandingSafetyTypes.hpp            warnings, policies, config, SafetySnapshot
Components/Algorithms/Landing/LandingSafetyAnalyzer.hpp         warnings (interface)
Components/Algorithms/Landing/LandingSafetyAnalyzer.cpp         warnings, persistence, hysteresis, cooldown
Components/Algorithms/Safety/SafetyInput.hpp                    SafetyInput + safetyInputFrom()
Components/Algorithms/Safety/SafetyEvent.hpp                    SafetyEvent, enums, ISafetyEventSink
Components/Algorithms/Safety/SafetyEventEmitter.hpp             event / episode numbering, sinks
Components/Algorithms/Safety/SafetyEventQueue.hpp               bounded priority-aware queue
Components/Algorithms/Safety/SafetySnapshotMailbox.hpp          latest-value snapshot hand-over
Components/Algorithms/Safety/SafetyMonitor.hpp                  orchestration (interface), SafetyConfig
Components/Algorithms/Safety/SafetyMonitor.cpp                  orchestration
Tests/sim/FlightSafetySimulator.hpp                             independent flight / terrain / LiDAR simulator
Tests/unit/test_ground_clearance.cpp                            13 tests
Tests/unit/test_flight_state.cpp                                14 tests
Tests/unit/test_landing_safety.cpp                              16 tests
Tests/unit/test_flight_safety_integration.cpp                   12 tests
Docs/FLIGHT_LANDING_DESIGN.md
Docs/FLIGHT_LANDING_VALIDATION_REPORT.md
Docs/FLIGHT_LANDING_WALKTHROUGH.txt
Docs/FLIGHT_LANDING_GIT_SUMMARY.md                              this file
```

## Files modified in Milestone 5 (13, all ParaGuardian-owned)

| File | Milestone 5 change | Why |
|---|---|---|
| `Components/Processing/ProcessedSensorFrame.hpp` | `LidarInfo::lastRejectReasons`, `lastRejectTimestampMs` (additive) | tell "beyond range / weak" from "LiDAR silent" |
| `Components/Processing/PreprocessingPipeline.hpp` | `LidarState` gains the two fields | same |
| `Components/Processing/PreprocessingPipeline.cpp` | records the reasons of a **rejected** LiDAR sample in `ingest()`; copies them into the frame | same; accepted-sample handling unchanged |
| `Components/Tasks/SensorProcessingTask.hpp` | includes; `safety_`, `safetyLock_`, `safetyOut_`, `safetyEventLock_`, `safetyEvents_`; `safety()` / `safetyEvents()` accessors; data-flow comment | RTOS integration |
| `Components/Tasks/SensorProcessingTask.cpp` | constructs them, registers the queue sink, `SOAR_ASSERT(safety_.configValid())`, one line in `Run()` | RTOS integration |
| `Components/Tasks/TaskConfig.hpp` | `SAFETY_EVENT_QUEUE_DEPTH = 16` | queue size |
| `Components/Tasks/SystemHealth.hpp` | 24 safety fields | health diagnostics |
| `Components/Tasks/HealthMonitorTask.hpp/.cpp` | `safetyCopy_`; copies the snapshot and queue statistics into `g_systemHealth` | health diagnostics |
| `Tests/run_host_tests.sh` | compiles the 4 new `.cpp` files | host tests |
| `Components/Algorithms/README.md` | Flight / Landing / Safety section; out-of-scope list | documentation |
| `Docs/RTOS_TIMING_ANALYSIS.md` | SensorProc row; new §4c | timing budget |
| `Docs/HARDWARE_INTEGRATION_TODO.md` | new §15 | hardware decisions |

**Not modified:**
* project and build files: `.ioc`, `.cproject`, `.project`, `.mxproject`, `Src/*`, `Inc/*`, linker
  scripts;
* vendor code: HAL / CMSIS / FreeRTOS;
* every sensor driver, including TF02-Pro and NEO-M9N;
* acquisition, validation, calibration and voting logic;
* the Madgwick attitude code and the UKF / navigation code;
* task priorities, stack sizes, sampling rates, clocks, pins;
* the SoarOS submodule.

The new folders are compiled by the existing `Components` source entry, with no build-setting change.
No CMake or PlatformIO.

## Verification after the changes

| Check | Result |
|---|---|
| Host tests | 393 / 393 (338 existing unchanged + 55 new) |
| ASan + UBSan / TSan | 393 / 393 each, 0 reports |
| CubeIDE Debug | 0 / 0; text 122 804, data 104, bss 51 120 |
| CubeIDE Release | 0 / 0; text 70 920, data 104, bss 51 112 |
| SoarOS | unchanged |

## Proposed commit boundaries (for your manual review; run them yourself)

Commit Milestones 3 and 4 first (see `MADGWICK_GIT_CHANGE_SUMMARY.md`, `VERTICAL_UKF_GIT_SUMMARY.md`),
then:

1. `feat(preprocessing): pass LiDAR rejection reasons through the frame`: `ProcessedSensorFrame.hpp`,
   `PreprocessingPipeline.hpp/.cpp` (LiDAR hunks only).
2. `feat(safety): SafetyEvent, emitter, bounded queue, snapshot mailbox, SafetyInput`:
   `Components/Algorithms/Safety/SafetyEvent*.hpp`, `SafetySnapshotMailbox.hpp`, `SafetyInput.hpp`.
3. `feat(landing): LiDAR ground clearance, closure rate and time to contact`:
   `Components/Algorithms/Landing/GroundClearance*`.
4. `feat(flight): evidence-based flight-state detector`: `Components/Algorithms/Flight/*`.
5. `feat(landing): landing / descent warnings with persistence, hysteresis and cooldown`:
   `Components/Algorithms/Landing/LandingSafety*`, `Components/Algorithms/Safety/SafetyMonitor.*`.
6. `feat(rtos): run the safety layer in SensorProcessingTask, publish snapshot / events, health`:
   `SensorProcessingTask.*`, `TaskConfig.hpp`, `SystemHealth.hpp`, `HealthMonitorTask.*`.
7. `test(safety): independent flight / terrain / LiDAR simulator and 55 tests`:
   `Tests/sim/FlightSafetySimulator.hpp`, `Tests/unit/test_{ground_clearance,flight_state,landing_safety,flight_safety_integration}.cpp`,
   `Tests/run_host_tests.sh`.
8. `docs(safety): design, validation report, walkthrough, git summary`: `Docs/FLIGHT_LANDING_*`,
   `Docs/RTOS_TIMING_ANALYSIS.md`, `Docs/HARDWARE_INTEGRATION_TODO.md`, `Components/Algorithms/README.md`.

Files that carry hunks from several milestones (e.g. `SensorProcessingTask.cpp`, `SystemHealth.hpp`)
can be split with `git add -p`.

## AI-use attribution

This work was produced with AI assistance (Claude Code) under the student's direction. Per your
university's policy, state this in the commit messages and project documentation. No `Co-authored-by`
trailer or other attribution was added to any file or commit (no commits were made).
