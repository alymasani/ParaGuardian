# Git Change Summary: Sensor Preprocessing (Milestone 2)

**No commits, pushes, branches, tags, stashes, resets, cleans, rebases or identity changes were made in either repository. This work staged nothing; the two entries that were already staged (`.gitmodules`, `SoarOS`) are unchanged.**

## Initial state (recorded before any edit)

| Command | Result |
|---|---|
| `git status` | branch `main`, **no commits yet**; staged: `.gitmodules`, `SoarOS` (gitlink); everything else untracked (milestone 1 and hardening work) |
| `git branch --show-current` | `main` |
| `git submodule status` | ` 426c1efe895bb58ba2a57ea128c9aad642e9be9f SoarOS (heads/master)` |
| `git -C SoarOS status` | `On branch master … up to date with 'origin/master' … nothing to commit, working tree clean` |
| `git config user.name` | `alymasani` (unchanged) |

## Final state

| Repo | State |
|---|---|
| ParaGuardian | same branch, still no commits, same two staged entries, no stash; all changes below are **untracked working-tree changes** |
| SoarOS | **unchanged**: `426c1efe…`, clean, `master`. No `paraguardian/*` branch was needed |

## Files created

```
Components/Processing/VectorMath.hpp
Components/Processing/ProcessingTypes.hpp
Components/Processing/ProcessingConfig.hpp
Components/Processing/TimeBase.hpp
Components/Processing/RunningStats.hpp
Components/Processing/SampleValidator.hpp / .cpp
Components/Processing/StationaryDetector.hpp / .cpp
Components/Processing/Calibration.hpp / .cpp
Components/Processing/CalibrationController.hpp / .cpp
Components/Processing/AxisAlignment.hpp / .cpp
Components/Processing/RedundancyVoter.hpp / .cpp
Components/Processing/ProcessedSensorFrame.hpp
Components/Processing/ProcessedFrameMailbox.hpp
Components/Processing/PreprocessingPipeline.hpp / .cpp
Components/Tasks/SensorProcessingTask.hpp / .cpp
Tests/sim/SensorSimulator.hpp
Tests/unit/test_preprocessing_validation.cpp
Tests/unit/test_preprocessing_calibration.cpp
Tests/unit/test_preprocessing_alignment.cpp
Tests/unit/test_preprocessing_redundancy.cpp
Tests/unit/test_preprocessing_pipeline.cpp
Docs/SENSOR_PREPROCESSING_WALKTHROUGH.txt
Docs/SENSOR_PREPROCESSING_DESIGN.md
Docs/SENSOR_PREPROCESSING_TEST_REPORT.md
Docs/SENSOR_PREPROCESSING_GIT_SUMMARY.md
```

## Files modified (all ParaGuardian-owned)

| File | Change |
|---|---|
| `Components/Acquisition/SampleFreshness.hpp` | `SequenceTracker::observeHeader()` + `SequenceKind`/`SequenceObservation`; `duplicates()` counter. `observe()` behaviour unchanged |
| `Components/Tasks/TaskConfig.hpp` | `PROCESSING_PRIORITY` (32), `PROCESSING_STACK_WORDS` (1024), `PROCESSING_QUEUE_DEPTH` (64), `PROCESSING_IDLE_TIMEOUT_MS` (50) + static_asserts on priority ordering |
| `Components/Tasks/SystemHealth.hpp` | preprocessing fields (task counters, newest-frame summary, calibration states) |
| `Components/Tasks/HealthMonitorTask.hpp / .cpp` | collects the preprocessing fields; frame copy kept in a member (not on the 2 KiB stack) |
| `Components/SysCore/RunInterface.cpp` | creates `SensorProcessingTask` (subscribes to the acquisition hub) and adds its stack to the health watch list |
| `Tests/run_host_tests.sh` | compiles the 7 new `Components/Processing/*.cpp` |
| `Components/Algorithms/README.md` | points future algorithms at `ProcessedSensorFrame` |
| `Docs/HARDWARE_INTEGRATION_TODO.md` | preprocessing items that need hardware |
| `Docs/RTOS_TIMING_ANALYSIS.md` | task table: `SensorProc` row |

**Not modified:**
- `.ioc`, `.cproject`, `.project`, `Src/*`, `Inc/*`, linker scripts and the CubeMX USER CODE regions;
- HAL/CMSIS/FreeRTOS sources, the SoarOS submodule;
- every sensor driver, `SensorAcquisitionTask`, `AcquisitionScheduler`, `SensorDataHub`, `SensorTypes.hpp` and `SensorMessage.hpp`.

The new `Components/Processing/` folder is compiled by the existing `Components` source entry, so no build-setting change was needed.

## Proposed commit boundaries (for your review; run them yourself)

1. `feat(processing): types, config, time base, streaming statistics`: `VectorMath.hpp`, `ProcessingTypes.hpp`, `ProcessingConfig.hpp`, `TimeBase.hpp`, `RunningStats.hpp`, `SampleFreshness.hpp` (`SequenceTracker` extension).
2. `feat(processing): sample validation`: `SampleValidator.*`.
3. `feat(processing): stationary detection and calibration`: `StationaryDetector.*`, `Calibration.*`, `CalibrationController.*`.
4. `feat(processing): sensor-to-body axis alignment`: `AxisAlignment.*`.
5. `feat(processing): redundancy voting and processed frame`: `RedundancyVoter.*`, `ProcessedSensorFrame.hpp`, `ProcessedFrameMailbox.hpp`, `PreprocessingPipeline.*`.
6. `feat(rtos): SensorProcessingTask and health reporting`: `SensorProcessingTask.*`, `TaskConfig.hpp`, `SystemHealth.hpp`, `HealthMonitorTask.*`, `RunInterface.cpp`.
7. `test(processing): simulator and 85 preprocessing tests`: `Tests/`.
8. `docs(processing): design, walkthrough, test report, git summary`: `Docs/`, `Components/Algorithms/README.md`.

Per your university's policy, note in the commit messages that this work was produced with AI assistance (Claude Code). No `Co-authored-by` trailer was added anywhere.
