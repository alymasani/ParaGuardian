# Git Change Summary: Madgwick Attitude Estimation (Milestone 3)

**No commits, pushes, pull requests, branches, tags, stashes, resets, cleans, rebases or identity changes
were made in either repository. Nothing was staged. All changes are uncommitted working-tree changes for
your review.**

## Initial state (recorded before any edit)

| Command | Result |
|---|---|
| `git status` | branch `Pre-processing`, up to date with `origin/Pre-processing`, **working tree clean** |
| `git log` | HEAD `c26ff8c` ("added in driver code and pre-processing pipeline…"); `main` at `8602405` (merge of PR #1 from `Pre-processing`) |
| Parent branch | `main` (`origin/HEAD -> origin/main`) |
| `git submodule status` | ` 426c1efe895bb58ba2a57ea128c9aad642e9be9f SoarOS (heads/master)` |
| `git -C SoarOS status` | `## master...origin/master`, clean |
| `git config user.name` | `alymasani` (unchanged) |
| Stash | empty |
| Host tests | 205/205 passed |
| CubeIDE Debug / Release | 0 errors / 0 warnings each (text 61552 / 37744) |

## Final state

| Repo | State |
|---|---|
| ParaGuardian | same branch `Pre-processing`, same HEAD `c26ff8c`, no stash; 11 modified tracked files, 20 new untracked files (below) |
| SoarOS | **unchanged**: `426c1ef`, `master`, clean |

## Files created (20)

```
Components/Algorithms/Attitude/Quaternion.hpp           quaternion maths and the q_ENU_from_body convention
Components/Algorithms/Attitude/MadgwickFilter.hpp       Madgwick step (interface)
Components/Algorithms/Attitude/MadgwickFilter.cpp       Madgwick step, accel/mag gradients
Components/Algorithms/Attitude/AttitudeEstimator.hpp    estimator (interface)
Components/Algorithms/Attitude/AttitudeEstimator.cpp    state machine, timing, gates, recovery, magnetometer policy
Components/Algorithms/Attitude/GravityCompensator.hpp   linear acceleration (interface)
Components/Algorithms/Attitude/GravityCompensator.cpp   a_ENU = R(q) f + (0,0,-g)
Components/Algorithms/Attitude/AttitudeConfig.hpp       parameters + isValid()
Components/Algorithms/Attitude/AttitudeTypes.hpp        enums, flags, AttitudeInput, AttitudeEstimate
Components/Algorithms/Attitude/AttitudeFrameAdapter.hpp ProcessedSensorFrame -> AttitudeInput
Components/Algorithms/Attitude/AttitudeMailbox.hpp      latest-value hand-over
Tests/sim/AttitudeSimulator.hpp                         independent truth + sensor model + error metrics
Tests/unit/test_attitude_math.cpp                       24 tests
Tests/unit/test_attitude_estimator.cpp                  22 tests
Tests/unit/test_attitude_scenarios.cpp                  20 tests (17 scenarios + simulator check + 3 sweeps)
Tests/unit/test_attitude_integration.cpp                7 tests
Docs/MADGWICK_ATTITUDE_DESIGN.md
Docs/MADGWICK_VALIDATION_REPORT.md
Docs/MADGWICK_ATTITUDE_WALKTHROUGH.txt
Docs/MADGWICK_GIT_CHANGE_SUMMARY.md                     this file
```

## Files modified (11, all ParaGuardian-owned)

| File | Change | Why |
|---|---|---|
| `Components/Tasks/SensorProcessingTask.hpp` | includes; `attitude_`, `attitudeLock_`, `attitudeOut_` members; `attitude()` accessor; header explains option A | attitude runs in the existing processing task |
| `Components/Tasks/SensorProcessingTask.cpp` | constructs the estimator and mailbox; asserts a valid `AttitudeConfig`; after `frames_.publish`: `attitude_.update(attitudeInputFromFrame(work_))` + `attitudeOut_.publish(...)` | integration (two lines in the loop) |
| `Components/Tasks/SystemHealth.hpp` | 18 attitude fields (state, heading, correction status, centi-degree angles, mm/s² vertical acceleration, counters) | health diagnostics |
| `Components/Tasks/HealthMonitorTask.hpp` | `attitudeCopy_` member (kept off the 2 KiB stack) | health diagnostics |
| `Components/Tasks/HealthMonitorTask.cpp` | `toScaledInt()` helper (NaN → `INT32_MIN`); copies the newest estimate into `g_systemHealth` | health diagnostics |
| `Components/Tasks/TaskConfig.hpp` | comment only: stack budget now includes the attitude update | documentation |
| `Components/Processing/ProcessedSensorFrame.hpp` | comment only: gravity removal now exists (`AttitudeEstimate::linearAccel`) | stale "future Madgwick" text |
| `Components/Algorithms/README.md` | describes `Attitude/` and how to read the estimate | documentation |
| `Tests/run_host_tests.sh` | compiles the 3 new attitude `.cpp` files | host tests |
| `Docs/RTOS_TIMING_ANALYSIS.md` | SensorProc row; new §4a "Attitude estimation inside SensorProc" | timing budget |
| `Docs/HARDWARE_INTEGRATION_TODO.md` | new §13 "Attitude estimation: decisions that need the hardware" | hardware decisions |

**Not modified**: `.ioc`, `.cproject`, `.project`, `.mxproject`, `Src/*`, `Inc/*` (no CubeMX USER CODE
region touched), linker scripts, HAL/CMSIS/FreeRTOS, every sensor driver, `SensorAcquisitionTask`,
`AcquisitionScheduler`, `SensorDataHub`, `SensorTypes.hpp`, `SensorMessage.hpp`, all preprocessing logic
(`Components/Processing/*.cpp`), the SoarOS submodule. The new `Components/Algorithms/Attitude/` folder is
compiled by the existing `Components` source entry: no build-setting change.

## Proposed commit boundaries (for your manual review; run them yourself)

1. `feat(attitude): quaternion maths and conventions`: `Quaternion.hpp`.
2. `feat(attitude): Madgwick filter core`: `MadgwickFilter.*`.
3. `feat(attitude): estimator state machine, gating and magnetometer policy`: `AttitudeConfig.hpp`,
   `AttitudeTypes.hpp`, `AttitudeEstimator.*`.
4. `feat(attitude): gravity compensation`: `GravityCompensator.*`.
5. `feat(rtos): run attitude in SensorProcessingTask, publish and report health`:
   `AttitudeFrameAdapter.hpp`, `AttitudeMailbox.hpp`, `SensorProcessingTask.*`, `SystemHealth.hpp`,
   `HealthMonitorTask.*`, `TaskConfig.hpp`, `ProcessedSensorFrame.hpp` (comment).
6. `test(attitude): independent simulator, 73 tests, sensitivity sweeps`: `Tests/sim/AttitudeSimulator.hpp`,
   `Tests/unit/test_attitude_*.cpp`, `Tests/run_host_tests.sh`.
7. `docs(attitude): design, validation report, walkthrough, git summary`: `Docs/MADGWICK_*`,
   `Docs/RTOS_TIMING_ANALYSIS.md`, `Docs/HARDWARE_INTEGRATION_TODO.md`, `Components/Algorithms/README.md`.

Per your university's policy, say in the commit messages that this work was produced with AI assistance
(Claude Code). No `Co-authored-by` trailer was added anywhere.
