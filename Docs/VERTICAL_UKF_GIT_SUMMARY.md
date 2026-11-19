# Git Change Summary: Vertical UKF (Milestone 4)

**No commits, pushes, pull requests, branch changes, tags, stashes, resets, cleans, rebases or
identity changes were made in either repository. Nothing was staged.** All Milestone 4 changes are
uncommitted working-tree changes for your review, on top of the (also uncommitted) Milestone 3 work.

## Initial state (recorded before any edit)

| Command | Result |
|---|---|
| `git branch --show-current` | `Pre-processing` |
| `git rev-parse --short HEAD` | `c26ff8c` (`main` at `8602405`) |
| `git status` | Milestone 3 work uncommitted: 11 modified tracked files, 10 untracked entries (`Components/Algorithms/Attitude/`, 4 `Docs/MADGWICK_*`, `Tests/sim/AttitudeSimulator.hpp`, 4 `Tests/unit/test_attitude_*.cpp`) |
| `git submodule status` | ` 426c1efe895bb58ba2a57ea128c9aad642e9be9f SoarOS (heads/master)` |
| `git -C SoarOS status` | `## master...origin/master`, clean |
| Stash | empty |
| `git config user.name` | `alymasani` (unchanged) |
| Host tests | 278 / 278 passed |
| CubeIDE Debug / Release | 0 errors / 0 warnings each; text 80 916 / 49 484, bss 46 024 / 46 016 |

## Final state

| Repo | State |
|---|---|
| ParaGuardian | same branch `Pre-processing`, same HEAD `c26ff8c`, nothing staged, no stash |
| SoarOS | **unchanged**: `426c1ef`, `master`, clean |

`git status --short` now shows 12 modified tracked files: the 11 Milestone 3 files plus
`PreprocessingPipeline.cpp`. Milestone 4 modified 11 files: 10 of the Milestone 3 files (all except
`TaskConfig.hpp`, which it did not touch) and `PreprocessingPipeline.cpp`. The untracked entries are the Milestone 3 ones plus the Milestone 4
files below.

## Files created in Milestone 4 (24)

```
Components/Algorithms/Navigation/NavMath.hpp                     fixed-size matrix maths, Cholesky, SPD inverse
Components/Algorithms/Navigation/UnscentedKalmanFilter.hpp       generic UKF engine (template on N)
Components/Algorithms/Navigation/IVerticalProcessModel.hpp       process-model replacement boundary
Components/Algorithms/Navigation/ConstantAccelerationModel.hpp   placeholder model (interface)
Components/Algorithms/Navigation/ConstantAccelerationModel.cpp   placeholder model (white jerk + unmodelled acceleration)
Components/Algorithms/Navigation/NavigationTypes.hpp             NavigationInput, NavigationEstimate, enums, flags
Components/Algorithms/Navigation/NavigationConfig.hpp            parameters + isValid()
Components/Algorithms/Navigation/BaroSampleTracker.hpp           each barometer sample assimilated once
Components/Algorithms/Navigation/VerticalNavigationEstimator.hpp navigation integration (interface)
Components/Algorithms/Navigation/VerticalNavigationEstimator.cpp timing, selection, init, gating, recovery, output
Components/Algorithms/Navigation/NavigationInputAdapter.hpp      frame + attitude -> NavigationInput
Components/Algorithms/Navigation/NavigationMailbox.hpp           latest-value hand-over
Components/Algorithms/Navigation/BaroAlphaBetaEstimator.hpp      barometer-only baseline (interface)
Components/Algorithms/Navigation/BaroAlphaBetaEstimator.cpp      barometer-only baseline
Tests/sim/LinearKalmanReference.hpp                              independent double-precision linear KF
Tests/sim/VerticalSimulator.hpp                                  independent truth + sensor simulation + metrics
Tests/unit/test_navigation_math.cpp                              12 tests
Tests/unit/test_navigation_estimator.cpp                         17 tests
Tests/unit/test_navigation_scenarios.cpp                         27 tests (scenarios 1-30, consistency, sweeps)
Tests/unit/test_navigation_integration.cpp                       4 tests
Docs/VERTICAL_UKF_DESIGN.md
Docs/VERTICAL_UKF_VALIDATION_REPORT.md
Docs/VERTICAL_UKF_WALKTHROUGH.txt
Docs/VERTICAL_UKF_GIT_SUMMARY.md                                 this file
```

## Files modified in Milestone 4 (11, all ParaGuardian-owned)

| File | Milestone 4 change | Why |
|---|---|---|
| `Components/Tasks/SensorProcessingTask.hpp` | includes; `navModel_`, `navigation_`, `navigationLock_`, `navigationOut_` members; `navigation()` accessor; header text on why navigation runs here | RTOS integration |
| `Components/Tasks/SensorProcessingTask.cpp` | constructs model / estimator / mailbox; `SOAR_ASSERT(navigation_.configValid())`; two lines in `Run()` after the attitude update | RTOS integration |
| `Components/Tasks/SystemHealth.hpp` | 24 navigation fields | health diagnostics |
| `Components/Tasks/HealthMonitorTask.hpp` | `navigationCopy_` member (kept off the 2 KiB stack) | health diagnostics |
| `Components/Tasks/HealthMonitorTask.cpp` | copies the newest `NavigationEstimate` into `g_systemHealth` | health diagnostics |
| `Components/Processing/ProcessedSensorFrame.hpp` | `GnssInfo::speedAccuracy_mps` (additive pass-through field); consumer comment updated | GNSS velocity accuracy for the UKF |
| `Components/Processing/PreprocessingPipeline.cpp` | **one line**: `g.speedAccuracy_mps = s.speedAccuracy_mps;` | the driver already decoded sAcc; preprocessing did not pass it on |
| `Components/Algorithms/README.md` | `Navigation/` section; out-of-scope list updated | documentation |
| `Tests/run_host_tests.sh` | compiles the 3 navigation `.cpp` files | host tests |
| `Docs/RTOS_TIMING_ANALYSIS.md` | SensorProc row; new §4b "Vertical navigation (UKF) inside SensorProc" | timing budget |
| `Docs/HARDWARE_INTEGRATION_TODO.md` | new §14 "Vertical navigation (UKF): decisions that need the hardware" | hardware decisions |

**Not modified:** `.ioc`, `.cproject`, `.project`, `.mxproject`, `Src/*`, `Inc/*` (no CubeMX USER CODE
region), linker scripts, HAL/CMSIS/FreeRTOS, every sensor driver, `SensorAcquisitionTask`,
`AcquisitionScheduler`, `SensorDataHub`, the voter / validator / calibration logic, the Madgwick
attitude code (`Components/Algorithms/Attitude/*`), task priorities, stack sizes, sampling rates,
queue policies, clocks, pins, the SoarOS submodule. The new `Components/Algorithms/Navigation/`
folder is compiled by the existing `Components` source entry, with no build-setting change. CMake and
PlatformIO were not introduced.

## Verification after the changes

| Check | Result |
|---|---|
| Host tests | 338 / 338 (278 existing + 60 new) |
| ASan + UBSan / TSan | 338 / 338 each, 0 reports |
| CubeIDE Debug | 0 errors / 0 warnings; text 102 308, data 104, bss 48 032 |
| CubeIDE Release | 0 errors / 0 warnings; text 59 336, data 104, bss 48 024 |
| SoarOS | unchanged |

(The CubeIDE builds ran headless on a scratch copy of the tree. No `Debug/` or `Release/` output was
written into the repository.)

## Proposed commit boundaries (for your manual review; run them yourself)

Commit Milestone 3 first (see `MADGWICK_GIT_CHANGE_SUMMARY.md`), then:

1. `feat(nav): fixed-size matrix maths and generic UKF engine`: `NavMath.hpp`, `UnscentedKalmanFilter.hpp`.
2. `feat(nav): replaceable process model and constant-acceleration placeholder`:
   `IVerticalProcessModel.hpp`, `ConstantAccelerationModel.*`.
3. `feat(nav): vertical navigation estimator and NavigationEstimate API`: `NavigationTypes.hpp`,
   `NavigationConfig.hpp`, `BaroSampleTracker.hpp`, `VerticalNavigationEstimator.*`,
   `NavigationInputAdapter.hpp`, `NavigationMailbox.hpp`.
4. `feat(nav): barometer-only alpha-beta baseline`: `BaroAlphaBetaEstimator.*`.
5. `feat(preprocessing): pass GNSS speed accuracy through the frame`: `ProcessedSensorFrame.hpp`,
   `PreprocessingPipeline.cpp`.
6. `feat(rtos): run navigation in SensorProcessingTask, publish and report health`:
   `SensorProcessingTask.*`, `SystemHealth.hpp`, `HealthMonitorTask.*`.
7. `test(nav): linear-KF reference, vertical simulator, 60 tests, consistency and sweeps`:
   `Tests/sim/LinearKalmanReference.hpp`, `Tests/sim/VerticalSimulator.hpp`,
   `Tests/unit/test_navigation_*.cpp`, `Tests/run_host_tests.sh`.
8. `docs(nav): design, validation report, walkthrough, git summary`: `Docs/VERTICAL_UKF_*`,
   `Docs/RTOS_TIMING_ANALYSIS.md`, `Docs/HARDWARE_INTEGRATION_TODO.md`, `Components/Algorithms/README.md`.

Where a file carries both Milestone 3 and Milestone 4 changes (e.g. `SensorProcessingTask.cpp`),
`git add -p` separates the hunks.

## AI-use attribution

This work was produced with AI assistance (Claude Code) under the student's direction. Per your
university's policy, state this in the commit messages and in your project documentation. No
`Co-authored-by` trailer or other attribution was added to any file or commit (no commits were made).
