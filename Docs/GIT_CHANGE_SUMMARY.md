# Git Change Summary

**No commits, pushes, tags, branches, rebases, resets, cleans, stashes or identity changes were made
in either repository.** All changes are uncommitted in the working tree for you to review.

## Parent repository (ParaGuardian)

| Item | State |
|---|---|
| Branch | `main` (no commits yet — "No commits yet") |
| Remote | none configured (unchanged) |
| Pre-existing staged entries | `.gitmodules`, `SoarOS` (gitlink) — **unchanged** |
| Pre-existing untracked | `.cproject .mxproject .osx.project .project .settings/ Debug/ Drivers/ Inc/ Middlewares/ ParaGuardian.ioc STM32H743ZITX_FLASH.ld STM32H743ZITX_RAM.ld Src/ Startup/` |
| Git identity | unchanged (`user.name = alymasani`) |

## SoarOS submodule

| Item | State |
|---|---|
| Original commit | `426c1efe895bb58ba2a57ea128c9aad642e9be9f` |
| Original branch | `master`, tracking `origin/master`, clean, no stashes |
| Final state | **identical** — no file modified, `git -C SoarOS status` clean |
| `paraguardian/cmsis-v2` branch | **not created** (compatibility achieved inside ParaGuardian) |

## Files changed in the parent repository

### Modified (pre-existing files)
| File | Change |
|---|---|
| `Src/main.c` | 3 USER CODE edits: include `RunInterface.h`; call `paraguardian_create_tasks()` in RTOS_THREADS; `StartDefaultTask` body → `paraguardian_default_task()` |
| `.cproject` | G++ compiler options (defines, include paths, `-include` compat header, GNU++17) for Debug and Release; G++ linker script; source entries `Components` and `SoarOS` (with exclusions) |
| `.project` | added `org.eclipse.cdt.core.ccnature` |

Originals of these three files were not otherwise altered (the `.ioc`, `FreeRTOSConfig.h`,
`freertos.c`, HAL config and all other generated files are untouched).

### Moved (not deleted)
| From | To | Why |
|---|---|---|
| `Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex 2.c` | `.finder_duplicates_backup/...` | byte-identical Finder copy; caused 39 duplicate-symbol link errors |
| `Drivers/STM32H7xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy 2.h` | `.finder_duplicates_backup/...` | identical duplicate |
| `Drivers/CMSIS/Device/ST/STM32H7xx/Include/{stm32h7xx,system_stm32h7xx,stm32h743xx} 2.h` | `.finder_duplicates_backup/...` | identical duplicates |
You may delete `.finder_duplicates_backup/` once satisfied.

### Added
```
Components/SystemDefines.hpp
Components/SysCore/{SoarOSCmsisV2Compat.h, RunInterface.h, RunInterface.cpp}
Components/Common/{Status.hpp, Status.cpp, ByteOrder.hpp, BusInterfaces.hpp, RegisterBus.hpp, RegisterBus.cpp, ByteRingBuffer.hpp}
Components/Sensors/Common/{SensorTypes.hpp, SensorHealth.hpp}
Components/Sensors/LSM6DSRX/{Lsm6dsrx.hpp, Lsm6dsrx.cpp, Lsm6dsrxRegisters.hpp}
Components/Sensors/BMP585/{Bmp585.hpp, Bmp585.cpp, Bmp585Registers.hpp}
Components/Sensors/MMC5983MA/{Mmc5983ma.hpp, Mmc5983ma.cpp}
Components/Sensors/NEOM9N/{UbxParser.hpp, UbxParser.cpp, NeoM9n.hpp, NeoM9n.cpp}
Components/Sensors/TF02Pro/{Tf02Pro.hpp, Tf02Pro.cpp}
Components/Storage/W25Q128/{W25q128.hpp, W25q128.cpp}
Components/Communications/Bluetooth/{EzSerialLink.hpp, EzSerialLink.cpp, SosMessage.hpp, SosMessage.cpp}
Components/Output/Audio/IAudioAlert.hpp
Components/Output/LED/IStatusLed.hpp
Components/Power/IPowerMonitor.hpp
Components/Acquisition/{SensorMessage.hpp, SampleQueue.hpp, SensorDataHub.hpp, SensorDataHub.cpp, AcquisitionSources.hpp, AcquisitionSources.cpp}
Components/Tasks/{TaskConfig.hpp, RtosSampleQueue.hpp, SystemHealth.hpp, SensorAcquisitionTask.hpp, SensorAcquisitionTask.cpp, HealthMonitorTask.hpp, HealthMonitorTask.cpp}
Components/BSP/{RtosClock.hpp, RtosClock.cpp, BoardConfig.hpp, BoardConfig.cpp, HalSpiDevice.hpp, HalSpiDevice.cpp, HalI2cBus.hpp, HalI2cBus.cpp, HalUartPort.hpp, HalUartPort.cpp}
Components/Algorithms/README.md
Tests/{TestFramework.hpp, run_host_tests.sh}
Tests/mocks/{MockClock.hpp, FakeRegisterDevice.hpp, FakeW25q.hpp, FakeUart.hpp}
Tests/unit/{test_main, test_common, test_lsm6dsrx, test_bmp585, test_mmc5983ma, test_gnss, test_tf02pro, test_w25q128, test_bluetooth, test_acquisition}.cpp
Docs/{PARAGUARDIAN_FIRMWARE_WALKTHROUGH.txt, CMSIS_V2_SOAROS_MIGRATION.md, SENSOR_DRIVER_TEST_REPORT.md, HARDWARE_INTEGRATION_TODO.md, GIT_CHANGE_SUMMARY.md}
```

The local `Debug/` folder was **not** rebuilt (all verification builds ran on a copy in a temporary
directory), so it still contains the old, failing build output. Rebuild in CubeIDE.

## Uncommitted changes remaining
Yes — everything above. Nothing was staged by this session.

## `.gitignore` (created on request; `.DS_Store` added because one appeared in the tree)
```
Debug/
Release/
.finder_duplicates_backup/
*.o
*.d
*.su
*.cyclo
```

## Proposed logical commit boundaries (you run these; messages are suggestions)

1. **CubeMX baseline** — `.ioc`, generated `Src/ Inc/ Drivers/ Middlewares/ Startup/`, linker scripts, `.mxproject`, `.settings/`, original `.project`/`.cproject`, `.gitmodules` + SoarOS gitlink.
   *Note:* `Src/main.c`, `.project` and `.cproject` already contain this session's edits; if you want a
   pure baseline commit, stage them with `git add -p` or commit them in step 2.
   `chore: initial STM32CubeMX project (H743, FreeRTOS CMSIS-V2) with SoarOS submodule`
2. **Build fix + C++ enablement** — move of Finder duplicates, `.project`, `.cproject`, `Components/SystemDefines.hpp`, `Components/SysCore/*`, `Src/main.c`.
   `build: enable C++17 and integrate SoarOS with CMSIS-RTOS V2 via compatibility header`
3. **Common infrastructure** — `Components/Common/*`, `Components/Sensors/Common/*`.
   `feat: bus interfaces, register bus, status codes, ring buffer`
4. **Sensor drivers** — LSM6DSRX, BMP585, MMC5983MA, NEOM9N, TF02Pro (one commit each if you prefer).
5. **Storage + comms** — W25Q128, Bluetooth framing, SOS formatter; Output/Power interfaces.
6. **Acquisition + tasks + BSP** — `Components/Acquisition`, `Components/Tasks`, `Components/BSP`, `Components/Algorithms/README.md`.
7. **Host tests** — `Tests/`.
8. **Docs** — `Docs/`.

Per your university's AI-use policy, consider noting in the commit messages or README that this code
was produced with AI assistance (Claude Code) and reviewed by you.

## Private-fork procedure (only if SoarOS ever needs changes)

1. Fork UCSOAR/SoarOperatingSystem to your account (private).
2. `git -C SoarOS remote add fork <your-fork-url>`; create a branch, commit the changes yourself, push to the fork.
3. In ParaGuardian: `git submodule set-url SoarOS <your-fork-url>` → `git -C SoarOS checkout <commit>` →
   `git add .gitmodules SoarOS` → commit. A submodule stores a commit SHA, so uncommitted changes
   inside `SoarOS/` would never reach anyone else's clone.
