# Milestone 8 – Git Change Summary

## Initial state
- **Branch:** `main`. The brief expected `Pre-processing`. Immediately before this milestone, at the user's explicit request, `Pre-processing` was merged into `main` (merge commit `baec91b`) and `main` was pushed to `origin` (`8602405..baec91b`). The two branches had identical trees at that point. The milestone work was done on `main`, without changing branches.
- **HEAD:** `baec91b Merge branch 'Pre-processing' into main`.
- **Working tree:** clean at the start (no staged, unstaged or untracked changes). There were no earlier user changes to preserve.
- **SoarOS:** `426c1efe895bb58ba2a57ea128c9aad642e9be9f` (heads/master), clean. **Unchanged** at the end.

## Final state
- Nothing was staged, committed or pushed for Milestone 8. HEAD is still `baec91b`.
- 30 files are modified and 56 are new (listed below). Untracked build outputs (`Debug/`, `Release/`, Android `build/`, `__pycache__`, backend `*.db`) are ignored by `.gitignore`.
- No branch change, rebase, reset, stash, clean, Git identity change or SoarOS change.

## Files created (56)

**Firmware, portable:**
- `Components/Storage/`: `NorFlash.hpp`, `EmergencyJournal.{hpp,cpp}`, `JournaledEmergencyStore.hpp`, `LogRecords.{hpp,cpp}`, `FlightLog.{hpp,cpp}`, `FlightRecorder.{hpp,cpp}`, `StorageService.{hpp,cpp}`
- `Components/Power/PowerManager.{hpp,cpp}`
- `Components/Output/Audio/PcmToneOutput.{hpp,cpp}`, `Components/Output/LED/GpioStatusLed.{hpp,cpp}`
- `Components/Sensors/LSM6DSRX/Lsm6dsrxFifo.{hpp,cpp}`
- `Components/Common/TaskSupervisor.{hpp,cpp}`

**Firmware, RTOS:** `Components/Tasks/StorageTask.{hpp,cpp}`

**Firmware tests:**
- `Tests/mocks/FakeNorFlash.hpp`
- `Tests/unit/test_storage_journal.cpp`, `test_emergency_persistence.cpp`, `test_power_output.cpp`
- `Tests/vectors/pglog_v1_records.bin`

**Backend:** `Backend/README.md`, `Backend/paraguardian_sos/{__init__,__main__,auth,config,providers,server,service,store}.py`, `Backend/tests/{__init__,test_backend}.py`

**Android:**
- `app/src/main/java/.../RemoteDelivery.kt`, `Lifecycle.kt`
- `app/src/main/res/xml/backup_rules.xml`, `network_security_config.xml`
- `app/src/debug/res/xml/network_security_config.xml`
- `app/src/test/.../DeliveryTest.kt`, `app/src/androidTest/.../M8LifecycleTest.kt`

**Tools / CI:** `Tools/check.sh`, `Tools/pglog/pglog_decode.py`, `Tools/pglog/test_pglog_decode.py`, `.github/workflows/ci.yml`

**Docs:** `Docs/MILESTONE8_SOFTWARE_RELIABILITY_DESIGN.md`, `MILESTONE8_VALIDATION_REPORT.md`, `MILESTONE8_WALKTHROUGH.txt`, `MILESTONE8_HARDWARE_BRINGUP.md`, `MILESTONE8_GIT_SUMMARY.md`

## Files changed (30)

**Firmware:**
- `Algorithms/Emergency/IEmergencyStore.hpp`: asynchronous durability interface; record v2.
- `EmergencyTypes.hpp`, `EmergencyConfig.hpp`: new flags, status fields and config values.
- `EmergencyManager.{hpp,cpp}`: durability, identity, restore policy, battery.
- `Algorithms/Safety/AlertTypes.hpp`, `AlertPriorityPolicy.hpp`, `SafetyEventDispatcher.{hpp,cpp}`: 3 appended alert kinds; system inputs.
- `Tasks/EmergencyAlertTask.{hpp,cpp}`: journal store, recorder, PowerManager.
- `Tasks/SensorProcessingTask.cpp`: recorder calls.
- `Tasks/HealthMonitorTask.{hpp,cpp}`: storage, power and supervisor fields.
- `Tasks/SystemHealth.hpp`, `Tasks/TaskConfig.hpp`, `SysCore/RunInterface.cpp`: storage task creation.
- `Algorithms/README.md`.

**Tests:** `Tests/mocks/MockEmergencyIo.hpp` (new store interface, async mode), `Tests/run_host_tests.sh` (10 sources added).

**Android:** `AndroidManifest.xml`, `Emergency.kt`, `GuardianApplication.kt`, `MainActivity.kt`, `MonitoringService.kt`, `Storage.kt`, `README.md`.

**Docs:** `HARDWARE_INTEGRATION_TODO.md` (§18), `RTOS_TIMING_ANALYSIS.md` (§4f).

**Repository:** `.gitignore`.
- Backend database and Python caches are now ignored.
- The CubeIDE output patterns are anchored as `/Debug/` and `/Release/`. Before this, `Debug/` also matched Android's `app/src/debug/` on case-insensitive macOS, which would have silently left the debug network-security config out of Git.

**Not modified:** no `.cproject`/`.ioc` change (the whole `Components/` tree is already a source folder), no SoarOS change, no pin/clock change, and no existing test assertion changed. Historical milestone reports were not rewritten.

## Suggested manual commit boundaries
1. `storage: NOR interface, power-loss-safe emergency journal, flight/critical logs, storage service` (Components/Storage, FakeNorFlash, test_storage_journal, golden vector, Tools/pglog)
2. `emergency: asynchronous durability, persistent incident identity, restore policy` (IEmergencyStore, Emergency*, MockEmergencyIo, test_emergency_persistence)
3. `power/outputs/fifo/supervisor: PowerManager, PCM tones, GPIO LED, LSM6DSRX FIFO, TaskSupervisor, system alerts` (+ test_power_output)
4. `tasks: StorageTask, recorder/power/supervisor integration, health fields` (Tasks/*, SysCore/RunInterface.cpp, run_host_tests.sh)
5. `backend: emergency delivery API with idempotent jobs and provider adapters` (Backend/)
6. `android: HTTPS delivery outbox, lifecycle recovery, Keystore credentials, lint fixes` (Android/)
7. `ci/docs: check script, workflow, Milestone 8 documentation` (.github, Tools/check.sh, Docs, READMEs, .gitignore)

Each boundary builds and passes on its own only in this order (2 depends on 1, 4 on 1–3, 6 on 5 for the end-to-end test).

## AI-use disclosure
This milestone was implemented with Claude (an AI assistant, Anthropic), working in the repository under the student's direction. The assistant:
- wrote the code, tests and documents listed above;
- ran the builds and tests reported in `MILESTONE8_VALIDATION_REPORT.md`;
- added no AI co-author trailers and created no commits.

The student should review every change, decide the commit boundaries, and be able to explain the design decisions; `MILESTONE8_WALKTHROUGH.txt` is written for that purpose. Hardware facts were deliberately left open where no schematic exists.
