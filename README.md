# ParaGuardian

ParaGuardian is a flight-safety and crash-alert system for paragliding pilots. It has three parts:

1. **Firmware** for an STM32H743 board. It reads redundant sensors, estimates attitude and vertical motion, warns about unsafe landings, detects suspected crashes, and runs an SOS countdown that the pilot can cancel.
2. **An Android companion app.** It receives SOS requests over Bluetooth LE, stores them durably, and delivers them to emergency contacts.
3. **An emergency backend.** It delivers SOS messages without the pilot having to do anything, and reports delivery status honestly.

> **Status (Milestone 8, November 2026):**
> - All hardware-independent software is implemented and tested on a laptop, on the Android emulator and in native STM32CubeIDE builds.
> - **Nothing has been validated on the real PCB.** The schematic does not exist yet, so the firmware runs with honest "unconfigured" backends for the flash, battery, audio, LEDs and the BLE module.
> - **No real emergency message has ever been sent.** Live delivery is disabled by default.

---

## Contents
1. [Architecture](#1-architecture)
2. [Repository layout](#2-repository-layout)
3. [Hardware](#3-hardware)
4. [Quick start: run every check](#4-quick-start-run-every-check)
5. [Firmware](#5-firmware)
6. [Firmware host tests](#6-firmware-host-tests)
7. [Emergency backend](#7-emergency-backend)
8. [Android companion](#8-android-companion)
9. [Flash logs and the decoder](#9-flash-logs-and-the-decoder)
10. [Continuous integration](#10-continuous-integration)
11. [What works today, and what needs the PCB](#11-what-works-today-and-what-needs-the-pcb)
12. [Documentation index](#12-documentation-index)
13. [Safety notice](#13-safety-notice)

---

## 1. Architecture

```
 Sensors (3x LSM6DSRX IMU, 2x BMP585 baro, MMC5983MA mag, NEO-M9N GNSS, TF02-Pro LiDAR)
   │
   ▼  SensorAcquisitionTask (priority 40, 100 Hz)
 Preprocessing: validation, calibration, axis alignment, redundancy voting
   │
   ▼  SensorProcessingTask (32)
 Madgwick attitude ─► 3-state vertical UKF ─► flight state + landing safety ─► crash detector
   │                                                                        │
   │ FlightRecorder (logs)                                                   ▼
   │                                       EmergencyAlertTask (36): EmergencyManager (countdown, SOS, retries)
   │                                         ├─► SafetyEventDispatcher ─► audio / LED patterns
   │                                         ├─► PowerManager (battery, 5 V rail, amplifier)
   │                                         └─► BLE bridge ──► Android app
   ▼                                                                         │
 StorageTask (20): emergency journal, critical log, flight log on W25Q128    │
 HealthMonitorTask (16): g_systemHealth, task supervision                    │
                                                                             ▼
 Android: EmergencyEngine ─► durable outbox ─HTTPS─► Backend API ─► provider (fake / Twilio) ─► consented contacts
```

**Design rules followed throughout:**
- Portable C++17 algorithms with no HAL calls, so they all run on a laptop.
- Static allocation only, and no heap use in steady state.
- Bounded queues: a full queue drops and counts, it never blocks.
- Every stage reports what it really knows. "Queued" is not "saved". "Provider accepted" is not "delivered". A missing battery reading is "Unknown", never "Healthy".

The full design is in [Docs/](Docs/). Start with [MILESTONE8_SOFTWARE_RELIABILITY_DESIGN.md](Docs/MILESTONE8_SOFTWARE_RELIABILITY_DESIGN.md).

## 2. Repository layout

| Path | What it is |
|---|---|
| `Components/Sensors/` | sensor drivers (LSM6DSRX + FIFO, BMP585, MMC5983MA, NEO-M9N UBX, TF02-Pro) |
| `Components/Acquisition/`, `Processing/` | acquisition scheduler, data hub, preprocessing pipeline |
| `Components/Algorithms/` | attitude, navigation (UKF), flight / landing safety, crash, emergency, alert dispatch ([README](Components/Algorithms/README.md)) |
| `Components/Storage/` | W25Q128 driver, power-loss-safe emergency journal, flight / critical logs, storage service |
| `Components/Power/` | PowerManager (battery, charging, 5 V rail, amplifier policy) |
| `Components/Output/` | audio (scheduler + PCM tone renderer) and LED (scheduler + GPIO backend) |
| `Components/Communications/Bluetooth/` | BLE protocol, SOS message, emergency bridge |
| `Components/Tasks/` | FreeRTOS tasks (acquisition, processing, emergency, storage, health) and `TaskConfig.hpp` |
| `Components/BSP/`, `SysCore/` | the one place hardware is bound (empty until the schematic exists), boot glue |
| `Core`-style folders (`Src/`, `Inc/`, `Drivers/`, `Middlewares/`, `Startup/`), `ParaGuardian.ioc` | STM32CubeMX-generated project |
| `SoarOS/` | team RTOS framework (git submodule, **never modified**) |
| `Tests/` | host unit tests, simulators, mocks, shared test vectors |
| `Backend/` | emergency delivery backend (Python standard library) ([README](Backend/README.md)) |
| `Android/ParaGuardian/` | Android companion (Kotlin, Jetpack Compose) ([README](Android/ParaGuardian/README.md)) |
| `Tools/` | `check.sh` (all checks), `pglog/` (flash-log decoder) |
| `Docs/` | design documents, validation reports, beginner walkthroughs and hardware checklists for each milestone |
| `.github/workflows/ci.yml` | CI for host tests, backend and Android |

## 3. Hardware

| Part | Role |
|---|---|
| STM32H743ZIT6 (Cortex-M7, FreeRTOS 10.6.2 via CMSIS-RTOS v2, TIM6 HAL timebase) | MCU |
| 3× LSM6DSRX | IMUs |
| 2× BMP585 | barometers |
| MMC5983MA | magnetometer |
| NEO-M9N | GNSS |
| TF02-Pro | LiDAR (ground clearance) |
| CYW20822 | Bluetooth LE |
| W25Q128JV | 16 MiB SPI NOR flash (journal + logs) |
| MAX98357A | I2S audio amplifier |

Pin assignments, polarities and timings are **not** in the code until the schematic exists. See [Docs/MILESTONE8_HARDWARE_BRINGUP.md](Docs/MILESTONE8_HARDWARE_BRINGUP.md) and [Docs/HARDWARE_INTEGRATION_TODO.md](Docs/HARDWARE_INTEGRATION_TODO.md).

## 4. Quick start: run every check

```sh
git clone --recurse-submodules https://github.com/alymasani/ParaGuardian.git
cd ParaGuardian
sh Tools/check.sh all
```

| Step | Needs | Command |
|---|---|---|
| Firmware host tests, ASan + UBSan | a C++17 compiler (clang++ or g++) | `sh Tools/check.sh firmware` |
| Firmware host tests, ThreadSanitizer | clang++ / g++ with TSan | `sh Tools/check.sh tsan` |
| Firmware host tests, no sanitizers (fast) | a C++17 compiler | `sh Tools/check.sh plain` |
| Backend tests | Python 3.9+ | `sh Tools/check.sh backend` |
| Log decoder tests | Python 3.9+ | `sh Tools/check.sh decoder` |
| Android JVM tests, lint, debug APK | JDK 17 + Android SDK 36 | `sh Tools/check.sh android` |
| Native firmware builds (Debug + Release) | STM32CubeIDE (macOS default path, or set `CUBEIDE=`) | `sh Tools/check.sh cubeide` |

Steps whose tools are missing are reported as **SKIPPED**, never as passed.

## 5. Firmware

**Build system:** the native **STM32CubeIDE** project. There is no CMake or PlatformIO.

**Build in the IDE:**
1. Clone with submodules (`SoarOS`).
2. Open STM32CubeIDE (tested with 2.2.0, GNU Tools for STM32 14.3).
3. Choose *File → Import → Existing Projects into Workspace* and select the repository folder.
4. Build the **Debug** or **Release** configuration. The whole `Components/` tree is already a source folder, so new files need no project changes.

**Build headlessly (what `Tools/check.sh cubeide` does, on a scratch copy):**
```sh
/Applications/STM32CubeIDE.app/Contents/MacOS/STM32CubeIDE --launcher.suppressErrors -nosplash \
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
  -data /tmp/ws/.ws -import /tmp/ws/ParaGuardian -build ParaGuardian/Release
```

**Flash and debug:** use the IDE's debug configuration over ST-LINK/SWD. With no debug UART on the board yet, add `g_systemHealth` to *Live Expressions*. It shows:
- task stacks and timing;
- sensor health;
- navigation and safety state;
- emergency state and its durability;
- storage mounts;
- power state;
- supervisor verdicts.

**Boot sequence:**
1. `main.c` (CubeMX).
2. `paraguardian_create_tasks()` in `Components/SysCore/RunInterface.cpp` creates, in order: acquisition, processing, storage, emergency, then health.
3. `osKernelStart()`.

**Configuration:** every tunable value lives in one config struct per module. Examples: `TaskConfig.hpp`, `EmergencyConfig.hpp` (countdowns, retries, identity hold), `CrashConfig.hpp`, `PowerManager.hpp` (`BatteryConfig`, `PowerPolicyConfig`). Many values are engineering placeholders and are labelled as such.

**Release footprint (Milestone 8):**
- text 105,984 B, bss 68,072 B;
- 0 errors and 0 warnings in both configurations.

## 6. Firmware host tests

The portable code (drivers, algorithms, storage, power, outputs) is compiled together with the unit tests and fakes using the laptop's compiler:

```sh
sh Tests/run_host_tests.sh                    # ASan + UBSan (default)
SANITIZE=thread sh Tests/run_host_tests.sh    # ThreadSanitizer
SANITIZE=0 sh Tests/run_host_tests.sh         # fastest
CXX=g++ sh Tests/run_host_tests.sh            # choose a compiler
```

**Current result: 512/512 passing** under all three modes. Highlights:
- **Simulation scenarios:** sensor, attitude, UKF, flight-safety and crash/emergency scenarios (`Tests/sim/`).
- **Flash fault injection** with `Tests/mocks/FakeNorFlash.hpp`: power cut at every byte of a journal commit, interrupted erases, bit rot, timeouts, missing flash.
- **Reset recovery:** each reset test destroys all RAM objects and keeps only the fake flash. It covers resets during the countdown, during transmission, after phone acceptance, after delivery and during cancellation, and checks that incident IDs stay unique across 25 boots.
- **Shared vectors:** `Tests/vectors/` is used by both the C++ and the Kotlin / Python tests (BLE frames, flash log records).

The BLE peer fixture used by the Android JVM tests is built with `sh Tests/build_ble_peer.sh <output-path>`.

## 7. Emergency backend

This is the reference implementation of unattended SOS delivery. It uses only the Python standard library.

```sh
cd Backend
python3 -m unittest discover -s tests -t .                                   # 18 tests, fake provider, loopback only

export PG_DB=/tmp/paraguardian.db
python3 -m paraguardian_sos provision-device pg-dev-1   # prints {"deviceId":..,"secretHex":..} ONCE
python3 -m paraguardian_sos serve                       # http://127.0.0.1:8443, fake provider, no real messages
python3 -m paraguardian_sos diagnostics                 # aggregate counts
python3 -m paraguardian_sos purge                       # remove location data older than 30 days
```

**Safety properties:**
- **Authentication:** every request is signed per device (HMAC-SHA256 with timestamp and nonce). Replays, tampered bodies and unknown devices are rejected.
- **No duplicates:** submissions are idempotent. Each outbound message exists once per (incident, contact, purpose). A provider timeout or a worker crash becomes `unknown` and is **never resent automatically**.
- **Honest stages:** the status moves through `pending` → `provider_accepted` → `delivered` (from a provider report) or `failed` / `unknown`.
- **Recipients:** only the server's stored, consented contacts receive messages (at most 5, E.164). An incident cannot name its own recipients.
- **TLS:** required on any non-loopback address (`PG_TLS_CERT`, `PG_TLS_KEY`).
- **Real delivery (Twilio adapter)** needs **both** `PG_PROVIDER=twilio`, `PG_LIVE_DELIVERY_ENABLED=true` **and** the credentials (`TWILIO_ACCOUNT_SID`, `TWILIO_AUTH_TOKEN`, `TWILIO_FROM`, `PG_PUBLIC_BASE_URL`), supplied through environment variables only.

The full API and configuration are in [Backend/README.md](Backend/README.md). The live setup and a safe test procedure are in [MILESTONE8_HARDWARE_BRINGUP.md §14–15](Docs/MILESTONE8_HARDWARE_BRINGUP.md).

## 8. Android companion

**Requirements:** JDK 17, Android SDK Platform 36. The Gradle wrapper pins 8.11.1.

```sh
cd Android/ParaGuardian
./gradlew :app:assembleDebug :app:testDebugUnitTest :app:lintDebug
./gradlew :app:connectedDebugAndroidTest          # with an emulator or phone attached

# include the C++ crash/BLE fixture in the JVM tests
sh ../../Tests/build_ble_peer.sh /tmp/pg_ble_peer
PG_BLE_PEER=/tmp/pg_ble_peer ./gradlew :app:testDebugUnitTest --rerun-tasks
```

**Results:**
- JVM tests: 23/23. They include an end-to-end test where the Kotlin client talks to the real Python backend over loopback, when `python3` is available.
- Emulator tests (API 36): 7/7.
- Lint: 0 errors. The 7 remaining warnings are informational "newer dependency version" notices.

**Using the app:**
- **Simulation (default).** Connect the simulator in *Setup*. Then in *Emergency*, choose *Flight* → *Suspect crash* → *Advance 31 seconds*. The SOS goes through the in-app mock provider. Fault controls can drop or delay reports, corrupt frames, remove GNSS or the network, and change the provider's response.
- **Physical mode.** Associate the device with Android's companion picker, then *Start monitoring*. This starts a connected-device foreground service. Physical SOS traffic needs the provisioned CYW20822 and authenticated peer verification (still open, see §11).
- **Unattended delivery.** In *Settings → Unattended emergency delivery*, enter the server URL, device id and secret from `provision-device`. The secret is encrypted with an Android Keystore key. Release builds accept only HTTPS. Debug builds also allow `http://127.0.0.1` / `http://10.0.2.2` for the local backend; `adb reverse tcp:8443 tcp:8443` makes the host's backend reachable from the emulator. *Simulation sends TEST incidents to the server* exercises the whole path; test incidents never reach a live provider.
- **SMS fallback.** The independent phone SOS opens the messaging app for a consented contact; the user confirms the send. The app then reports the delivery as unknown.

**Monitoring recovery:**
- The app keeps the user's request separate from what is actually running, and shows that honestly ("Monitoring interrupted · tap Resume").
- It restarts after process death (START_STICKY), after a reboot (boot receiver) and when the device appears (companion presence, Android 12+).
- After a **force stop** the user must open the app again. This is Android policy and cannot be bypassed.
- Pending SOS submissions resume through a persisted JobScheduler job.

## 9. Flash logs and the decoder

The W25Q128 partition plan is defined in `Components/Storage/NorFlash.hpp`:

| Region | Address | Size |
|---|---|---|
| reserved (provisioning) | 0x000000 | 64 KiB |
| emergency journal | 0x010000 | 8 KiB |
| critical log | 0x012000 | 56 KiB |
| flight log | 0x020000 | ≈15.9 MiB |

**Logging profiles:** Minimal, Normal (default, about 50 B/s, about 92 h of retention), Detailed, and Emergency (automatic while an incident is open). Raw sensor samples are not logged.

Decode a dump of a log partition:
```sh
python3 Tools/pglog/pglog_decode.py flightlog.bin --region > flight.csv
python3 Tools/pglog/pglog_decode.py criticallog.bin --region --json
```
Records whose CRC fails (torn writes or corruption) are counted, never guessed.

## 10. Continuous integration

`.github/workflows/ci.yml` runs on every push and pull request:
- firmware host tests (ASan/UBSan and TSan);
- backend and decoder tests;
- Android JVM tests, lint and the APK build.

STM32CubeIDE cannot run on hosted runners, so the native firmware build stays a local step (`sh Tools/check.sh cubeide`). CI uses no secrets.

## 11. What works today, and what needs the PCB

| Area | Software | Hardware status |
|---|---|---|
| Sensor drivers, preprocessing, attitude, UKF, flight / landing safety, crash detection | implemented, host-tested | sensors not wired (`BoardConfig.cpp` empty) |
| Emergency manager, SOS countdown / retries / cancellation | implemented, host-tested | n/a |
| Emergency persistence + incident identity, flight logging | implemented, fault-injection tested | W25Q128 SPI not wired → firmware uses `UnconfiguredNorFlash` |
| BLE protocol and Android app | implemented, emulator + simulation tested | CYW20822 transport and authenticated pairing open |
| Automatic SOS backend | implemented, tested with the fake provider | live provider never contacted; no deployment |
| Power management | implemented, host-tested | battery ADC, charger pins, enable pins unknown |
| Audio (PCM tones) / LED | implemented, host-tested | I2S/SAI, amplifier and LED pins unknown |
| IMU FIFO impact capture | decoder + bounded drain tested | not enabled until the registers are confirmed and it is measured |
| Watchdog | supervision policy tested | IWDG not enabled |
| Paraglider aerodynamic process model | **not started** (next phase) | needs flight data |

The step-by-step bring-up list is in [Docs/MILESTONE8_HARDWARE_BRINGUP.md](Docs/MILESTONE8_HARDWARE_BRINGUP.md).

## 12. Documentation index

| Milestone | Documents |
|---|---|
| M1 sensor acquisition and hardening | `FIRMWARE_RELIABILITY_AUDIT.md`, `FIRMWARE_REGRESSION_REPORT.md`, `SENSOR_DRIVER_TEST_REPORT.md`, `FIRMWARE_HARDENING_WALKTHROUGH.txt`, `PARAGUARDIAN_FIRMWARE_WALKTHROUGH.txt`, `CMSIS_V2_SOAROS_MIGRATION.md` |
| M2 preprocessing and calibration | `SENSOR_PREPROCESSING_DESIGN.md`, `_TEST_REPORT.md`, `_WALKTHROUGH.txt` |
| M3 Madgwick attitude | `MADGWICK_ATTITUDE_DESIGN.md`, `MADGWICK_VALIDATION_REPORT.md`, `MADGWICK_ATTITUDE_WALKTHROUGH.txt` |
| M4 vertical UKF | `VERTICAL_UKF_DESIGN.md`, `_VALIDATION_REPORT.md`, `_WALKTHROUGH.txt` |
| M5 flight state and landing safety | `FLIGHT_LANDING_DESIGN.md`, `_VALIDATION_REPORT.md`, `_WALKTHROUGH.txt` |
| M6 crash detection and emergency | `CRASH_EMERGENCY_DESIGN.md`, `_VALIDATION_REPORT.md`, `_WALKTHROUGH.txt` |
| M7 BLE and Android | `BLE_ANDROID_ARCHITECTURE.md`, `_VALIDATION_REPORT.md`, `_HARDWARE_INTEGRATION.md`, `_WALKTHROUGH.txt` |
| M8 reliability, storage, delivery, power | `MILESTONE8_SOFTWARE_RELIABILITY_DESIGN.md`, `_VALIDATION_REPORT.md`, `_WALKTHROUGH.txt`, `_HARDWARE_BRINGUP.md`, `_GIT_SUMMARY.md` |
| Cross-cutting | `HARDWARE_INTEGRATION_TODO.md`, `RTOS_TIMING_ANALYSIS.md`, per-milestone `*_GIT_SUMMARY.md` |

## 13. Safety notice

ParaGuardian is a student capstone project and is **not certified safety equipment**:
- The crash detection thresholds, countdown durations and alert priorities are engineering placeholders. They have not been validated with real flights or crashes.
- A "suspected crash" is not a confirmed crash.
- "Delivered" means a carrier delivery report, not that a person read the message.

Never rely on it as your only means of calling for help, and never test it against real emergency service numbers.
