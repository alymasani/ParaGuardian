# Milestone 8 – Validation Report

**Date:** 2026-11-19.
**Machine:** macOS (Darwin 24.6), Apple Clang 17, Python 3.9.6, STM32CubeIDE 2.2.0 (GNU Tools for STM32 14.3), Corretto 17, Gradle 8.11.1 (offline), Android emulator API 36 (ARM64 image `ParaGuardianM7`).
**Not available:** the PCB, the W25Q128 wiring, the battery and charger, the MAX98357A, the LEDs, the CYW20822, a physical Android phone, and any live messaging account.

**A passing host suite does not show physical emergency reliability.** No hardware, real crash or live-communication validation was performed.

## 1. Baseline (before any change, repository at `baec91b` on `main`)

| Check | Result |
|---|---|
| Firmware host tests (`run_host_tests.sh`, ASan+UBSan) | **473/473 passed** |
| Android JVM tests | 14 tests, 1 skipped, 0 failures |
| Android lint | **0 errors, 15 warnings** (M7 reported 16) |
| Android debug APK | built |
| CubeIDE Debug / Release | baseline **not re-run** before editing; the M6 figures (Debug text 156,636 / bss 59,288, Release text 89,080 / bss 59,288) are used as the reference, as in M6 |
| SoarOS | `426c1ef` (heads/master), clean |

**Toolchain note:** the system has no Java runtime on PATH. The first baseline attempt therefore failed with "Unable to locate a Java Runtime". The JDK and SDK that M7 left in `/private/tmp/pg-m7-tooling` were reused for every Android result here.

## 2. Final results by verification level

### 2.1 Host verified

| Suite | Command | Result |
|---|---|---|
| Firmware host tests, ASan + UBSan | `sh Tests/run_host_tests.sh` | **512/512 passed, 0 sanitizer reports** |
| Firmware host tests, plain | `SANITIZE=0 sh Tests/run_host_tests.sh` | 512/512 |
| Firmware host tests, ThreadSanitizer | `SANITIZE=thread sh Tests/run_host_tests.sh` | **512/512 passed, 0 TSan reports** |
| Backend | `cd Backend && python3 -m unittest discover -s tests -t .` | **18/18 OK** |
| Log decoder | `cd Tools/pglog && python3 -m unittest test_pglog_decode` | **2/2 OK** |
| Android JVM (`./gradlew testDebugUnitTest`) | EmergencyTest 14 + DeliveryTest 9 | **23/23 passed** with `PG_BLE_PEER` (the C++ crash/bridge fixture from `Tests/build_ble_peer.sh`); without it, 1 M7 test skips by design |
| Strict ARM compile of all new / changed portable files | arm-none-eabi-g++ `-O2 -Wall -Wextra -Wshadow -Wpedantic -fno-exceptions -fno-rtti` | **0 diagnostics** |

New firmware tests, by subsystem (39 new; 473 existing tests unchanged and passing):

| File | Tests | Covers |
|---|---|---|
| `test_storage_journal.cpp` | 17 | journal blank/commit/remount, rotation and wear, power cut at **every byte** (65 cut points), cut during rotation erase, CRC corruption fallback, all-corrupt, program failure / burned slot, flash unavailable, sequence exhaustion, pre-erase; log append/scan/resume, torn record, overwrite vs stop-when-full, cut during rotation erase; storage-service priority under a telemetry flood, coalescing + failed commits + recovery, flash absent, recorder round trip, golden vector |
| `test_emergency_persistence.cpp` | 12 | the real chain manager → store → service → journal → fake flash with resets |
| `test_power_output.cpp` | 13 | PowerManager (5), PCM tones (3), GPIO LED (1), system alerts (1), IMU FIFO (2), supervisor (1) |

### 2.2 Emulator verified (API 36, `./gradlew connectedDebugAndroidTest`)
**7/7 passed:**
- 4 existing M7 tests (persistence, notification permission, monitoring inactive after recreation, simulated automatic SOS);
- `keystoreCredentialsRoundTripAndPlaintextIsNotStored`: AndroidKeyStore AES-GCM; no plain HTTP off loopback; only ciphertext at rest;
- `databaseMigrationKeepsIncidentsAndOutboxSurvivesReopen`: a real version-1 (M7) database upgraded to version 2 keeps its incidents, and the outbox survives a reopen;
- `monitoringIsNotStartedUnlessRequested`.

### 2.3 Real-device verified
**Nothing.** No physical phone, PCB or flash chip was used.

### 2.4 Simulated only
- **Every flash behaviour:** FakeNorFlash models NOR semantics, timing (program 1 ms, erase 45 ms typical) and fault injection.
- **The cross-system path** Kotlin `RemoteDelivery` → real Python backend (loopback HTTP) → `FakeProvider`, in the JVM test `endToEndAgainstTheRealBackendWithItsFakeProvider` (it ran; it was not skipped).
- **Audio:** rendered into buffers; no speaker.
- **LED:** written to mock pins.
- **FIFO:** register-bus model.
- **Battery:** synthetic inputs.

### 2.5 Hardware / external-service blocked
- **Hardware blocked:**
  - W25Q128 wiring, which also means real erase/program timing and power-cut behaviour on silicon;
  - battery ADC, charger pins, regulator and amplifier enables;
  - I2S/SAI output, LED pins;
  - CYW20822 association;
  - IMU FIFO enablement;
  - IWDG;
  - on-target stack and CPU timing.
- **External-service blocked:** live Twilio delivery (no account; disabled by design), a public HTTPS deployment, a real carrier delivery report.

### 2.6 ThreadSanitizer
`SANITIZE=thread sh Tests/run_host_tests.sh`: **512/512 passed, 0 ThreadSanitizer warnings** (exit 0).
TSan checks the host concurrency tests (mailboxes, queues, inboxes). It **cannot** validate ISR interactions, FreeRTOS critical sections or the real scheduler.

## 3. Real commands executed (summary)

```
git push origin main                                     # requested by the user before M8 (8602405..baec91b)
sh Tests/run_host_tests.sh                               # baseline 473/473; final 512/512 (ASan+UBSan)
SANITIZE=0 / SANITIZE=thread sh Tests/run_host_tests.sh
STM32CubeIDE --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild \
  -data <scratch>/.ws -import <scratch>/ParaGuardian -build ParaGuardian/Debug   (and /Release)
arm-none-eabi-nm -u <objects>                            # undefined symbols / heap calls
arm-none-eabi-g++ ... -Wall -Wextra -Wshadow -Wpedantic  # strict pass
arm-none-eabi-g++ ... -fstack-usage                      # stack per function
python3 -m unittest discover -s tests -t .  (Backend), python3 -m unittest test_pglog_decode (Tools/pglog)
./gradlew --offline testDebugUnitTest lintDebug assembleDebug ; ./gradlew --offline connectedDebugAndroidTest
emulator -avd ParaGuardianM7 -no-window ; adb emu kill
```
The CI workflow (`.github/workflows/ci.yml`) was written but **not executed**: it runs on GitHub after a push, and no push of M8 was made. Its Linux host-test job has not been tried on Linux.

## 4. Flash fault injection and storage recovery

| Case | Result |
|---|---|
| Power cut at each of 65 byte positions of a commit | 64× previous record, 1× new record committed, **0 garbage**. 4 intact-uncommitted bodies were reported separately. The journal kept working after every cut. |
| Cut during the rotation erase | latest record recovered from the other sector; the sector was re-erased on the next commit |
| Bit flip in the newest slot | previous record used; `tornAtMount = 1` |
| All slots corrupt | `Corrupt` reported; the journal is usable again with a higher sequence |
| Program failure | slot burned, next slot committed, sequence never reused |
| 300 commits with resets every 37 | always the newest record; 3 erases in total (64 slots per sector) |
| Pre-erase | the rotating commit costs 0 erases and ≤ 3 ms of virtual flash time |
| Write timeout (persistent) | failed ticket reported after 3 attempts, `emergencyDegraded`, recovered with backoff once writes succeed |
| Coalescing | 2 saves before the commit give 1 commit; `coalesced = 1`; the newest content is on flash |
| Log: cut inside the 2nd of 3 new records | 21 of 22 records recovered (all that were complete); writing resumes at seq 22 |
| Log: cut during rotation erase | head falls back to the previous sector; sequences stay ordered; the next append re-erases |
| Log rotation, 3-sector ring, 1000 × 56 B records | newest 168 retained (64 per sector), contiguous and ordered |
| StopWhenFull | oldest records kept, new ones refused and counted |
| Flash absent | store `Unavailable`, `save()` → `Rejected`, ticket 0, service never spins |

## 5. Emergency reboot cases (real manager + journal + fake flash, reset = all RAM objects destroyed)

| Case | Result |
|---|---|
| First emergency, blank journal | id 1; SOS sent only after the id was committed; no volatile flag |
| Storage starved | SOS held (bounded); sent as soon as the commit landed |
| Storage permanently failing | SOS sent once `Failed` is reported (≈ 2 s), marked `IncidentIdVolatile`, flags `IdentityUnconfirmed` + `PersistenceDegraded` |
| Reset during countdown | same id, full countdown, `countdownRestarts` 1, 2; 3rd reset → request created at restore (`RepeatedResetEscalated`) and sent |
| Reset during transmission | `SosPending`, same id, revision 2, `Restored` bit |
| Reset after phone acceptance | stage `AcceptedByPhone` kept; re-sent with the same id |
| Reset after delivery | `Delivered` kept; **nothing re-sent** in 3 s; next incident gets id 2 |
| Reset during cancellation | cancel notice re-sent for the same id |
| 25 boots, every third reset before commit | 16 SOS requests, **16 distinct ids** |
| Torn record (commit word cut) | state not trusted (Idle); `nextIncidentId = 3` (id 2 never reused) |
| Corrupt journal | `IdentityUncertain`, requests marked volatile, flag persists across resets |
| Counter exhaustion | `IdentityExhausted` persisted, wraps to 1, volatile marking, no pointless hold. **This test found a real defect:** the hold could never end after a wrap. Fixed. |
| Flash unavailable | `PersistenceUnavailable`, SOS sent immediately |
| uint32 ms wrap during countdown + commit | sent, committed, persistence latency < 100 ms |

## 6. Logging results
- The recorder round trip through flash with the Detailed profile for 2 s produced 2 critical records (boot + one emergency transition; an unchanged status was not logged twice) and 20 telemetry records (10 Hz).
- The golden vector `pglog_v1_records.bin` (37 bytes) is identical between the C++ encoder and the Python decoder. Flipping one bit makes it rejected.
- **Decoder region test:**
  - sectors are ordered by header sequence, not position;
  - a torn page is counted;
  - a garbage sector (interrupted erase) is ignored.

## 7. Android and backend

**Backend (18 tests):**
- duplicates and lost-response retries produce exactly one message per contact;
- a newer revision does not resend;
- provider-accepted is not delivered;
- callbacks are applied once, never downgraded, and an unknown message id is ignored;
- a timeout after send stays `unknown` and is not resent; it is re-sent only by an explicit `retry-unknown`;
- worker crash → lease expiry → `unknown`;
- 429/5xx retried with backoff, then `failed` after 5 attempts;
- cancel before dispatch sends nothing; cancel after send sends a cancellation notice only to the contacts who may have been alerted;
- an alert after cancel is ignored;
- test incidents never reach the live provider;
- volatile keys are separate incidents;
- schema validation (11 malformed inputs rejected), contact consent / E.164 / limit, both rate limits, the 30-day purge;
- Twilio request construction and error mapping, with an injected opener and **no network**;
- the live-delivery switch requires an explicit flag and credentials;
- the Twilio webhook signature matches the documented example vector.

**Over real loopback HTTP:**
- submit → duplicate → status → simulated callback → delivered;
- bad secret, unknown device, clock skew, tampered body and replayed nonce → 401;
- oversize body → 413; unknown incident → 404;
- the health endpoint contains no phone numbers;
- plain HTTP on a non-loopback address is refused.

**Android DeliveryTest (9):**
- signature equals the Python reference vector;
- offline → 503 → accepted with an identical body each time, never "delivered" from an HTTP 202;
- polling to provider_accepted → delivered; no downgrade; MCU report codes;
- duplicates, revision / cancel resubmission, 401 rejected and not retried;
- volatile key and outbox repair after a kill between the two writes;
- TLS rule;
- monitoring state function (8 cases);
- JSON helper;
- **end-to-end against the real backend**.

**No real emergency message was sent.** Every test used the fake provider, loopback addresses or injected transports.

**Lint:**
- **15 → 7 warnings, 0 errors.**
- Fixed: 8 `UseKtx` (transactions now use KTX; preference writes go through one helper that keeps `commit()`'s result, with a justified suppression) and `DataExtractionRules` (a real `backup_rules.xml`).
- Remaining 7: `GradleDependency` / `AndroidGradlePluginVersion` "newer version available". Upgrading needs network access and is a separate, risk-bearing change.

## 8. Timing and memory

| Build | text | data | bss | Δ text vs M6 | Δ bss |
|---|---|---|---|---|---|
| Debug | 191,548 | 104 | 68,080 | +34,912 | +8,792 |
| Release | 105,984 | 104 | 68,072 | **+16,904** | **+8,784** |

Both builds: **0 errors, 0 warnings**, no undefined symbols. Flash used (Release) is about 104 KB of 2 MB; static RAM is about 66.5 KB of RAM_D1 (512 KB).

| Object (Release) | Size |
|---|---|
| `StorageTask` (new) | 7,320 B: 3 KiB stack, critical queue 16 × 56 B, telemetry queue 32 × 56 B, service + journal + 2 logs (page buffers) |
| `EmergencyAlertTask` | 5,484 → 6,536 B (+PowerManager, power mailbox) |
| `g_systemHealth` | 904 → 976 B (+28 fields) |
| Emergency journal on flash | 8 KiB |
| Audio buffer | 0 B (planned: 256-frame mono double buffer = 1 KiB, allocated only once I2S exists) |

- **Heap:** `arm-none-eabi-nm -u` over all 11 new firmware objects shows no `malloc`, `new` or `free`. No steady-state heap allocation is introduced.
- **Stack** (`-fstack-usage`, `-Os`, per function):
  - `FlightLog::scan` 376 B (host export only);
  - `resumeInHead` 368 B, `openSector` 296 B, `flush` 280 B;
  - `EmergencyJournal::commit` 104 B;
  - `FlightRecorder::logEmergency` 112 B (emergency task).
  - Deepest storage chain: step → drain → append → advancePage → openSector, estimated under 1 KB of the 3 KiB stack. The emergency task gains ≤ 112 B and the processing task ≤ 96 B.
  - **Estimates**, not on-target high-water marks.
- **Host CPU** (Apple M-series, `-O2`): `save` + journal commit on the fake flash 0.43 µs; `FlightRecorder::logNavigation` 0.013 µs per call (mostly rate-limited early returns); `PowerManager::update` 0.006 µs.
- **Target estimate (not measured):** encode + push of one record ≈ 100–200 cycles; well under 0.05 ms per frame at 64 MHz.
- **Flash time** (datasheet, not measured): page program typ 0.4 ms / max 3 ms; sector erase typ 45 ms / max 400 ms. The worst emergency-commit wait is therefore one erase (400 ms) plus 2 programs.

## 9. Failures, unmet requirements, known limitations

1. **No hardware validation.** Everything in §2.5 is untested on silicon.
2. **Flash not wired:** firmware runs `UnconfiguredNorFlash`, so the firmware is still non-persistent in practice. This is reported honestly in `EmergencyStatus` and `g_systemHealth`.
3. **Volatile ids on Android:** a volatile incident id that repeats after an MCU reset is a separate incident on the server (`<id>-v<received>`). On the local SMS / simulation path it is still keyed on (device, id); the newer revision is treated as the same incident.
4. **Android lifecycle paths not exercised:**
   - START_STICKY restart, BOOT_COMPLETED restart and companion-presence restart are implemented but **not observed on a device**;
   - physical monitoring requires an associated, verified CYW20822, which does not exist, and `PhysicalTrust` still returns no verified peer;
   - force-stop recovery is impossible by Android policy.
5. **FIFO not enabled** in acquisition (register confirmation pending); **IWDG not enabled**.
6. **Backend secret storage:** the server stores HMAC secrets in SQLite. Production needs a secrets manager and TLS termination. No deployment was made, by design.
7. **Gradle dependency upgrades not done** (offline toolchain); 7 informational lint warnings remain.
8. **Baseline CubeIDE build not re-run before editing** (as in M6); M6 figures are used for the deltas.
9. **CI workflow not executed.**
10. **The countdown policy is a design choice:** a full restart, at most 2 restarts per incident. Like every M6 timing value, it needs human-factors validation.

## 10. Test expectations corrected during development (not production changes)
Each case below was a wrong assumption in a NEW Milestone 8 test. None of the 473 existing tests was changed.

| Test | Wrong expectation | Correct behaviour (kept) |
|---|---|---|
| `journal_maintenance_pre_erases...` | maintenance due on a blank journal | a blank journal's second sector is already erased; the test now rotates first |
| `log_rotation_overwrites...` | > 400 of 1000 records retained in 3 sectors | 64 × 56-byte records per sector; about 2.6 sectors are live |
| `persist_first_emergency...` | status `Committed` right after construction | status updates on `tick()`; `Pending` before, `Committed` after (now asserted both ways) |
| `persist_sos_waits_bounded...` | permanently failing flash waits the full 2 s hold | once the store REPORTS failure, waiting is pointless; sent at about 2 s with the volatile flag |
| `persist_incidents_across_many_resets...` | every transport submission is an alert | cancel notices are submissions too; the test counts alerts only |
| `persist_torn_record...` | cancelled incident re-arms without acknowledgment | a cancel notice that has gone out must be acknowledged first (M6 rule); the test acknowledges it |
| `power_rail_and_amplifier...` | amplifier ready at 60 ms | the amplifier settles 20 ms after the rail (ready at 40 ms) |

**Production defects found by the new tests (fixed):**
1. The identity hold never ended after id exhaustion (`EmergencyManager::serviceTransport`).
2. `GpioStatusLed` skipped writing the first "on" phase.
