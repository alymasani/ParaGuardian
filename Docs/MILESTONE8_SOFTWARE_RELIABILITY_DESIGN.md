# Milestone 8 – Software Reliability Design

**Scope:** emergency persistence and incident identity, flash logging, unattended SOS delivery, Android lifecycle recovery, power management, audio/LED outputs, IMU FIFO capture, supervision.
**Status:** implemented and tested on the host, the Android emulator and in native CubeIDE builds. **Not validated on a PCB.** The flash, battery ADC, charger pins, regulator and amplifier enables, I2S, LEDs and the CYW20822 are not wired in firmware, because the schematic does not exist yet.

Results are in [MILESTONE8_VALIDATION_REPORT.md](MILESTONE8_VALIDATION_REPORT.md) and bring-up steps in [MILESTONE8_HARDWARE_BRINGUP.md](MILESTONE8_HARDWARE_BRINGUP.md).

---

## 1. System architecture after Milestone 8

```
 Sensors ─► SensorAcquisitionTask (40) ─► SensorProcessingTask (32) ─► Crash / Safety ─────────────┐
                                                │ FlightRecorder.logNavigation / logQuality / logCrash │
                                                ▼                                                     ▼
                                     telemetry LogQueue<32>                         EmergencyAlertTask (36)
                                                                                    EmergencyManager ─► ISosTransport (BLE bridge)
                                     critical LogQueue<16> ◄── FlightRecorder ◄──── │ SafetyEventDispatcher ─► Audio / LED schedulers
                                                                                    │ PowerManager (1 Hz)    (PcmToneOutput, GpioStatusLed)
                                                                                    ▼
                                                               JournaledEmergencyStore (RAM copy + ticket)
                                                                                    │ notify
                                                                                    ▼
             StorageTask (20) ── StorageService ── 1. EmergencyJournal commit  ─► W25Q128 (INorFlash)
                                                    2. critical FlightLog
                                                    3. telemetry FlightLog
                                                    4. journal maintenance
 HealthMonitorTask (16) ── TaskSupervisor (policy) ── g_systemHealth

 Phone:  BLE ─► Guardian ─► EmergencyEngine ─► RemoteQueue ─► outbox (SQLite) ─► RemoteDelivery ─HTTPS─►
         Backend API (HMAC auth, idempotent) ─► jobs (SQLite, leases) ─► provider (Fake | Twilio, disabled by default)
```

Task priorities: acquisition 40 > emergency 36 > processing 32 > storage 20 > health 16. These are enforced with `static_assert` in `TaskConfig.hpp`.

## 2. New modules

| Path | Purpose |
|---|---|
| `Components/Storage/NorFlash.hpp` | `INorFlash`, the W25Q128 adapter, `UnconfiguredNorFlash`, and the **partition plan** (§4.4) |
| `Components/Storage/EmergencyJournal.*` | Append-only, power-loss-safe journal of 32-byte emergency records |
| `Components/Storage/JournaledEmergencyStore.hpp` | `IEmergencyStore` with asynchronous durability (tickets) |
| `Components/Storage/LogRecords.*` | Versioned binary record schema, framing, `LogQueue` (bounded MPSC) |
| `Components/Storage/FlightLog.*` | Rotating log ring with torn-write recovery (used for both the flight log and the critical log) |
| `Components/Storage/FlightRecorder.*` | What is logged and how often (profiles); routing to the critical or telemetry queue |
| `Components/Storage/StorageService.*` | Priority scheduling of every flash operation |
| `Components/Tasks/StorageTask.*` | Static FreeRTOS task (priority 20, 3 KiB stack) running the service |
| `Components/Power/PowerManager.*` | Battery, charge and supply interpretation; 5 V rail and amplifier policy |
| `Components/Output/Audio/PcmToneOutput.*` | PCM tone renderer and `IAudioAlert`, gated by the amplifier |
| `Components/Output/LED/GpioStatusLed.*` | GPIO LED backend for the existing pattern table |
| `Components/Sensors/LSM6DSRX/Lsm6dsrxFifo.*` | FIFO configuration, tagged-word decoder, bounded drain with peak extraction |
| `Components/Common/TaskSupervisor.*` | Heartbeat supervision and an emergency-safe watchdog policy |
| `Backend/paraguardian_sos/` | Emergency API, idempotent job store, providers (Python standard library only) |
| `Android/.../RemoteDelivery.kt` | Request signer, HTTPS transport, durable outbox engine |
| `Android/.../Lifecycle.kt` | Monitoring state, boot receiver, companion presence service, JobScheduler job, Keystore credentials |
| `Tools/pglog/pglog_decode.py` | Flash-log decoder to CSV / JSON |
| `Tools/check.sh`, `.github/workflows/ci.yml` | Reproducible checks |

Dependency direction: the Algorithms code knows only `IEmergencyStore`. The Storage code knows the record bytes, but not the manager. Only the Tasks code knows about FreeRTOS, and every new portable file builds on the host.

## 3. Emergency persistence

### 3.1 Interface change (why)
The Milestone 6 interface had a synchronous `bool save()`. With a real flash, a synchronous save would either block the emergency task during an erase (up to 400 ms) or return `true` for a write that has only been queued. Both are wrong. The revised `IEmergencyStore::save()` therefore never blocks, and returns one of three answers:

- `Rejected`: the store cannot take the record.
- `Queued`: the record is in RAM only.
- `Committed`: the record was written before `save()` returned. Only the RAM test double does this.

It also returns a **ticket**. The record is durable only once `committedTicket() >= ticket`. The manager polls this in `tick()` and reports `EmergencyStatus::persist` as `Disabled`, `Pending`, `Committed` or `Failed`. It also sets the flags `PersistencePending` and `PersistenceDegraded`, and the field `persistLatencyMaxMs`. **Queued is never reported as saved.**

Coalescing: only the newest pending record is kept. It contains every older transition, so committing it commits them all. Each superseded record is counted.

### 3.2 Journal format (64-byte slot)

| Offset | Field |
|---|---|
| 0 | magic `PGJ1` |
| 4 | layout version (1) |
| 5 | record type (1) |
| 6 | payload length (32) |
| 8 | sequence (generation) |
| 12 | boot count |
| 16 | payload (the 32-byte `EmergencyPersistRecord` v2, with its own CRC) |
| 48 | writer uptime |
| 56 | CRC-32 over bytes 0..55 |
| 60 | **commit word `0x3CC3A55A`, programmed last** |

The CRC detects accidental corruption only. It is **not authentication**.

### 3.3 Power-loss argument

```
 slot:  erased ──program body──► body ──read back──► body ✔ ──program commit──► committed ──read back──► done
 cut here:     garbage (CRC fails)        (unchanged)       partially-1 commit word ≠ 0x3CC3A55A
```

- **NOR flash:** a program operation only clears bits (1→0), so an interrupted program leaves an unpredictable mix of old 1 bits and new 0 bits. A slot counts only if the magic, version, CRC and the exact commit word are all correct.
- **Every cut point tested:** the test cut power at each of the 65 byte positions of a commit. 64 cases kept the previous record and 1 committed the new one. None produced garbage or the half-written record.
- **Body complete, commit missing:** a cut between the body and the commit word leaves an **intact, uncommitted** slot. It is never used as state; it is used only to keep incident ids monotonic (§5).
- **Rotation:** two sectors form a ring. The newest committed record is always in the active sector, so erasing the other sector can never destroy the only valid copy. A cut during the erase leaves garbage that mount ignores; the sector is erased again before it is used.
- **Sequence numbers:** a new slot's sequence is higher than any intact slot ever seen, committed or not. A slot whose write failed is "burned" and its sequence is never reused.
- **Pre-erase:** `maintain()` erases the next sector in idle time, once the active sector is half full, so a commit does not usually pay for an erase.
- **Exhaustion:** the journal refuses new commits beyond sequence `0xFFFFFF00` (reported) rather than wrapping. This is not reachable in practice.
- **Wear:** each sector is erased once per 64 commits. At the datasheet minimum of 100k erase cycles, that is about 12.8 million commits.

### 3.4 Durability during operation
- The manager persists on **transitions**, not periodically: incident opened, request created, transport accepted, phone stages, cancellation, re-arm, and one boot record.
- The storage task commits a queued record within one storage unit, typically 2 page programs, about 2 ms on the fake flash. The worst case waits behind one sector erase (400 ms, datasheet maximum).
- A failed commit is retried 3 times, each on a new slot. After that the ticket is reported failed, the status shows `PersistenceDegraded`, and the commit is retried with backoff from 200 ms up to 10 s for as long as it fails. A later successful commit clears the condition.

### 3.5 Storage scheduling (`StorageService::step`)
Strict priority, one bounded unit per step:
1. emergency commit
2. log remount (only while the flash is unavailable)
3. critical log (flushed immediately)
4. telemetry log (flushed when a page fills, or every 1 s)
5. journal maintenance

The emergency record is checked before **every** unit, so a telemetry flood can delay it by at most one unit. The test `storage_service_emergency_commit_preempts_telemetry_flood` shows this.

## 4. Flight logging

### 4.1 Records
Each record is `sync A5 | type | len | schema | seq u32 | t_ms u32 | payload | CRC-32`, which is 16 + payload bytes.
- Payloads are written field by field in little-endian, never copied as structs, so there are no padding bytes.
- Invalid values are stored as `INT32_MIN`.
- A golden vector, `Tests/vectors/pglog_v1_records.bin`, is checked by both the C++ tests and the Python decoder tests.

### 4.2 Profiles (`FlightRecorder`)

| Profile | Navigation | Sensor quality | Health | Events / crash / emergency / power |
|---|---|---|---|---|
| Minimal | – | – | 60 s | on change |
| Normal (default) | 1 Hz | 0.2 Hz | 10 s | on change |
| Detailed | 10 Hz | 1 Hz | 5 s | on change |
| Emergency (automatic while an incident is open) | 5 Hz | 1 Hz | 5 s | on change |

- **Not logged:** raw samples. Three IMUs at 416 Hz plus barometers would be about 20 KB/s, which would fill the 16 MB partition in about 14 minutes and compete with the SPI bus.
- **Critical queue:** emergency transitions, SOS stages, Warning-or-worse safety events, flight-phase changes, crash suspicion, power state changes, boot and storage faults.

### 4.3 Integrity, recovery and rotation
- Every sector starts with a 16-byte header: `PGLS`, version, region tag, sector sequence and CRC.
- Records never cross a 256-byte page.
- `flush()` programs only the bytes added since the last flush. NOR flash allows programming the still-erased tail of a page.
- **Mount:** the head is the sector with the highest valid header sequence. Within it, each page is parsed until the first erased byte or the first invalid frame. A torn page is abandoned; writing resumes on the next clean page, with a sequence above everything recovered.
- **Rotation:** `OverwriteOldest` is the default. `StopWhenFull` exists and is tested. A sector whose erase was interrupted has no valid header and is erased again when the ring reaches it.
- **Three failure classes (each tested):**
  - recent committed records are recovered;
  - only the record being written at the cut is lost;
  - with the flash missing, mounts report `Unavailable`, nothing blocks, and every save is `Rejected`.

### 4.4 Partition plan (W25Q128JV, 16 MiB)
I checked first: no firmware code writes the external flash, so nothing on the chip is in use.

| Region | Address | Size | Use |
|---|---|---|---|
| Reserved | 0x000000 | 64 KiB | future provisioning (device identity / keys); untouched |
| Emergency journal | 0x010000 | 8 KiB (2 sectors) | incident state + identity |
| Critical log | 0x012000 | 56 KiB | ~1,600 critical records, ring |
| Flight log | 0x020000 | 15.9 MiB (4,064 sectors) | telemetry ring |

The partitions are separate regions checked with `static_assert`, so ordinary logs can never overwrite the emergency journal.

### 4.5 Storage budget (record sizes: navigation 37 B, quality 28 B, health 40 B; about 7% lost to page padding)

| Profile | Average rate | Flight-log retention |
|---|---|---|
| Minimal | < 2 B/s + events | months |
| Normal | ≈ 50 B/s | ≈ 92 h (3.8 days of flying) |
| Emergency | ≈ 235 B/s | ≈ 20 h |
| Detailed | ≈ 435 B/s | ≈ 10.6 h |

These are calculated from the record sizes, not measured on hardware.

## 5. Incident identity across resets

- **Rule:** an incident id is visible outside the device once its SOS request leaves through the transport. Before the **first** transmission of an incident, the manager waits until a record with `nextIncidentId > incidentId` is **committed**. That wait is bounded by `identityHoldMs` (2 s). If the store reports `Failed`, or the hold runs out, the request is sent anyway, marked `IncidentIdVolatile`, with the status flag `IdentityUnconfirmed`. **Help is never withheld because the flash is slow or broken.**
- **Boot record:** `restore()` sets the next id to the highest of these, then commits a boot record immediately:
  - the committed `nextIncidentId`;
  - the committed `incidentId + 1`;
  - the same two values taken from the intact-uncommitted body.
- **Reset before commit:** if a reset happens before an incident's record is committed, the incident never left the device, so reusing its id cannot cause a wrong de-duplication. The test `persist_incidents_across_many_resets_never_share_an_identity` runs 25 boots, some reset before commit: 16 SOS requests were sent and all 16 had distinct ids.
- **Corrupt journal:** if the store mounts as `Corrupt` (data present but nothing valid), the identity flag `Uncertain` is set and persisted. It stays set, and every later request carries `IncidentIdVolatile`.
- **Blank flash:** a blank chip legitimately starts at id 1. A wiped chip **cannot** be told apart from a new one. That case is documented, and re-provisioning a fresh device id is the remedy (bring-up §9).
- **Exhaustion:** when the 32-bit id space runs out, the `Exhausted` flag is persisted, the counter wraps to 1, and every request is marked volatile. The identity hold is skipped in this state, because waiting cannot help.
- **What volatile means on the phone:** for a volatile id, Android sends the server key `<id>-v<first-received-ms>`, so a repeated MCU id after a reset is a separate server incident. The local SMS/simulation path still de-duplicates on (device, id); this is a known limitation (validation report §9).

## 6. Recovery semantics (`EmergencyManager::restore`)

| State at the reset | After restore |
|---|---|
| Countdown | The MCU clock does not run while powered off, so elapsed time is **unknown**. The full countdown restarts so the pilot can cancel. After `maxCountdownRestarts` (2) resets of the same countdown, the request is created immediately (`RepeatedResetEscalated`), so a brown-out loop cannot postpone help forever. |
| SosPending / AwaitingPhoneAck / AwaitingDelivery / DeliveryUnknown | Same incident id, next revision, flagged `Restored`, re-sent. The phone and the server de-duplicate on the id. |
| Cancelled with a notice pending or sent | The cancel notice is sent again. |
| Delivered / DeliveryFailed | Kept until `ResetIncident`. Nothing is sent again. |
| Corrupt / torn record | Not trusted as state; ids stay monotonic. |
| No store / flash missing | `PersistenceUnavailable`, `IncidentIdVolatile`. The SOS is not delayed. |

## 7. Automatic SOS delivery

```
 MCU ──BLE──► Android EmergencyEngine ──► outbox row (SQLite, before any network call)
                                           │ RemoteDelivery.runOnce (worker loop, JobScheduler job after reboot / network return)
                                           ▼
                  POST /v1/incidents (HMAC-signed, TLS) ──► Backend: incidents + jobs (UNIQUE per incident/contact/purpose)
                  GET /v1/incidents/<key> (poll 15 s)  ◄── worker: lease ─► provider ─► provider_accepted / failed / unknown
                                                             provider status callback (signature-verified, de-duplicated) ─► delivered
```

**Stages, kept separate end to end:**

| Stage | Meaning |
|---|---|
| `SUBMISSION_PENDING` | queued on the phone; no internet or the server is busy |
| `SERVER_ACCEPTED` | the server stored it; sending |
| `PROVIDER_ACCEPTED` | the provider API accepted it; **not delivery** |
| `DELIVERED` | a provider delivery report; a carrier report, not proof that a human read it |
| `FAILED` | delivery failed |
| `UNKNOWN` | the outcome cannot be determined |

**No duplicate messages (backend):**
- Re-submitting the same or an older revision changes nothing.
- Jobs are unique per incident, contact and purpose.
- A job is claimed with a lease. If the worker dies mid-send, the lease expires and the job becomes `unknown`; it is **not re-sent**.
- A provider timeout is `unknown`. Only definite "nothing was sent" answers are retried, with backoff, at most 5 times: HTTP 429, 5xx, or connection refused.
- `retry-unknown` exists as an explicit human decision.
- Callbacks are de-duplicated and never downgrade a delivery.
- **Exactly-once delivery is impossible with SMS providers.** The design gives at most one send per job unless a person asks for a resend, and it reports every uncertain case.

**Providers:**
- `FakeProvider`: deterministic and offline.
- `DisabledProvider`: the default when Twilio is selected without the explicit `PG_LIVE_DELIVERY_ENABLED=true` switch and credentials. It refuses everything.
- `TwilioProvider`: real request construction, Basic authentication, form encoding, status callback, timeout and error mapping, plus webhook signature verification. **Never contacted by any test.**
- Incidents marked `test` (every simulation incident) are delivered only through the fake provider.

**Security:**
- **Request signing:** per-device HMAC-SHA256 over method, path, timestamp, nonce and the body hash.
- **Replay protection:** ±300 s clock window, and each nonce is stored and accepted once.
- **Authorization:** the device id alone authorizes nothing, and every authentication failure gets the same answer.
- **Limits:** 4 KiB body limit; strict schema validation with unknown fields rejected; 30 requests/min and 6 new incidents/h per device.
- **Recipients:** resolved on the server from consented contacts (at most 5, E.164); an incident cannot name recipients.
- **Data minimisation:** logs contain no numbers or coordinates, and positions are purged after 30 days.
- **TLS:** required for any non-loopback bind. Android accepts plain `http` only in debug builds, only for loopback / emulator host addresses, enforced both by the network security config and by `BackendConfig.valid`.
- **Secret at rest:** encrypted with an Android Keystore AES-GCM key; backup is disabled.
- **Known limitation:** the server must store the HMAC secret to verify signatures (production: KMS / secrets manager).

## 8. Android lifecycle

| Situation | What happens | Verified |
|---|---|---|
| Process death while monitoring | `START_STICKY`; Android re-creates the service with a null intent, and it resumes from the persisted request | code + reasoning; not observed on a device |
| Task removed (swipe from Recents) | The foreground service continues (`onTaskRemoved`) | code |
| Phone reboot | `BootReceiver` starts the connectedDevice foreground service. This start type is allowed from BOOT_COMPLETED; the Android 15 boot restriction covers other types. The outbox job is persisted (`setPersisted`). | code; not observed |
| Device comes into range (Android 12+) | `PresenceService` (CompanionDeviceService); observation is started when monitoring is requested | code; needs an associated real device |
| Android refuses a background start | `recoveryPending` is set, a notification asks the user to tap Resume, and the UI shows "Monitoring interrupted" | code + UI |
| User force-stop | **Not recoverable.** Android delivers no broadcasts until the user opens the app; the UI shows "interrupted" on the next launch | stated, not claimed |
| Bluetooth off / on | Existing link backoff; the UI shows "Bluetooth is off" | M7 + new state |
| Permission revoked | The service stops; the request is kept; the UI shows "permission required" | unit test (state function) |

- **Requested vs actual:** the user's request (`monitoringRequested`, persisted) is kept separate from the actual state (`ViewState.monitoring`). The single function `monitorStatus()` derives the label shown, so a stale "active" label is impossible.
- **No duplicate sessions:** `Guardian.monitor()` ignores a second start for the same address.
- **Unresolved incidents** stay in SQLite and are visible after any restart.

## 9. Power management
- **Separate dimensions:** the battery level (Unknown / Healthy / Low / Critical), charge state (Unknown / NotCharging / Charging / Complete / Fault) and supply state (Unknown / Disconnected / Present) are independent.
- **Level changes:** confirmed over 3 samples, with 50 mV hysteresis on the way back up.
- **While charging:** upward level changes are not taken from a voltage the charger is lifting.
- **Defaults:** the thresholds default to "not configured", in which case the level is always Unknown and never Healthy. State of charge is reported only from a fuel gauge.
- **Rails:** `PowerRail` handles the safe default at start, settle time, off-delay against toggling, power-good fault detection and retry. The amplifier is enabled only while audio is requested, plus a 1.5 s hold, and only on a ready 5 V rail. It is forced off if the rail drops.
- **LiDAR policy:** the LiDAR stays on by default. It may be shed only on Critical battery, on the ground, with no open emergency, and only if explicitly configured. An open emergency keeps the rail up and allows all audio.
- **Firmware today:** both switches are `UnconfiguredPowerSwitch` and the inputs are Unknown, so the status says so.

## 10. Audio and LED
- **Alert priorities:** the M6 table is unchanged. Added: `LowBattery` (45 Low / 70 Critical), `PhoneDisconnected` (30, in flight, after 10 s, only while no incident is open) and `SystemFault` (22: persistence degraded or a supervisor stall). An SOS countdown still outranks everything (tested with critical battery during an SOS).
- **`PcmToneOutput`:**
  - square-wave tones at a fixed frequency per tone (measured 2000 Hz for Critical);
  - 2 ms attack and release ramps;
  - exact durations in samples;
  - volume capped at 12000/32767;
  - pre-emption with release;
  - **drops** a tone if the amplifier is not ready within 250 ms or is lost mid-tone. `tonesAudible` counts only tones rendered into a ready amplifier.
- **I2S/SAI:** the peripheral, pins and DMA are unknown, so the firmware keeps `UnconfiguredAudioAlert`. A 256-frame double buffer (1 KiB at 16 kHz mono) is the planned DMA size; it is not allocated yet.
- **`GpioStatusLed`:** reproduces `LedPatternScheduler::levelAt` exactly (tested over 4 s) and writes only on edges. A bug found by its test (the first "on" period was never written) was fixed.

## 11. IMU impact capture
- **Finding:** the IMUs run at **416 Hz** (`Lsm6dsrxConfig`) but are polled at **100 Hz**, so about 3 of every 4 samples are never read, and a short impact peak can be missed.
- **Implemented and tested:** FIFO continuous-mode configuration, decoding of the tagged 7-byte words, and a bounded drain (16 words per call) that returns the peak |a| and |ω|, saturation, overflow, unknown tags and partial reads. In the test, a 9.8 g spike that a poll would miss is captured.
- **Not enabled:** register addresses must be confirmed against the datasheet revision during bring-up, and `ImuSource` does not call it yet. The polling path is unchanged.

## 12. Supervision and watchdog policy
- **What `TaskSupervisor` watches:** the existing progress counters of acquisition, processing, emergency (critical, 2 s timeouts) and storage (5 s), sampled by the health task at 1 Hz.
- **Storage stall:** gives `Degrade`. A critical stall gives `RequestReset`.
- **Reset deferral:** while an incident is open and its newest state is not committed, the reset is **deferred** for up to 5 s so the journal can commit, and then requested anyway.
- **Hardware watchdog:** the IWDG is **not enabled**. `feedWatchdog()` is the policy answer the bring-up will connect.

## 13. RTOS integration and concurrency
- **Shared objects added:**
  - `JournaledEmergencyStore`: critical sections copy at most 32 bytes.
  - `LogQueue`: copies one 56-byte entry under a critical section.
  - `ValueMailbox` instances for the storage and power status.
  - `FlightRecorder::emergencyActive_`: a single byte written by one task.
- **No locks around flash I/O.** No new lock ordering exists: each critical section is a leaf.
- **Blocking:** the storage task blocks only in the W25Q driver's busy polling, which is bounded by the datasheet timeouts. It sleeps on a notification or a 50 ms idle period otherwise.
- **Emergency task wait:** it waits at most 2 s at boot for the journal mount (`STORAGE_MOUNT_WAIT_MS`).
- **No heap:** none of the new firmware objects references `malloc`, `new` or `free` (checked with `nm -u`).
- **ThreadSanitizer** covers the host-side concurrency tests only. It cannot see ISR interactions or the real scheduler.

## 14. Failure handling summary

| Failure | Handling |
|---|---|
| Torn program | Detected by CRC and the commit word; the previous record is used |
| Torn erase | Sector rejected and erased again |
| Program timeout | Slot burned, retried, `Failed` reported after 3 attempts, retried with backoff |
| Flash absent | `Unavailable`, `Rejected`, SOS unaffected |
| Log queue full | New entry dropped and counted per queue (critical and telemetry kept separate) |
| Amplifier failure | `RailState::Fault`; tones dropped and counted, never reported as audible |
| Battery data missing | `Unknown` + `StaleMeasurement`, never Healthy |
| Phone or server offline | `SUBMISSION_PENDING`, retried with backoff up to 5 min |
| Server timeout | Resubmitted (idempotent) and reconciled by polling |
| Provider timeout | `unknown`; no automatic resend |
| Worker crash | Lease expiry gives `unknown` |
| Duplicate callback | Ignored |

## 15. Remaining hardware dependencies
- W25Q128 SPI instance and chip select
- Battery divider and ADC channel, battery chemistry and thresholds
- Charger status pins and their polarity
- 5 V regulator and amplifier enable pins and polarity, plus settle times
- SAI/I2S instance, pins, DMA and clock
- LED pins
- CYW20822 provisioning and authenticated association
- LSM6DSRX FIFO register confirmation and the INT pins
- IWDG configuration
- On-target stack and timing measurements

All of these are listed in [MILESTONE8_HARDWARE_BRINGUP.md](MILESTONE8_HARDWARE_BRINGUP.md).
