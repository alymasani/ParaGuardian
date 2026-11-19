# RTOS Timing Analysis

Date: 2026-11-17 · Status: **analysis + host simulation only; no hardware measurements exist.**
All microsecond figures below are estimates from data sizes and clock rates, to be replaced by
measurements (see §9). Millisecond "wait" figures come from the code and the host tests.

## 1. Tasks and priorities

| Task | Priority (FreeRTOS number) | Period / trigger | Stack | Created by |
|---|---|---|---|---|
| SensorAcq | 40 (`osPriorityHigh`) | 10 ms, `xTaskDelayUntil` | 1024 words (static) | `paraguardian_create_tasks` |
| EmergencyAlert | 36 (`osPriorityAboveNormal4`) | every 20 ms (`ulTaskNotifyTake` timeout) or at once on a command / transport report (task notification); **crash -> emergency manager -> alert dispatcher (Milestone 6)** | 768 words (static) | `paraguardian_create_tasks` (CRASH_EMERGENCY_DESIGN.md) |
| SensorProc | 32 (`osPriorityAboveNormal`) | on data (`popWait`, 50 ms idle timeout); ~1 frame per acquisition cycle; **one Madgwick attitude update (Milestone 3), one vertical-navigation UKF update (Milestone 4), one flight / landing safety update (Milestone 5) and one crash-detector update (Milestone 6) per frame** | 1024 words (static) | `paraguardian_create_tasks` (Milestone 2, see SENSOR_PREPROCESSING_DESIGN.md; attitude: MADGWICK_ATTITUDE_DESIGN.md; navigation: VERTICAL_UKF_DESIGN.md; safety: FLIGHT_LANDING_DESIGN.md; crash: CRASH_EMERGENCY_DESIGN.md) |
| defaultTask | 24 (`osPriorityNormal`) | exits immediately (`osThreadExit`) | 128 words (heap) | CubeMX |
| Health | 16 (`osPriorityBelowNormal`) | 1 s or on command | 512 words (static) | `paraguardian_create_tasks` |
| Timer service | 2 | unused (no software timers) | 256 words (static, CMSIS) | kernel |
| Idle | 0 | — | 128 words (static, CMSIS) | kernel |

Interrupts: TIM6 HAL tick (15), SysTick/PendSV (15). UART RX ISRs (future) must be ≥ 5 if they ever call
`FromISR` APIs; the current design's ISRs only touch lock-free ring buffers.

## 2. One acquisition cycle (steady state)

`AcquisitionScheduler::runCycle()` services every source once, in rotating order:

| Source | Work per due read | Default period | Bus bytes | Est. time @ SPI 8 MHz* | Est. @ I2C 400 kHz* |
|---|---|---|---|---|---|
| IMU ×3 | STATUS read + 14-byte burst | 10 ms | 2 + 15 | ~60–100 µs each | ~0.5 ms each |
| Baro ×2 | 6-byte burst | 20 ms | 7 | ~40 µs each | ~0.2 ms each |
| Mag (two-phase) | trigger: 2 writes; collect: STATUS + 7-byte burst | 20 ms | 4 / 10 | ~50 µs | ~0.3 ms |
| GNSS | drain ≤ 512 B + UBX parse | every cycle | — | ≤ ~0.4 ms (512 B × ~50 cycles @ 64 MHz) | — |
| LiDAR | drain ~115 B + parse | every cycle | — | ~0.1 ms | — |
| Hub publish | per message: copy 88 B to each queue + mailbox (critical section) | per sample | — | ~2–5 µs per message | — |

\*Includes an assumed ~10–20 µs of HAL call overhead per blocking HAL transfer at 64 MHz.

**Estimated steady-state cycle: ≈ 0.5–1.5 ms with SPI sensors, ≈ 2–3 ms if the IMUs are on 400 kHz I2C**,
against a 10 ms period. Host simulation (`audit_b_full_sensor_set_normal_cycle_has_no_waits`): with all
8 sources, steady-state cycles contain **0 ms of waiting**, no overruns, the LiDAR buffer is empty after
every cycle, and the expected sample counts are produced (3 IMUs × ~100/s, 2 baros × ~50/s, mag ~50/s).

CPU load of SensorAcq ≈ 5–15 % (estimate). Health: < 1 ms per second.

## 3. Blocking operations and their bounds

| Operation | Where | Bound | Notes |
|---|---|---|---|
| SPI transfer | `HalSpiDevice` | `timeoutMs` per HAL call (≤ 2 calls) + bus-mutex wait ≤ `timeoutMs` | SPI never waits for an absent device |
| I2C transfer | `HalI2cBus` | **25 ms** HAL busy-flag wait + `timeoutMs` per call | stuck SDA = worst case; prefer SPI for IMUs |
| UART transmit | `HalUartPort::write` | `txTimeoutMs` | not used in the acquisition path |
| LSM6DSRX init | reset poll | 50 ms (`kResetTimeoutMs`) | normally ~2 ms (one 1-ms poll + tick rounding) |
| BMP585 init | soft-reset wait | 2 ms (+1 tick) | |
| MMC5983MA init | SW reset + SET | 10 + 1 ms (+ticks) ≈ 12–13 ms | the longest *normal* init |
| MMC5983MA read | two-phase | **0 ms wait** per cycle; conversion timeout 18 ms checked per poll | was 8–18 ms blocking before hardening |
| W25Q128 / BLE | not in acquisition task | — | only interfere via a shared SPI mutex |
| Health collect | Health task | short; reads mailbox under critical sections (8 × 88 B copies) | |

No call in the acquisition path can wait without bound.

## 4. Worst-case cycle times (bounded)

| Scenario | Before hardening | After hardening |
|---|---|---|
| Normal steady state | mag read: ~8–9 ms every 20 ms → frequent overruns | ~1 ms (estimate), 0 ms waiting (simulated) |
| Start-up, all sensors present | all inits in cycle 1: ~3×2 + 2×3 + 13 ≈ 25 ms | one init per cycle; worst cycle ≈ 13 ms (MMC init) → one missed deadline per start-up, spread over 6 cycles (simulated ≤ 13 ms) |
| Absent magnetometer | 10+ ms stall every 1 s forever | one 10+ ms stall at 1, 2, 4, 8, 8, … s |
| 3 IMUs with hung reset | ~150 ms cycle every 1 s | ≤ 53 ms in one cycle; the other two deferred (simulated) |
| I2C bus stuck (if I2C used) | 25 ms + timeout per transaction, every read | same per transaction; sensor re-inits after 10 failures, then backs off (1–8 s) |
| Sensor dies after init | failing reads forever | 10 failing reads, then re-init with backoff |

After any overrun the loop re-anchors (no catch-up burst), so one long cycle costs exactly the missed
deadlines it spans.

### 4a. Attitude estimation inside SensorProc (Milestone 3)

| Item | Value | Basis |
|---|---|---|
| Work per frame | one `AttitudeEstimator::update()`: ~400 FLOPs, ~10 `sqrtf`, ~6 divisions, 1 `acosf`, 1 `asinf`, 2–4 `atan2f` | code inspection (MADGWICK_ATTITUDE_DESIGN.md §11) |
| Host time | 76 ns (6-axis) / 86 ns (9-axis) per update, Apple M4, `-O2` | measured (benchmark in MADGWICK_VALIDATION_REPORT.md §10) |
| Target time | **estimate** 25–80 µs per update at 64 MHz, caches off | cycle estimate; NOT measured |
| Share of the 10 ms period | ≤ 0.8 % (estimate) | — |
| Stack added on the SensorProc path | ≤ ~0.5 KB (Release `-fstack-usage`), sequential to the preprocessing path | measured from `.su` files |
| Blocking | none (mailbox copy of ~0.2 KB under a critical section) | code |

The attitude update runs after `buildFrame()` in the same wake, so it can never delay SensorAcq
(priority 40). It does not run more often than frames are built; a frame without a newer gyro
timestamp is not integrated.

### 4b. Vertical navigation (UKF) inside SensorProc (Milestone 4)

| Item | Value | Basis |
|---|---|---|
| Work per frame | one `VerticalNavigationEstimator::update()`: ≤ 3 sample updates (barometer, acceleration, GNSS), each = a 7-sigma-point prediction + a 7-point evaluation + a 3×3 update; typically acceleration every frame and barometer every second frame | code (VERTICAL_UKF_DESIGN.md §12) |
| Worst case per frame | after a 3 s gap: up to 30 prediction steps of 100 ms, then `Unavailable` (no unbounded loop) | `maxPredictionMs` / `maxPredictStepMs` |
| Host time | 152 ns per update (Apple M4, `-O2`, 60 000 frames with GNSS) | measured (VERTICAL_UKF_VALIDATION_REPORT.md §12) |
| Target time | **estimate** 50–150 µs per update at 64 MHz | host × Milestone 3 ratio; NOT measured |
| Share of the 10 ms period | ≤ 1.5 % (estimate); attitude + navigation together ≤ ~2.3 % | — |
| Stack, deepest chain | `Run` 256 B (was 208) + navigation chain 672 B = 928 B of 4 096 B | Release `.su`, measured |
| Static RAM | +1 608 B in the SensorProcessingTask object (estimator 1 264, mailbox 320, model 12) | `nm -S`, measured |
| Blocking | none; mailbox copy of 312 B under `taskENTER_CRITICAL` (a few µs, estimate) | code |

Navigation runs after the attitude update, in the same wake, with no new task, queue or priority
change. Sampling rates and queue overflow policies are unchanged. Each barometer, acceleration and
GNSS sample is assimilated once at its own timestamp, so idle-timeout or republished frames cost only
the classification (no filter step).

### 4c. Flight / landing safety inside SensorProc (Milestone 5)

| Item | Value | Basis |
|---|---|---|
| Work per frame | `SafetyMonitor::update()`: LiDAR projection (one quaternion rotation), <= 64-sample line fit when a new LiDAR sample arrives, flight-phase evidence, 6 warning trackers; bounded, no loops over unbounded data | code (FLIGHT_LANDING_DESIGN.md section 9) |
| Host time | 126 ns per update (Apple M4, `-O2`, 60 000 frames, LiDAR valid most frames) | measured (FLIGHT_LANDING_VALIDATION_REPORT.md section 10) |
| Target time | **estimate** 40-130 us per update at 64 MHz | host x Milestone 3 ratio; NOT measured |
| Share of the 10 ms period | <= 1.3 % (estimate); attitude + navigation + safety together <= ~3.6 % | - |
| Stack | `Run` 296 B (was 256); safety chain ~0.4 KB after navigation returns; deepest chain (navigation) ~968 B of 4 096 B | Release `.su`, measured |
| Static RAM | +2 808 B in the SensorProcessingTask object (SafetyMonitor 1 896, SafetyEventQueue<16> 680, snapshot mailbox 196, locks) | `nm -S`, measured |
| Blocking | none; snapshot copy 188 B and event copy 40 B under `taskENTER_CRITICAL` (a few us, estimate) | code |

The safety update runs after navigation in the same wake (no new task, queue or priority change) and
uses THIS frame's attitude and navigation. Events are produced only on state transitions (a few per
flight), so the 16-slot queue is not a throughput concern; it keeps the most severe events when full
(overflow counters in `g_systemHealth`). Since Milestone 6 it is consumed by EmergencyAlertTask (4d).

### 4d. Crash detection (SensorProc) and the EmergencyAlert task (Milestone 6)

| Item | Value | Basis |
|---|---|---|
| Crash detector work per frame | per-IMU peak corroboration (3 values), one 250 ms history slot update, <= 32-slot scan only when an impulse starts, a few timers; bounded | code (CRASH_EMERGENCY_DESIGN.md section 6) |
| Crash detector host time | 26-29 ns per update (Apple M4, `-O1`, 30 000 updates incl. impacts) | measured (CRASH_EMERGENCY_VALIDATION_REPORT.md section 9) |
| Crash detector target time | **estimate** 10-30 us per update at 64 MHz (host x the Milestone 3 ratio) | NOT measured |
| SensorProc stack | `Run` 312 B (was 296); crash chain (`update` 104 + `startCandidate` 56 + event emit 104 + queue publish ~40) ~0.6 KB after the safety chain returns; deepest chain still navigation ~0.98 KB of 4 096 B | Release `.su`, measured |
| SensorProc static RAM | +2 096 B (CrashDetector 1 560, CrashReport 164, IncidentContext 52, report mailbox, lock) | `nm -S`, measured |
| EmergencyAlert period / wake | 20 ms timeout, or immediately on `submitCommand` / `submitTransportReport` (notification) | code |
| EmergencyAlert work per wake | <= 8 commands, <= 8 transport reports, 1 crash-report copy (164 B), 1 manager tick, <= 16 + 8 events, 1 snapshot copy (188 B), 1 dispatcher update, 3 status publishes | code |
| EmergencyAlert host time | manager tick 16-19 ns, dispatcher update 28-33 ns (+ copies) | measured (host) |
| EmergencyAlert target time | **estimate** < 0.1 ms per wake at 64 MHz (< 0.5 % CPU at 50 Hz) | NOT measured |
| EmergencyAlert stack | `Run` 184 B Release / 232 B Debug; deepest chain (command -> expiry -> request -> event -> queue) ~0.45 KB Release, ~0.55 KB Debug of 3 072 B | `.su`, measured |
| EmergencyAlert static RAM | task object 5 484 B (3 072 B stack, manager 480, dispatcher 456, inboxes 264, local event queue 360, mailboxes, copies) | `nm -S`, measured |
| Blocking | none: transport `submit` and audio `request` only copy; persistence is `NullEmergencyStore` (a flash backend must be asynchronous) | code |
| Priority reasoning | above SensorProc so a processing overload cannot starve a countdown / cancel / retry; below SensorAcq so sampling is never delayed. Preemption of SensorProc: < 0.1 ms per 20 ms (estimate) | TaskConfig.hpp static_asserts |
| Latencies | command -> applied: next wake (notification: immediate); processing event -> dispatched: <= 20 ms; countdown resolution 20 ms (1 ms with 1 ms ticks in the host test); a countdown does NOT depend on sensor frames | code + tests |

Synchronisation: the processing queue has ONE consumer (this task) and one producer (SensorProc); the
crash report, safety snapshot, emergency status, SOS record and dispatch status are latest-value
mailboxes (one writer each) copied under `taskENTER_CRITICAL` (<= 188 B, a few us); command and report
inboxes are locked bounded FIFOs (drop-new, counted). No task blocks on another.

## 5. Deadline analysis

- Period 10 ms; with SPI sensors the steady-state utilisation is far below 1 (estimate ≤ 15 %).
- Only higher-priority work can preempt SensorAcq: interrupts (TIM6/SysTick/PendSV at 15, future UART
  ISRs). No task has a higher priority.
- Expected deadline misses: only during start-up inits (one ~13 ms cycle) and while a sensor is failing
  (bounded, rate-limited by backoff).
- Observability: `missedDeadlines`, `cycleOverruns`, `maxCycleTimeMs`, `lastCycleTimeMs`,
  `initAttemptsDeferred` in `g_systemHealth` (1 ms resolution; sub-ms cycles read as 0 ms).

## 6. Initialisation timing (boot)

| Phase | Time | Tick advancing? |
|---|---|---|
| Static init (SoarOS mutex) | µs | n/a |
| `HAL_Init` + `SystemClock_Config` (HSI 64 MHz, no PLL) | < 1 ms expected | yes, via `HalTickFallback` polling (was frozen) |
| `osKernelInitialize` → tasks created → `osKernelStart` | < 1 ms | yes (polling) |
| First SensorAcq cycles: one sensor init per cycle | ~6 cycles ≈ 60–70 ms to bring up 6 polled sensors | interrupt-driven |
| GNSS first fix | seconds to minutes (receiver dependent) | — |

## 7. Queues, occupancy and loss

| Store | Size | Policy | Memory |
|---|---|---|---|
| SensorAcq latest mailbox | 8 slots × 88 B | overwrite per sensor | 776 B |
| Future navigation queue (`RtosSampleQueue<32>`) | 32 × 88 B | DropOldest | ~2.9 KB |
| Future logging queue | sized to flash write rate | DropNewest | 88 B per slot |
| UART RX ring (per port) | 1024 B | drop newest byte | 1 KB |

Production rate (all sensors): ~300 IMU + 100 baro + 50 mag + ≤ 25 GNSS + 100 LiDAR ≈ **575 msgs/s**.
A consumer must drain ≥ 575 msgs/s on average; a 32-deep queue covers a ~55 ms consumer stall before
the policy acts. Loss is counted per subscriber (`dropsFor`, `overwritesFor`) and visible to consumers
as sequence gaps (`SequenceTracker`).

## 8. Failure scenarios and detection times

| Scenario | Detection | Recovery | Test |
|---|---|---|---|
| IMU unresponsive (floating MISO) | `isHealthy()` false after 3 reads = **30 ms** | re-init after 10 reads (100 ms), backoff 1→8 s | `audit_b_dead_sensor_after_init…`, `…detection_time…` |
| IMU absent at boot | first init attempt (≤ 1 cycle) | retries 1, 2, 4, 8, 8 … s | `audit_b_two_barometers…` (same logic) |
| Two barometers fail together | first cycles | staggered, neither starved | `audit_b_two_barometers_fail_together…` |
| Several devices initialising at once | — | one per cycle | `audit_b_simultaneous_init_failures…` |
| I2C transaction repeatedly times out | per read; overrun counted | re-init after 10, backoff | `audit_b_overrun_is_observable` (model) |
| GNSS buffer fills during heavy activity | `rxOverflows()` | 512 B drained/cycle → 1500 B backlog cleared in 3 cycles | `audit_b_gnss_burst…` |
| LiDAR faster than consumer | `dropsFor/overwritesFor` | per policy | `audit_c_*` |
| Magnetometer conversion hangs | 18 ms timeout per measurement | next trigger next period | `audit_b_mag_nonblocking_timeout_detected` |
| GNSS stops sending | 2 s stale timeout | — | existing `gnss_stale_solution_invalidated` |
| LiDAR stops sending | 100 ms stale timeout | — | existing `tf02_stale_timeout` |
| Low-priority task holds SPI mutex | bounded by mutex timeout; counted as Timeout | priority inheritance boosts the holder | not testable on host |

## 9. Recommended hardware measurements

1. Toggle a spare GPIO (once pins exist) around `runCycle()` and measure with a logic analyser:
   steady-state cycle time per sensor set; confirm ≤ 2 ms with SPI.
2. Read `g_systemHealth` after 10 min: `missedDeadlines` (expect ≈ 1 from boot), `cycleOverruns`,
   `maxCycleTimeMs`, stack high-water marks (> 64 words), `minimumEverFreeHeapBytes`.
3. Confirm Audit A: `tickFallbackIncrements > 0` after boot; `uptimeMs` close to wall-clock time.
4. Unplug/replug a sensor: check detection (~30 ms), `reinitCount`, and recovery.
5. Hold SDA low on an I2C bus (if used): measure the stall per transaction.
6. LiDAR + GNSS at full rate during a forced 50 ms stall: `rxOverflows()` must stay 0.
7. Provoke a stack overflow with a test task: confirm reset and `resetFlags` bit 24 (SFTRSTF).
8. Consider DWT cycle timing (with `DWT->LAR` unlock and a non-moving-counter guard) for µs-resolution
   cycle statistics.

### 4e. BLE transport (Milestone 7)

The emergency task services the BLE bridge after its existing command and report inboxes. It observes the most recent crash state, runs the emergency tick and dispatches local alerts before attempting one outbound packet. Its period, priorities and 768-word stack allocation are unchanged. The two sensor tasks make no BLE calls. There is no new RTOS task or heap allocation in the communications path.

The bridge takes at most four incoming fragments and attempts one outbound fragment per wake. There are 112-byte outbound and reassembly buffers, a 118-byte packet scratch buffer and one 96-byte payload scratch field. One critical SOS can displace pending routine status. A driver must use bounded queues and publish Busy on backpressure. No vendor UART or GATT call runs synchronously in this task.

At default ATT the six-packet SOS requires six transmit opportunities. With an idle scheduling interval of 20 ms that is approximately 120 ms including initial wake alignment, before radio, driver or phone delays. With one negotiated packet it is about one 20 ms opportunity. Each non-blocked wake adds one output packet or advances input by at most four packets. A partial frame expires after 2 seconds. M6 still uses the 10-second application acceptance timeout and independent retry policy.

CubeIDE reported the Release task frame as 240 bytes, versus 184 bytes previously. Debug reported 256 versus 232 bytes. Release bridge receive and transmit frames were 64 and 56 bytes individually. These are compiler static frame sizes and do not measure full nested call paths or high-water usage. Release BSS grew by 664 bytes, including the bridge and diagnostics snapshot. The application has no physical module backend, so no measured radio load or Cortex-M7 timing exists. See the [Milestone 7 validation report](BLE_ANDROID_VALIDATION_REPORT.md) for scope and measurement limitations.
