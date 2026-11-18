# RTOS Timing Analysis

Date: 2026-11-17 · Status: **analysis + host simulation only; no hardware measurements exist.**
All microsecond figures below are estimates from data sizes and clock rates, to be replaced by
measurements (see §9). Millisecond "wait" figures come from the code and the host tests.

## 1. Tasks and priorities

| Task | Priority (FreeRTOS number) | Period / trigger | Stack | Created by |
|---|---|---|---|---|
| SensorAcq | 40 (`osPriorityHigh`) | 10 ms, `xTaskDelayUntil` | 1024 words (static) | `paraguardian_create_tasks` |
| SensorProc | 32 (`osPriorityAboveNormal`) | on data (`popWait`, 50 ms idle timeout); ~1 frame per acquisition cycle | 1024 words (static) | `paraguardian_create_tasks` (Milestone 2, see SENSOR_PREPROCESSING_DESIGN.md) |
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
