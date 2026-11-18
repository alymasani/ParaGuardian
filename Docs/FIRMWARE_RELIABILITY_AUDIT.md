# Firmware Reliability Audit

Date: 2026-11-17 · Scope: existing ParaGuardian foundation (no new features) · Hardware: **none available**

AI assistance: this audit, the code changes and these documents were produced with Claude Code (an AI
assistant) at the student's request, and must be reviewed by the team before use.

Status words used below:
**CONFIRMED** (evidence found in code/image; reproduced where possible) ·
**DISPROVEN** · **PARTLY CONFIRMED** · **UNVERIFIED** (needs hardware).

## 0. Starting point (verified, not assumed)

| Claim from the previous report | Re-checked result |
|---|---|
| Host tests 98/98 | **98/98 passed** (ASan+UBSan), re-run before any change |
| Debug / Release 0 errors 0 warnings | **Confirmed** by headless CubeIDE build of a copy: Debug text 25 936 B, Release text 15 680 B |
| SoarOS unmodified at 426c1ef | **Confirmed**: `master`, `426c1efe…`, clean, no stash |
| Workspace unchanged since last session | **Confirmed**: no file newer than the last report; git state recorded in GIT_CHANGE_SUMMARY_HARDENING.md |

---

## Audit A — Start-up and pre-scheduler interrupt masking

**Hypothesis.** SoarOS's global `Global::vaListMutex` is constructed before `main()`; its constructor
enters FreeRTOS critical sections, leaving BASEPRI raised so the TIM6 HAL tick cannot advance until the
scheduler starts; HAL timeouts and `HAL_Delay()` before `osKernelStart()` would then hang.

**Files inspected.** `SoarOS/CubeDefines.cpp`, `SoarOS/Core/Mutex.cpp`,
`Middlewares/.../portable/GCC/ARM_CM4F/port.c` + `portmacro.h`, `queue.c`, `tasks.c`,
`Startup/startup_stm32h743zitx.s`, `Src/main.c`, `Src/stm32h7xx_hal_timebase_tim.c`,
`Drivers/.../stm32h7xx_hal.c`, `stm32h7xx_hal_tim.c`, `Inc/FreeRTOSConfig.h`, `stm32h7xx_hal_conf.h`,
and the linked ELF.

**Evidence.**
1. *When the constructor runs.* `startup_stm32h743zitx.s` lines 99–101: `bl __libc_init_array` then
   `bl main`. The ELF's `.init_array` holds exactly two entries: `frame_dummy` and
   `_GLOBAL__sub_I__ZN6Global11vaListMutexE`; disassembly shows it calls `Mutex::Mutex()` →
   `xQueueCreateMutex`. So the FreeRTOS call happens **before `main()`**.
2. *Critical sections inside it.* `xQueueCreateMutex` → `xQueueGenericCreate` (`pvPortMalloc`, whose
   `xTaskResumeAll` uses `taskENTER_CRITICAL`) and `xQueueGenericReset`/`xQueueGenericSend` (`queue.c`
   line 309 etc.: `taskENTER_CRITICAL`).
3. *Why BASEPRI stays raised.* `port.c` line 152: `static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;`.
   `vPortEnterCritical` raises BASEPRI and increments; `vPortExitCritical` decrements and only calls
   `portENABLE_INTERRUPTS()` when the count reaches **0**, which it cannot from 0xAAAAAAAA. BASEPRI is
   only cleared by `vPortSVCHandler` (`mov r0,#0; msr basepri,r0`) when the first task starts, after
   `xPortStartScheduler` sets `uxCriticalNesting = 0` (line 398). This is deliberate FreeRTOS behaviour.
4. *Is TIM6 affected?* BASEPRI = `configMAX_SYSCALL_INTERRUPT_PRIORITY` = 5 << 4 = 0x50. TIM6 priority
   = `TICK_INT_PRIORITY` = 15 → 0xF0 ≥ 0x50 → **masked**. (Interrupts 0–4 are not masked.)
5. *Does `HAL_GetTick()` advance?*

   | Stage | BASEPRI | TIM6 running | uwTick advances? |
   |---|---|---|---|
   | Reset_Handler → `__libc_init_array` | 0 → 0x50 (during ctor) | no | n/a |
   | `main()`: `HAL_Init()` → `HAL_InitTick` starts TIM6 | 0x50 | yes | **no** (before fix) |
   | `SystemClock_Config()` (HAL_PWREx/RCC timeouts) | 0x50 | yes | **no** (before fix) |
   | `MX_GPIO_Init`, `osKernelInitialize`, `osThreadNew`, `paraguardian_create_tasks` | 0x50 | yes | **no** (before fix) |
   | first task running | 0 | yes | yes |

6. *Who relies on the tick before the scheduler today.* `HAL_PWREx_ConfigSupply`, `HAL_RCC_OscConfig`,
   `HAL_RCC_ClockConfig` (all use `HAL_GetTick()` timeouts). They return normally if the hardware is
   ready; if not, they spin **forever instead of returning `HAL_TIMEOUT` → `Error_Handler`**. In future,
   any `MX_xxx_Init()` with a timeout (e.g. `HAL_UART_Init` → `UART_CheckIdleState`) or a `HAL_Delay()` in
   USER CODE (sensor power-up delays) would hang. No current code calls `HAL_Delay()` before the scheduler.
7. *RtosClock pre-scheduler path (also reviewed).* Two latent defects:
   - `millis()` used `xTaskGetTickCount()`, which is **0 until the scheduler starts**. Every driver timeout
     loop `(millis()-start) >= timeout` would therefore never end if a driver were initialised before
     the scheduler (combined with the busy-wait delay, the delay itself is bounded but the timeout is not).
   - `busyWaitMs()` used the DWT cycle counter. Wrap-around arithmetic and the 64 000 cycles/ms
     calculation (`SystemCoreClock/1000`, updated by `HAL_RCC_ClockConfig`) were correct, but on
     Cortex-M7 parts the DWT can require the Lock Access Register (`DWT->LAR = 0xC5ACCE55`) to be
     unlocked before `CYCCNT` counts without a debugger attached. The code did not unlock it and had no
     "counter not moving" guard, so the wait could be **unbounded** on a board without a debugger. This
     cannot be proven without hardware (**UNVERIFIED** as a fact about this exact chip), but the risk
     was real and removable.

**Verdict.** **CONFIRMED.** The previous report's interpretation was correct; the extra RtosClock
problems were not previously reported.

**Severity / impact.** Medium today (only matters if hardware misbehaves during clock set-up, where the
result is a hang in HAL instead of a hang in `Error_Handler`); **high for future work**: the next
milestone will add peripheral initialisation, and a `HAL_Delay()` placed in `main()` would freeze the
board with no error.

**Change made (smallest safe mitigation, application-side only).**
`Components/SysCore/HalTickFallback.cpp` defines a strong `HAL_GetTick()` (HAL's is `__weak`). When the
TIM6 interrupt **cannot run** because of PRIMASK/BASEPRI (decided by the pure, unit-tested
`pg::interruptBlockedByMask()` in `Components/Common/InterruptMask.hpp`), it polls TIM6's update flag;
if set, it clears it (`TIMx->SR = ~TIM_SR_UIF`, rc_w0 semantics) and calls `HAL_IncTick()`. Otherwise it
just returns `uwTick`.
- No interrupt mask is changed; FreeRTOS's critical-section state is untouched.
- One monotonic time base (`uwTick`) — no jump when the real interrupt takes over.
- No double counting: the interrupt cannot run while masked, and `HAL_TIM_IRQHandler` re-reads `SR`
  (`itflag`) so a pending interrupt whose flag was already cleared does nothing.
- The tick can only run slow (≤ 1 increment per poll), never fast → timeouts are at least as long as
  requested and always finite while polled.
- References `htim6` from the CubeMX time-base file, so a future time-base change fails at link time.
- `pg::g_tickFallbackIncrements` → `g_systemHealth.tickFallbackIncrements` lets hardware bring-up
  **confirm** the analysis (non-zero after boot = masking was real).

`RtosClock::millis()` now returns `HAL_GetTick()` (monotonic from `HAL_Init`, advancing pre-scheduler);
`delayMs()` uses `vTaskDelay` when the scheduler runs, else `HAL_Delay()` (now bounded). The DWT code was
removed.

**Alternatives considered.**
| Option | Why not chosen |
|---|---|
| Clear BASEPRI at the start of `main()` | Changes FreeRTOS's deliberate pre-scheduler mask state; any future pre-scheduler ISR calling a `FromISR` API could request a context switch before the kernel is ready. Rejected (brief: no unsafe mask manipulation). |
| Move/disable SoarOS's global mutex | Requires editing SoarOS (`CubeDefines.cpp`); a global `Mutex` object cannot be lazily constructed without changing its declaration. |
| Excluding `CubeDefines.cpp` and re-implementing it | Duplicates SoarOS internals; still needs a global `Mutex` of the same type. |
| Use SysTick/DWT for HAL time | DWT lock issue above; SysTick belongs to FreeRTOS. |
| Documentation only | Leaves an unbounded wait for the next milestone to trip over. |

**Acceptance criteria.** One `osKernelInitialize()` and one `osKernelStart()` (generated code only;
`grep` finds no other call; regeneration test kept them) ✓ · no unbounded start-up wait from a frozen
time base ✓ · no mask manipulation ✓ · documented ✓ (also RTOS_TIMING_ANALYSIS.md §6).

**Tests.** `audit_a_basepri_masks_tim6_before_scheduler` (masking rule with the measured register
values). The register-level polling itself can only run on the target → **UNVERIFIED on hardware**;
bring-up check: `tickFallbackIncrements > 0` and `uptimeMs` plausible.

**Residual risk.** A `HAL_GetTick()` caller inside a *higher-priority ISR* (priority 0–14 with BASEPRI 0)
still sees a frozen tick (same as stock HAL); HAL timeouts must not be used inside ISRs.

---

## Audit B — Acquisition timing and blocking

**Hypothesis.** A single 10 ms loop servicing all sensors sequentially may miss its deadline.

**Files inspected.** `SensorAcquisitionTask.cpp`, `AcquisitionSources.cpp`, all drivers' `init()`/read
paths, `HalSpiDevice.cpp`, `HalI2cBus.cpp`, `HalUartPort.*`, `stm32h7xx_hal_i2c.c`, `tasks.c`
(`xTaskDelayUntil`).

**Findings.**

| # | Finding | Verdict |
|---|---|---|
| B1 | `MagSource` used the **blocking** `Mmc5983ma::measure()`: trigger, then poll every 1 ms until done (8 ms at 100 Hz bandwidth, timeout 18 ms). Every magnetometer read consumed ~80 % of a 10 ms cycle (100 % with tick rounding). Reproduced: `audit_b_old_blocking_mag_measure_consumes_cycle`. | **CONFIRMED, high** |
| B2 | Init retries ran inside the loop with no limit: several failing sensors retried in the **same cycle** (fixed 1 s interval, same phase), stacking their waits. Worst case reproduced: three IMUs whose reset never completes → one 150 ms cycle (`audit_b_without_budget_init_waits_stack_up`). Even a simply absent magnetometer cost 10 ms every second (power-on wait precedes the ID check). | **CONFIRMED, medium-high** |
| B3 | A sensor that worked and then failed (unplugged, browned out) was read forever and **never re-initialised**. | **CONFIRMED, medium** |
| B4 | After a missed deadline, `xTaskDelayUntil` keeps the stale wake time, so the loop ran **back-to-back catch-up cycles** and counted each as a miss. | **CONFIRMED (by reading `xTaskDelayUntil`)**, low-medium |
| B5 | HAL I2C waits up to **25 ms** (`I2C_TIMEOUT_BUSY`) for a busy bus *in addition to* the caller's timeout, per transaction. A stuck SDA line could cost > 25 ms per read. | **CONFIRMED (HAL source)**; mitigated by observability; recommend SPI or bus-recovery (not implementable without pins) |
| B6 | SPI transactions are bounded by the caller's `timeoutMs` (×2 HAL calls) plus optional mutex wait; a missing SPI device does not block (it reads 0xFF → `InvalidData`/`WrongDeviceId` quickly). | Bounded ✓ |
| B7 | UART RX ring 512 B = ~44 ms of LiDAR traffic at 115 200 baud: overflows during a ~50 ms init stall. | **CONFIRMED**, low |
| B8 | GNSS/LiDAR draining bounded to 512 B/cycle ✓; flash and BLE are not called from the acquisition task ✓ (they can only interfere through a shared SPI bus mutex — bounded by the mutex timeout). | OK |
| B9 | Overrun measurement uses 1 ms resolution: sub-ms cycles report 0. | Limitation (documented) |

**Changes.**
- `Mmc5983ma::startMeasurement()` / `readMeasurementIfReady()` (non-blocking two-phase API; timeout
  now evaluated per poll, a finished conversion read late is still accepted). `MagSource` triggers when
  due and collects on later cycles (`continuePending`). The blocking `measure()` remains for other uses.
- `CycleBudget` + `AcquisitionScheduler` (new portable class): **one driver `init()` per cycle**
  (`TaskConfig::INIT_ATTEMPTS_PER_CYCLE`), with rotating start index for fair tie-breaking.
- `RetryPolicy`: exponential backoff 1 s → 2 s → 4 s → 8 s (cap), and **re-init after 10 consecutive read
  failures** (`reinitCount` reported).
- `SensorAcquisitionTask::Run`: re-anchor `lastWake` after a missed deadline (no catch-up burst).
- UART RX buffer 512 → 1024 B (~89 ms at 115 200 baud).
- New observability: `lastCycleTimeMs`, `initAttemptsDeferred`, per-source `reinitCount`.

**Alternatives considered.** Separate IMU task (more stacks/locking, not needed given the timing budget
in RTOS_TIMING_ANALYSIS.md); moving inits out of the loop to a low-priority task (needs bus locking with
the sampler, and init must not race reads); event-driven DRDY interrupts (needs pins). Kept for later.

**Tests (12 new).** `audit_b_*`: blocking reproduction, non-blocking mag (0 ms waited per cycle),
late-poll acceptance, mag timeout, three simultaneous IMU init failures (≤ 53 ms instead of 150 ms; 2
deferred), two barometers failing (gaps 1/2/4/8/8 s, second baro not starved, 0 overruns), dead-after-init
re-init and recovery, detection time (unhealthy after 3 reads = 30 ms), full 8-source set (0 ms waits in
steady state, LiDAR drained each cycle, expected sample counts), overrun observability, GNSS backlog
cleared in 3 bounded cycles.

**Acceptance.** No unbounded blocking in the acquisition path ✓ (all waits bounded; see timing
document for the worst-case table) · init failures cannot starve others ✓ · deadline misses and
overruns observable ✓ · multiple simultaneous failures tested ✓ · `service(now, sink)` and the hub
interface still compile unchanged ✓. **Hard real-time performance is NOT claimed** (no hardware).

---

## Audit C — SensorDataHub overflow policies

**Hypothesis.** Drop-newest can make navigation/safety consumers process old data.

**Evidence.** With an 8-deep drop-newest queue and a consumer taking one sample per 100 ms, served
samples are ~800 ms old (`audit_c_slow_consumer_sees_fresh_data_with_drop_oldest`: worst ≥ 700 ms).
Ordering and timestamps were otherwise correct; per-sensor sequence numbers already existed but no
consumer-side tool used them.

**Verdict.** **CONFIRMED** for fresh-data consumers; drop-newest is still right for logging.

**Policy per consumer (implemented, configurable).**
| Consumer | Mechanism | On overflow | Loss visible via |
|---|---|---|---|
| Navigation | `subscribe(q, OverflowPolicy::DropOldest)` | oldest queued sample discarded, new one enqueued (order kept) | `overwritesFor(i)`, sequence gaps |
| Safety monitoring | `LatestSampleMailbox` (one slot per sensor) | slot overwritten | `writes(id)`, sample age (`isFresh`) |
| Logging | `subscribe(q)` (DropNewest, default) | new sample not delivered; unbroken prefix kept | `dropsFor(i)`, sequence gaps |
| Diagnostics | DropNewest or mailbox | as above | as above |

**Safety of the operations.** Drop-oldest on `RtosSampleQueue` = producer does `xQueueReceive(…,0)` then
`xQueueSend(…,0)`. Each FreeRTOS queue call is atomic; with one producer and one consumer both
receiving, they simply get different items; only the producer adds items, so the freed slot is still
free. `xQueueOverwrite` was **not** used because it is only valid for length-1 queues. The mailbox copies
inside a critical section (`taskENTER_CRITICAL`, ~88 bytes); host test with real threads + ThreadSanitizer:
0 torn reads, 0 races. Statistics are never reset. Memory stays fixed (mailbox 776 B).

**Tests (7 new + 2 concurrency).** drop-oldest vs drop-newest contents/counters, slow consumer
freshness, mailbox latest-value and staleness, freshness across the 32-bit wrap, sequence gaps with
mixed rates (accounting identity holds), duplicate/out-of-order detection, publisher never blocks,
SPSC ring buffer under real threads (200 000 bytes, order + accounting), mailbox never torn.

**Residual.** The single default subscriber today is the task's own mailbox; no consumer task exists yet
(algorithms are next milestone). Queue depths must be re-chosen per consumer when they are added.

---

## Audit D — FreeRTOS memory and fault handling

| Item | Finding | Change |
|---|---|---|
| Stack-overflow checking | `configCHECK_FOR_STACK_OVERFLOW` absent (=0). | **Enabled = 2** through the `.ioc` |
| Malloc-failed hook | `configUSE_MALLOC_FAILED_HOOK` absent (=0). | **Enabled = 1** through the `.ioc` |
| How the `.ioc` names were verified | Not hand-guessed: a **disposable copy** with `FREERTOS.configCHECK_FOR_STACK_OVERFLOW=2` / `FREERTOS.configUSE_MALLOC_FAILED_HOOK=1` was regenerated by STM32CubeMX 6.18.1 (CLI). CubeMX kept the keys, wrote the two `#define`s into `FreeRTOSConfig.h` and generated both hooks **inside USER CODE 4/5** in `freertos.c`. Exactly those three CubeMX-generated files were copied into the project. | — |
| Hook bodies | `vApplicationStackOverflowHook` → `paraguardian_on_stack_overflow()`; `vApplicationMallocFailedHook` → `paraguardian_on_malloc_failed()` (calls placed inside the generated USER CODE regions; no duplicate symbols — linked image has one definition each). | `Components/SysCore/FaultHooks.{h,cpp}` |
| Overflow behaviour | interrupts disabled → count + bounded copy of the task name → `BKPT` **only if a debugger is attached** (`CoreDebug->DHCSR.C_DEBUGEN`; a BKPT without debugger would HardFault) → `NVIC_SystemReset()`. No blocking, heap or printing. | |
| What survives reset | `RCC->RSR` reset flags captured at boot into `g_systemHealth.resetFlags` and cleared (SFTRSTF = software reset incl. overflow/assert). The task name and counters do **not** survive (normal .bss). `.noinit` RAM was rejected (STM32H7 SRAM ECC on never-written RAM after power-up); RTC backup registers / flash log left for later. | |
| Stack units | Words everywhere for `xTaskCreateStatic`; bytes only for CubeMX's `osThreadAttr_t` (512 B defaultTask). ✓ | |
| High-water marks | `uxTaskGetStackHighWaterMark` (words) per watched task ✓; with check level 2 the 0xA5 fill pattern is also used by the kernel. | |
| Stack alignment | `StackType_t` arrays (4-byte aligned); FreeRTOS rounds the top down to 8 bytes (`tasks.c` line 862) ✓ | none |
| Heap | 15 360 B heap_4. Allocation call sites in the image: `Task::Task` (SoarOS `new Queue`), `soar_malloc` (`Command::AllocateData`), `xQueueGenericCreate`, `xTaskCreate` — **all start-up only**; none in the sampling path ✓. | |
| Static-init order | Only C++ dynamic initialiser is SoarOS's mutex (Audit A). ParaGuardian singletons are function-local statics constructed in `paraguardian_create_tasks()` after `osKernelInitialize()`; `g_systemHealth` is constant-initialised. ✓ | |
| Debugger dependence | Fault detection no longer needs a debugger (reset + reset flags). | |

**Verdict.** Missing checks **CONFIRMED** and fixed. **UNVERIFIED on hardware**: that the hooks fire
(could be provoked on the bench with a deliberately tiny test task).

---

## Audit E — CMSIS-V2 / SoarOS compatibility

| Check | Result |
|---|---|
| Compat header (`SoarOSCmsisV2Compat.h`) | Still minimal (FreeRTOS includes + `osKernelSysTickFrequency` + static_assert). No change needed. |
| Kernel init / start | One `osKernelInitialize`, one `osKernelStart` (generated); `SOAR_ASSERT(osKernelGetState()==osKernelReady)` before task creation ✓ |
| Stack units / priorities | words; `osPriority_t` names (High 40, BelowNormal 16) < `configMAX_PRIORITIES` 56 ✓ |
| Thread exit | defaultTask → `osThreadExit()` (V2-matching) ✓ |
| Handles | native `TaskHandle_t` for ParaGuardian/SoarOS; CMSIS handle only for defaultTask; no casts between them ✓ |
| ISR-safe APIs | No ParaGuardian ISR calls FreeRTOS. SoarOS `Queue::SendFromISR` passes `NULL` for `pxHigherPriorityTaskWoken` (no immediate yield → up to 1 tick extra latency) — SoarOS issue, unused by ParaGuardian, documented. |
| Timer daemon priority 2 | **Acceptable today**: no software timers are used (SoarOS `Timer` unused, `xTimerPendFunctionCall` unused). **Future risk**: any SoarOS `Timer` callback would run only when all priority ≥ 3 tasks block; the acquisition task blocks ~90 % of each 10 ms, so callbacks would be delayed, not starved — but a busy future algorithm task at Normal (24) could starve them. Recommendation: if timers are introduced, raise `TIMER_TASK_PRIORITY` in CubeMX above the timer users. Not changed (no demonstrated need). |
| Mutex timeouts | SoarOS `Mutex::Lock(ms)` uses `MS_TO_TICKS` (1:1 at 1 kHz) ✓ |

**Verdict.** No new defects; timer priority **DISPROVEN as a current problem**, kept as a documented
future risk.

---

## Audit F — CubeMX regeneration resilience

**Method.** Copied the project to a scratch directory, ran STM32CubeMX 6.18.1 in CLI mode
(`config load …ioc` / `project generate`), compared every file with the pre-generation copy.

**Result.** Only `.cproject` changed: the `Components` and `SoarOS` source entries were **moved** within
the list (same content). Preserved: G++ GNU++17, include paths, `-include` compat header, defines, G++
linker script, source exclusions, `ccnature` in `.project`. `Src/main.c` regenerated byte-identical
(USER CODE kept). The linker script was not touched. A second regeneration with the hook settings
changed exactly `.ioc`, `FreeRTOSConfig.h`, `freertos.c` (+ the same `.cproject` reordering).

**Verdict.** Silent loss of settings **DISPROVEN** for CubeMX 6.18.1 standalone generation. Not tested:
generation triggered from inside CubeIDE, or a CubeMX version upgrade/migration (`AskForMigrate=true`).
Checklist: RTOS_TIMING_ANALYSIS.md is about timing; the **regeneration checklist** is in
FIRMWARE_HARDENING_WALKTHROUGH.txt §F and HARDWARE_INTEGRATION_TODO.md §10.

---

## Audit G — Driver and test regression review

| Area | Finding | Verdict |
|---|---|---|
| Error propagation | All drivers return `Status`; health counters updated via `track()` | OK |
| Mag non-blocking timeout | First version of the new API would have reported `Timeout` for a finished conversion polled late if the timeout were checked first — design checks the done flag first; regression test added | Prevented |
| Test fakes | The magnetometer fake completed conversions after N polls (not time) — unrealistic; made time-based | Test-quality fix |
| LSM6DSRX all-zero check | A sensor in exact free fall, zero rotation, exactly 25.000 °C would read all zeros → `InvalidData`. Practically impossible with noise; documented | Accepted |
| BMP585 `NotReady` | First read right after init may return `NotReady` (one failure, then OK); counted toward re-init only after 10 in a row | OK |
| Flash safety | No erase in init, page-crossing rejected, bounds overflow-safe, WEL verified (tests) | OK |
| Parsers | UBX/TF02/EZ-Serial bounded; recovery tests pass | OK |
| Bluetooth | Still no invented commands | OK |
| Race conditions | Ring buffer and mailbox tested with real threads under ThreadSanitizer | OK |

Test counts: **98 before → 120 after** (22 added), all passing under ASan+UBSan and under TSan.

---

## Summary

| Concern | Verdict | Severity | Fixed? |
|---|---|---|---|
| A. Pre-scheduler HAL tick frozen by BASEPRI | CONFIRMED | Medium now / high future | Yes (HAL_GetTick fallback) |
| A. RtosClock pre-scheduler millis frozen; DWT wait possibly unbounded | CONFIRMED / risk | Medium | Yes |
| B1. Blocking magnetometer read | CONFIRMED | High | Yes |
| B2. Init retries stacking in one cycle | CONFIRMED | Medium-high | Yes (budget + backoff) |
| B3. No re-init after sensor loss | CONFIRMED | Medium | Yes |
| B4. Catch-up burst after missed deadline | CONFIRMED | Low-medium | Yes |
| B5. HAL I2C 25 ms busy wait | CONFIRMED | Medium (if I2C used) | Documented |
| B7. UART buffer too small for a stall | CONFIRMED | Low | Yes (1024 B) |
| C. Drop-newest serves stale data | CONFIRMED | Medium | Yes (policies + mailbox) |
| D. No stack-overflow / malloc-failed detection | CONFIRMED | Medium | Yes |
| E. Timer daemon priority 2 | DISPROVEN (now) | Future risk | Documented |
| F. Regeneration silently undoes settings | DISPROVEN (CubeMX 6.18.1 CLI) | — | Checklist |
| SoarOS modification needed | Not needed | — | SoarOS untouched |
