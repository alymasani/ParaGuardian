# SoarOS on CMSIS-RTOS V2 — Migration Record

Date: 2026-11-14 · Project: ParaGuardian (STM32H743ZIT6) · Status: **compiles and links; not run on hardware**

## 1. Versions

| Item | Version |
|---|---|
| SoarOS (UCSOAR/SoarOperatingSystem) | `master` @ **426c1efe895bb58ba2a57ea128c9aad642e9be9f** ("added TC and PT to databroker (#12)") |
| SoarOS working tree | clean, unmodified (verified with `git -C SoarOS status`) |
| STM32CubeMX | 6.18.1 (from `.ioc`: `MxCube.Version=6.18.1`) |
| STM32Cube FW_H7 | V1.13.0 |
| FreeRTOS kernel | V10.6.2 (`Middlewares/Third_Party/FreeRTOS/Source/include/task.h`) |
| CMSIS-RTOS API | V2 (`osCMSIS 0x20001`, `cmsis_os2.c` ST wrapper) |
| STM32CubeIDE | 2.2.0, GNU Tools for STM32 14.3.rel1 |

## 2. CMSIS V1 dependencies found in SoarOS

SoarOS does **not** call CMSIS V1 thread/queue functions (`osThreadCreate`, `osMessageGet`, ...).
It calls native FreeRTOS functions directly. Its V1 dependencies are header-level only:

| SoarOS file | Dependency | Breaks under V2 because |
|---|---|---|
| `Core/Inc/Mutex.hpp` | includes only `cmsis_os.h`, uses `SemaphoreHandle_t` | V2 `cmsis_os.h` includes only `cmsis_os2.h`, not `semphr.h` |
| `Core/Inc/Task.hpp` | uses `TaskHandle_t` after `cmsis_os.h` | V2 header does not include `task.h` |
| `Core/Inc/Timer.hpp` | `TimerHandle_t`, `MS_TO_TICKS` at namespace scope | needs `timers.h` + the macro below |
| `Core/Inc/CubeUtils.hpp` | `MS_TO_TICKS`/`TICKS_TO_MS` use **`osKernelSysTickFrequency`**; `MAX_DELAY_MS` is a `constexpr` using it | macro exists only in V1; V2 offers the *function* `osKernelGetTickFreq()`, unusable in `constexpr` |
| `CubeDefines.hpp` | `pvPortMalloc`, `vTaskSuspendAll` via `cmsis_os.h` | same missing-include issue |
| README startup recipe | `run_interface(); #if 0` around generated scheduler start; `run_main()` calls `osKernelStart()` | under V2 this skips `osKernelInitialize()`; `osKernelStart()` then returns `osError` and the scheduler **never starts** (verified in `cmsis_os2.c`: start requires `KernelState == osKernelReady`) |

Other non-RTOS build dependencies found:

| SoarOS file | Needs | Present in ParaGuardian? |
|---|---|---|
| `Core/CubeUtils.cpp` | `HAL_CRC_Calculate`, `SystemHandles::CRC_Handle` (CRC peripheral) | No CRC in `.ioc` |
| `Drivers/UARTDriver.cpp` | LL USART (`LL_USART_*`) | No UART in `.ioc` |
| `Drivers/UARTTask.cpp` | `UART::Debug` driver object | No debug UART (pins unverified) |
| `Profiler/ProfilerTask.cpp` | `TASK_PROFILER_*`, run-time-stats timer, `std::vector` | Not configured |
| `Components/DataBroker/*` | fixed rocket message types; heap copy per publish; `std::string` | header-only, not included by ParaGuardian |

## 3. Chosen strategy

Preference order from the task brief, and what was used:

1. **Reuse unchanged** — `Core/{Command,Queue,Mutex,Task,Timer}.cpp`, `CubeDefines.cpp`, `CubeTask.cpp` compile unmodified.
2. **Adapt inside ParaGuardian** — one compatibility header, force-included into C++ only:
   `Components/SysCore/SoarOSCmsisV2Compat.h`
   - includes `FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`, `timers.h`, `cmsis_os.h`
   - `#define osKernelSysTickFrequency configTICK_RATE_HZ` (exactly the V1 meaning)
   - `static_assert(configTICK_RATE_HZ % 1000 == 0)` because SoarOS computes `ms * (freq/1000)` with integer division
   - `#error` if accidentally used with V1
3. **Application-owned task creation** — ParaGuardian tasks derive from SoarOS `Task` but create their thread with `xTaskCreateStatic()`.
4. **Modify SoarOS** — **not needed. SoarOS was not modified; no branch was created.**

No V1→V2 function aliases were written (deliberately — see brief §4.1).

## 4. API-by-API semantic differences

| Area | CMSIS V1 (old SoarOS projects) | CMSIS V2 (ParaGuardian) | What ParaGuardian does |
|---|---|---|---|
| Thread create | `osThreadCreate(&def, arg)`; ST's V1 wrapper passes `osThreadDef` `stacksz` straight to `xTaskCreate`, i.e. effectively **words** | `osThreadNew(fn, arg, &attr)`; `attr.stack_size` in **bytes** (CubeMX: `128 * 4`) | SoarOS/ParaGuardian use native `xTaskCreate[Static]` → stack depth in **words**. Only `defaultTask` uses `osThreadNew`. |
| Priorities | 7 levels, `osPriorityNormal = 0` (−3..+3) mapped onto FreeRTOS 0..6 | `osPriority_t` values **are** FreeRTOS numbers: Low 8, BelowNormal 16, Normal 24, AboveNormal 32, High 40, Realtime 48; `configMAX_PRIORITIES = 56` | `TaskConfig.hpp` uses `static_cast<UBaseType_t>(osPriorityHigh)` etc. so CMSIS and native tasks share one scale. Raw SoarOS-style numbers (2, 3) would be *very* low here. |
| Timer daemon | — | `configTIMER_TASK_PRIORITY = 2` (below every V2 named priority) | ParaGuardian uses no software timers. SoarOS `Timer` callbacks would be starved by Normal+ tasks → documented risk. |
| Kernel init | none required before creating objects | `osKernelInitialize()` must run before `osKernelStart()`; V2 objects should be created after it | Generated sequence kept: `osKernelInitialize()` → `osThreadNew(defaultTask)` → **`paraguardian_create_tasks()`** → `osKernelStart()` |
| Scheduler start | `osKernelStart()` (often called from `run_main()`) | `osKernelStart()` refuses unless state Ready | Started **once**, by generated `main.c`. ParaGuardian never calls it. |
| Thread exit | `osThreadTerminate(osThreadGetId())` | `osThreadExit()` (no return) | `paraguardian_default_task()` → `osThreadExit()` for the CubeMX-created task. |
| Handles | `osThreadId` = `TaskHandle_t` | `osThreadId_t` = opaque (`TaskHandle_t` internally) | Native handles only in ParaGuardian/SoarOS; no casting between the two. |
| Queues | `osMessageQ` | `osMessageQueueNew` | Native: SoarOS `Queue` (`xQueueCreate`, heap) for commands; `RtosSampleQueue` (`xQueueCreateStatic`) for samples. |
| Mutex | `osMutexDef` | `osMutexNew` | SoarOS `Mutex` (native `xSemaphoreCreateMutex`). Unchanged semantics (priority inheritance). |
| Delays | `osDelay(ms)` (ticks == ms at 1 kHz) | `osDelay(ticks)` | Native `vTaskDelay(pdMS_TO_TICKS(ms)+1)` / `xTaskDelayUntil`. |
| Tick freq | macro `osKernelSysTickFrequency` | function `osKernelGetTickFreq()` | Macro re-provided by compat header = `configTICK_RATE_HZ` (1000). |

## 5. Files modified / added

### Generated files modified (inside USER CODE regions only)

| File | Region | Change | Reason |
|---|---|---|---|
| `Src/main.c` | `USER CODE Includes` | `#include "../Components/SysCore/RunInterface.h"` | C prototypes of the C++ entry points (relative path → no C include-path change) |
| `Src/main.c` | `USER CODE RTOS_THREADS` | `paraguardian_create_tasks();` | create tasks after `osKernelInitialize()`, before the single `osKernelStart()` |
| `Src/main.c` | `USER CODE 5` (`StartDefaultTask`) | body replaced by `paraguardian_default_task();` | deliberate handling of the CubeMX default task (it exits) |

### Project configuration (`.cproject`, `.project`)

| Change | Reason |
|---|---|
| `.project`: added `org.eclipse.cdt.core.ccnature` | project was generated as C-only; C++ nature needed for `.cpp` |
| G++ compiler (Debug+Release): defines `USE_PWR_LDO_SUPPLY USE_HAL_DRIVER STM32H743xx` (+`DEBUG` in Debug) | same as the C compiler had; G++ had none |
| G++ include paths: the 9 generated paths + `../Components`, `../SoarOS`, `../SoarOS/Core/Inc`, `../SoarOS/Drivers/Inc`, `../SoarOS/Libraries/embedded-template-library/include` | SoarOS includes headers both by bare name and by `Core/Inc/...` |
| G++ "Include files (-include)": `../Components/SysCore/SoarOSCmsisV2Compat.h` | the V2 compatibility shim (C++ only) |
| G++ language standard: **GNU++17** (`-std=gnu++17`) | brief requirement; ETL profile needs ≥ C++17 |
| G++ linker: linker script `STM32H743ZITX_FLASH.ld` + `$(USER_OBJS) $(LIBS)` | with C++ sources CubeIDE links with g++; its tool had no script |
| Source entries: `Components`, `SoarOS` (excluding `Profiler`, `Libraries`, `Drivers/UARTDriver.cpp`, `Drivers/UARTTask.cpp`, `Core/CubeUtils.cpp`) | compile ParaGuardian + the SoarOS parts that are usable without CRC/UART/profiler |

C compiler settings, the `.ioc`, `FreeRTOSConfig.h` and all other generated files are unchanged.
CubeIDE defaults kept: `-fno-exceptions -fno-rtti -fno-use-cxa-atexit`, `-ffunction-sections -fdata-sections`, `--gc-sections`.

### Pre-existing build fix (not RTOS related)

Five macOS Finder duplicates (`* 2.c`/`* 2.h`, byte-identical to their originals) were moved to
`.finder_duplicates_backup/`. `stm32h7xx_hal_tim_ex 2.c` caused 39 "multiple definition" link
errors: **the untouched project did not link before this session** (verified with a headless build of a copy).

### SoarOS
**No file changed.** No branch created. Submodule still at `426c1ef`.

## 6. Task start-up sequence (as implemented)

```
Reset_Handler (startup_stm32h743zitx.s)
 ├─ copy .data, zero .bss
 ├─ __libc_init_array → C++ static constructors
 │    └─ SoarOS Global::vaListMutex → xSemaphoreCreateMutex()   (pre-kernel FreeRTOS call, see §8)
 └─ main()
     ├─ MPU_Config(), HAL_Init()  (HAL_InitTick → TIM6 1 kHz)
     ├─ SystemClock_Config()      (HSI 64 MHz, no PLL, no external oscillator)
     ├─ MX_GPIO_Init()
     ├─ osKernelInitialize()      → KernelState = Ready
     ├─ osThreadNew(StartDefaultTask)          (dynamic, 512 B stack, prio 24)
     ├─ USER CODE RTOS_THREADS: paraguardian_create_tasks()
     │    ├─ guard: second call ignored + counted
     │    ├─ SOAR_ASSERT(osKernelGetState() == osKernelReady)
     │    ├─ SensorAcquisitionTask::Inst()  (Task(0): no queue)
     │    ├─ pg::board::registerSensors()   (registers nothing until the schematic exists)
     │    ├─ SensorAcquisitionTask::InitTask() → xTaskCreateStatic("SensorAcq", 1024 words, prio 40)
     │    ├─ HealthMonitorTask::Inst()      (Task(4): SoarOS Command queue, heap)
     │    └─ HealthMonitorTask::InitTask()  → xTaskCreateStatic("Health", 512 words, prio 16)
     └─ osKernelStart()  (once; never returns)
          ├─ SensorAcq: 10 ms fixed-rate loop (xTaskDelayUntil)
          ├─ Health:    1 s collection into pg::g_systemHealth
          └─ defaultTask: osThreadExit()
```

## 7. Stack-size units

| Where | Unit | Example |
|---|---|---|
| `xTaskCreate` / `xTaskCreateStatic` `ulStackDepth` | **words** (`StackType_t` = 4 B) | `ACQUISITION_STACK_WORDS = 1024` → 4096 B |
| `osThreadAttr_t::stack_size` (V2) | **bytes** | CubeMX `128 * 4` = 512 B |
| `configMINIMAL_STACK_SIZE` | words | 128 |
| `uxTaskGetStackHighWaterMark` | words | reported as `stackHighWaterWords` |

`TaskConfig.hpp` `static_assert`s each task stack ≥ `configMINIMAL_STACK_SIZE`.
Compiler stack analysis (`-fstack-usage`, Release) — largest frames: `W25q128::write` 312 B,
`LidarSource::service` 200 B, `GnssSource::service` 192 B. Run-time verification on hardware is still
required (`g_systemHealth.tasks[i].stackHighWaterWords`).

## 8. Queue / semaphore behaviour and interrupt restrictions

- SoarOS `Queue::Send` waits up to 15 ms; `Receive(cm, ms)` blocks up to `ms`. Every received
  `Command` must be `Reset()` (frees its heap data) — `HealthMonitorTask::handleCommand` does.
- `RtosSampleQueue::tryPush` uses a **0-tick** timeout (never blocks the sampler). Drops are counted
  per subscriber in `SensorDataHub`.
- No ParaGuardian ISR calls FreeRTOS. The UART ISR (`HalUartPort`, compiled only when a UART exists)
  only writes a lock-free ring buffer. If an ISR ever calls a `...FromISR` API, its NVIC priority must
  be numerically **≥ 5** (`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`).
- **Pre-scheduler interrupt masking (inherited from SoarOS, also true on the DAQ board):** the first
  FreeRTOS call before the scheduler (here: SoarOS's global `vaListMutex` constructor, before `main()`)
  raises BASEPRI to priority 5 and leaves it raised until `vTaskStartScheduler()` (FreeRTOS initialises
  `uxCriticalNesting` to `0xAAAAAAAA` on purpose). TIM6 (priority 15) therefore does not tick before the
  scheduler starts: `HAL_GetTick()` stays frozen and **`HAL_Delay()` before `osKernelStart()` would hang**.
  Generated init code does not call `HAL_Delay`, and HAL timeouts only matter if hardware fails.
  ParaGuardian initialises sensors only inside tasks.
  **Mitigated 2026-11-17:** `Components/SysCore/HalTickFallback.cpp` keeps `HAL_GetTick()` advancing while
  TIM6 is masked (no mask manipulation), and `RtosClock` now uses `HAL_GetTick`/`HAL_Delay` before the
  scheduler (the DWT busy-wait was removed). See FIRMWARE_RELIABILITY_AUDIT.md, Audit A.

## 9. Remaining compatibility risks

1. **CubeMX regeneration** may rewrite `.cproject`. It normally preserves user include paths/options,
   but re-check G++ settings and source entries after every regeneration (see HARDWARE_INTEGRATION_TODO).
2. **SoarOS `operator new/delete`** (`CubeDefines.hpp`) are `inline` replacements without a sized
   `operator delete(void*, size_t)` (GCC `-Wsized-deallocation`, seen with `-Wextra`). ParaGuardian never
   uses `new`/`delete` after start-up except SoarOS's own `new Queue`; avoid `delete` entirely.
3. **SOAR_ASSERT resets the MCU** (`HAL_NVIC_SystemReset`) with `DISABLE_DEBUG`. ParaGuardian uses it
   only for start-up programming errors; runtime faults return `pg::Status`. A reset loop at boot is
   visible with the debugger.
4. **Timer daemon priority 2** < all ParaGuardian tasks (see §4).
5. **FreeRTOS heap 15 360 B** (CubeMX default). Current heap users: SoarOS mutex, Health queue,
   defaultTask (freed on exit). Check `g_systemHealth.minimumEverFreeHeapBytes` on hardware;
   increase `TOTAL_HEAP_SIZE` in CubeMX before adding heap-allocating SoarOS tasks (e.g. CubeTask/UARTTask).
6. **Resolved 2026-11-17:** `configCHECK_FOR_STACK_OVERFLOW = 2` and `configUSE_MALLOC_FAILED_HOOK = 1` are now enabled via the `.ioc` (see FIRMWARE_RELIABILITY_AUDIT.md, Audit D). Original note:
   Recommended: enable in CubeMX and call `paraguardian_on_stack_overflow()` /
   `paraguardian_on_malloc_failed()` from the generated hooks in `Src/freertos.c`.
7. `CubeUtils.cpp` is excluded (needs CRC). `Utils::` functions are therefore unavailable; enable CRC in
   CubeMX and remove the exclusion if they are needed.
8. Nothing has run on hardware. Start-up order, priorities and timing are verified by reading the
   kernel/wrapper source and by compilation only.

## 10. SoarOS branch status and reproducibility

- Original state: `master`, `426c1ef`, clean, no stashes.
- `paraguardian/cmsis-v2` branch: **not created** (not needed).
- Because nothing in SoarOS changed, the parent repo's submodule pointer (`426c1ef`) fully reproduces
  this build on another computer (`git submodule update --init`).
- If SoarOS changes ever become necessary: a submodule records a **commit SHA**, so uncommitted
  submodule edits are invisible to anyone else who clones. Procedure: fork
  UCSOAR/SoarOperatingSystem privately → commit on a branch in the fork → in ParaGuardian run
  `git submodule set-url SoarOS <fork-url>` and `git -C SoarOS checkout <sha>` → commit the new
  submodule pointer in ParaGuardian.
