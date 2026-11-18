# Firmware Regression Report — Reliability Hardening

Date: 2026-11-17 · Hardware verification: **none**

## Host unit tests

| Run | Sanitizers | Result |
|---|---|---|
| Baseline (before any change) | ASan + UBSan | **98 tests, 98 passed, 0 failed** |
| After refactor, before new tests | ASan + UBSan | 98 passed (existing tests unchanged except one constructor argument, see below) |
| Final | ASan + UBSan (`sh Tests/run_host_tests.sh`) | **120 tests, 120 passed, 0 failed** |
| Final | ThreadSanitizer (`SANITIZE=thread sh Tests/run_host_tests.sh`) | **120 passed, 0 ThreadSanitizer warnings** |

Compiler: Apple clang 17.0.0, `-std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Wshadow -Werror -pthread`.

Existing test modified: `imu_source_retries_init_at_interval` — the 5th constructor argument
(`1000u`, retry interval) became the `RetryPolicy` struct; the test now relies on the default policy
(first retry after 1000 ms), so its expectations are unchanged.

### Tests added (22, file `Tests/unit/test_hardening.cpp`)
| Audit | Tests |
|---|---|
| A | `audit_a_basepri_masks_tim6_before_scheduler` |
| B | `audit_b_old_blocking_mag_measure_consumes_cycle`, `audit_b_mag_source_is_non_blocking_two_phase`, `audit_b_mag_nonblocking_timeout_detected`, `audit_b_mag_late_poll_of_finished_conversion_is_not_a_timeout`, `audit_b_simultaneous_init_failures_are_spread_across_cycles`, `audit_b_without_budget_init_waits_stack_up`, `audit_b_two_barometers_fail_together_retry_with_backoff`, `audit_b_dead_sensor_after_init_triggers_reinit_and_recovers`, `audit_b_health_detection_time_three_cycles`, `audit_b_full_sensor_set_normal_cycle_has_no_waits`, `audit_b_overrun_is_observable`, `audit_b_gnss_burst_during_heavy_activity_drains_bounded` |
| C | `audit_c_drop_oldest_keeps_newest_in_order`, `audit_c_slow_consumer_sees_fresh_data_with_drop_oldest`, `audit_c_mailbox_latest_per_sensor_and_staleness`, `audit_c_freshness_across_counter_wrap`, `audit_c_sequence_gaps_mixed_rates`, `audit_c_sequence_tracker_duplicates_and_gaps`, `audit_c_rtos_style_queue_never_blocks_publisher` |
| Concurrency | `concurrency_ring_buffer_spsc_order_and_accounting`, `concurrency_mailbox_reads_are_never_torn` |

Test-development notes (honest record): three new tests initially failed because of wrong assumptions
in the *tests* (counting the source's own init time, an unrealistic poll-count-based magnetometer
fake, and an over-estimated backlog age); the fake was made time-based and the expectations
recalculated. The production code was not changed to make a test pass.

## Firmware builds (native STM32CubeIDE 2.2.0 headless build of a copy, GNU Tools for STM32 14.3)

| Config | Before | After | Δ |
|---|---|---|---|
| Debug | 0 errors, 0 warnings; text 25 936, data 24, bss 27 312 | **0 errors, 0 warnings**; text 28 304, data 24, bss 28 264 | text +2 368 B, bss +952 B |
| Release | 0 errors, 0 warnings; text 15 680, data 24, bss 27 304 | **0 errors, 0 warnings**; text 16 540, data 24, bss 28 264 | text +860 B, bss +960 B |

bss growth is mainly the per-sensor `LatestSampleMailbox` (776 B) inside the acquisition task object
(now 5 148 B incl. its 4 096 B stack) and the larger `g_systemHealth` (476 B).
Flash used: 16.5 KB of 2 MB (Release). RAM: 28.3 KB of 512 KB RAM_D1, of which 15 KB is the FreeRTOS heap.

### Warnings
- CubeIDE build (`-Wall`): 0.
- Extra pass, arm-none-eabi-g++ 14.3 `-O2 -Wall -Wextra -Wshadow -Wpedantic` over all 24 ParaGuardian
  `.cpp` files: **0 warnings in ParaGuardian code**; only the two pre-existing SoarOS header warnings
  (`-Wsized-deallocation` in `CubeDefines.hpp`, `-Wshadow` in `Command.hpp`).
- `Src/freertos.c` (with the hook calls) compiled with `-Wall -Wextra`: 0 warnings.

### Link checks
- No undefined symbols (`arm-none-eabi-nm -u` empty).
- `HAL_GetTick` resolves to ParaGuardian's strong definition; HAL's weak version is discarded.
- `vApplicationStackOverflowHook`, `vApplicationMallocFailedHook`: one definition each (CubeMX-generated).
- Heap-allocation call sites: start-up only (SoarOS `Task`/`Command`, `xQueueGenericCreate`, `xTaskCreate`).

### CubeMX regeneration
STM32CubeMX 6.18.1 CLI regeneration of disposable copies: settings preserved (see audit §F).

### SoarOS
`426c1efe895bb58ba2a57ea128c9aad642e9be9f`, `master`, clean — not modified.

## Known limitations
- No hardware: timing figures in RTOS_TIMING_ANALYSIS.md are estimates; the HAL tick fallback, fault
  hooks, reset-flag capture and RTOS behaviour are verified by code reading, compilation and host tests only.
- Host timing tests model only deliberate waits (`delayMs`), not bus transfer time.
- Cycle statistics have 1 ms resolution.
- HAL I2C 25 ms busy wait remains (HAL behaviour).
- Fault details other than the RCC reset flags do not survive a reset.

## Outstanding validation (hardware)
See RTOS_TIMING_ANALYSIS.md §9: cycle time measurement, `tickFallbackIncrements`, sensor unplug/replug,
stack/heap margins, provoked stack overflow → reset flags, UART overflow under stall.
