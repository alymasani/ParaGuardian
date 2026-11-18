# Sensor Driver Test Report

Date: 2026-11-14 · Hardware available: **none** (no PCB, no sensors)

## Status categories

| Category | Meaning |
|---|---|
| **IMPLEMENTED** | Code written for the listed features. |
| **COMPILED** | Compiled for the STM32H743 by the native STM32CubeIDE build (headless), Debug and Release, 0 errors, 0 warnings. |
| **MOCK TESTED** | Exercised by host unit tests against simulated devices (fakes), with expected values computed independently. |
| **HARDWARE VERIFIED** | Run against the real part. **Nothing is in this category.** |

## Summary

| Module | Files | IMPLEMENTED | COMPILED | MOCK TESTED | HARDWARE VERIFIED |
|---|---|---|---|---|---|
| Common (Status, ByteOrder, RegisterBus, ByteRingBuffer) | `Components/Common/*` | Yes | Yes | Yes (10 tests) | No |
| LSM6DSRX IMU ×3 | `Components/Sensors/LSM6DSRX/*` | Yes | Yes | Yes (12 tests) | No |
| BMP585 baro ×2 | `Components/Sensors/BMP585/*` | Yes | Yes | Yes (10 tests) | No |
| MMC5983MA mag | `Components/Sensors/MMC5983MA/*` | Yes | Yes | Yes (10 tests) | No |
| NEO-M9N GNSS (UBX) | `Components/Sensors/NEOM9N/*` | Yes | Yes | Yes (16 tests) | No |
| TF02-Pro LiDAR | `Components/Sensors/TF02Pro/*` | Yes | Yes | Yes (10 tests) | No |
| W25Q128JV flash | `Components/Storage/W25Q128/*` | Yes | Yes | Yes (12 tests) | No |
| CYW20822 EZ-Serial link | `Components/Communications/Bluetooth/*` | **Partial** (framing only) | Yes | Yes (9 tests incl. SOS) | No |
| Acquisition (hub, sources) | `Components/Acquisition/*` | Yes | Yes | Yes (9 tests) | No |
| Audio / LED / Power | `Components/Output/*`, `Components/Power/*` | Interfaces only | Yes (headers) | No (no logic) | No |
| HAL adapters (SPI, I2C, UART) | `Components/BSP/Hal*` | Yes | I2C: yes. SPI/UART: **compiled out** (`HAL_SPI/UART_MODULE_ENABLED` undefined — no such peripheral in the `.ioc`) | No | No |
| RTOS tasks, startup | `Components/Tasks/*`, `Components/SysCore/*` | Yes | Yes | No (needs target) | No |

## Actual build results

Native STM32CubeIDE 2.2.0 headless build (`org.eclipse.cdt.managedbuilder.core.headlessbuild`) of a
**copy** of the project (so the IDE could not alter the original files), GNU Tools for STM32 14.3.rel1:

| Configuration | Result | text | data | bss |
|---|---|---|---|---|
| Debug (`-O0 -g3`) | **Build Finished. 0 errors, 0 warnings** | 25 936 | 24 | 27 312 |
| Release (`-Os`) | **Build Finished. 0 errors, 0 warnings** | 15 680 | 24 | 27 304 |

Compiled C++ units (28): all 21 ParaGuardian `.cpp` files + SoarOS `Command, Mutex, Queue, Task, Timer,
CubeDefines, CubeTask`. Flags include `-std=gnu++17 -fno-exceptions -fno-rtti -Wall` and the forced
include of `SoarOSCmsisV2Compat.h`.

Baseline (before this session, same method): **Build Failed, 39 errors** — multiple definitions from
the Finder duplicate `stm32h7xx_hal_tim_ex 2.c`.

### Compiler warnings
- CubeIDE build (`-Wall`): **0 warnings**.
- Extra strict pass, arm-none-eabi-g++ 14.3 `-O2 -Wall -Wextra -Wshadow -Wpedantic` over every
  `Components/**/*.cpp`: **0 warnings in ParaGuardian code.** Warnings appear only in SoarOS headers:
  - `SoarOS/CubeDefines.hpp`: program should also define `operator delete(void*, std::size_t)` [-Wsized-deallocation]
  - `SoarOS/Core/Inc/Command.hpp`: parameter `taskCommand` shadows a member [-Wshadow]
  (SoarOS not modified; documented in CMSIS_V2_SOAROS_MIGRATION.md §9.)

## Actual unit test results

Command: `sh Tests/run_host_tests.sh`
Compiler: Apple clang 17.0.0, flags `-std=c++17 -O1 -g -Wall -Wextra -Wpedantic -Wshadow -Werror -fsanitize=address,undefined`

```
98 tests, 98 passed, 0 failed
```

First run: **96/98**. The two failures were investigated:
1. `mmc5983ma_set_reset_removes_offset` — **real driver bug**: after the SET/RESET offset-cancel sequence
   the driver restored its shadow copy of `Auto_SR_en` but did not write it back to the write-only CTRL0
   register. Fixed in `Mmc5983ma::measureOffsetCancelled()`.
2. `w25q_sector_erase_alignment_and_effect` — **test bug** (prepared only 8 KiB, then checked byte
   0x2000). Test corrected.

### What the tests cover (by requirement)

| Requirement | Tests |
|---|---|
| Correct register transactions | `lsm6dsrx_init_success_writes_and_verifies`, `bmp585_init_sequence`, `mmc5983ma_init_sequence`, `spi_register_bus_sets_read_bit_and_write_frame`, `w25q_read_command_bytes_big_endian_address` (fakes reject a wrong R/W bit) |
| Data conversion / signed values | `byteorder_*`, `lsm6dsrx_read_converts_to_si_units`, `lsm6dsrx_negative_temperature_and_status_bits`, `bmp585_converts_pressure_and_temperature`, `bmp585_negative_temperature_sign_extended`, `mmc5983ma_decode_18bit_hand_computed`, `nav_pvt_decoded_to_si_units`, `tf02_valid_frame_decoded` |
| Invalid chip ID | `lsm6dsrx_wrong_who_am_i_rejected_without_writes`, `bmp585_rejects_bmp581_chip_id`, `mmc5983ma_wrong_product_id`, `w25q_dtr_variant_accepted_wrong_part_rejected` |
| Initialization failure | `bmp585_nvm_error_and_missing_por_are_not_ready`, `bmp585_invalid_osr_odr_combination_detected`, `lsm6dsrx_verify_failure_detected`, `imu_source_retries_init_at_interval` |
| Bus errors | `lsm6dsrx_bus_error_during_init_propagates`, `i2c_register_bus_wrong_address_is_bus_error`, `w25q_bus_error_propagates` |
| Timeouts | `lsm6dsrx_reset_never_completes_times_out`, `mmc5983ma_measurement_timeout`, `bmp585_forced_measurement_waits_for_standby`, `w25q_stuck_busy_times_out`, `ezs_response_timeout_and_fault`, `gnss_stale_solution_invalidated`, `tf02_stale_timeout`, `ubx_parser_partial_frame_timeout_then_recovery` |
| Short transfers / bad arguments | `register_bus_rejects_bad_arguments`, `w25q_bounds_checking`, `tf02_frame_split_across_calls`, `ezs_line_split_across_reads` |
| Invalid measurements | `lsm6dsrx_floating_bus_is_invalid_data`, `lsm6dsrx_saturation_flags`, `bmp585_zero_pressure_not_ready_and_out_of_range`, `mmc5983ma_saturation_marks_invalid`, `nav_pvt_validity_rules`, `tf02_weak_and_overexposed_signal_invalid`, `tf02_out_of_range` |
| Multiple instances | `lsm6dsrx_three_instances_are_independent`, `bmp585_two_instances_independent` |
| Parser corruption / checksum | `ubx_parser_rejects_corrupt_checksum_then_recovers`, `gnss_checksum_errors_counted_in_health`, `tf02_bad_checksum_rejected`, `ubx_checksum_matches_published_frames` |
| Recovery from partial packets | `ubx_parser_truncated_frame_recovers_within_two_frames`, `ubx_parser_oversize_frame_does_not_overflow`, `tf02_resync_inside_corrupted_window`, `tf02_garbage_then_valid_frame`, `ezs_overlong_line_discarded_safely` |
| Flash page / address boundaries | `w25q_page_program_rejects_page_crossing`, `w25q_write_splits_at_page_boundary_and_data_correct`, `w25q_large_write_spanning_many_pages`, `w25q_sector_erase_alignment_and_effect`, `w25q_bounds_checking` |
| Buffer overflow policy | `ring_buffer_overflow_drops_newest_and_counts`, `hub_drop_newest_per_subscriber`, `gnss_source_drains_bounded_amount_per_cycle` |

Independent expected values: datasheet constants and hand arithmetic written in test comments
(e.g. 2048 LSB × 0.488 mg/LSB × 9.80665 = 9.8010 m/s²), the published u-blox frames
`B5 62 06 08 06 00 64 00 01 00 01 00 7A 12` and `B5 62 0A 04 00 00 0E 34`, and checksums computed
separately in Python.

## Mocked functions

| Fake | Replaces | Behaviour modelled |
|---|---|---|
| `Tests/mocks/MockClock.hpp` | `IClock` | time advances only on `delayMs` |
| `FakeRegisterDevice` + `FakeSpiRegisterDevice` / `FakeI2cBus` | `ISpiDevice` / `II2cBus` | 128-register bank, bit-7 read protocol check, auto-increment, write log, hooks, error injection, floating bus, I2C NACK |
| `FakeW25q` | `ISpiDevice` | JEDEC, SR1 BUSY/WEL, WEL required, NOR AND-programming, **in-page wrap**, sector/block erase, write protect, stuck busy, bus error |
| `FakeByteSource` / `FakeByteWriter` | `IByteSource` / `IByteWriter` | UART RX queue / TX recorder |

## Known bugs / limitations

- **EZ-Serial (CYW20822): partial.** Only line framing, `@R`/`@E` classification, timeouts and callbacks.
  No command or event names (unverified for CYW20822). Cannot and does not report SOS delivery.
- LSM6DSRX: no FIFO, no interrupts/data-ready pin, no self-test, no gyro/accel filters configured
  beyond defaults; `XL_FS_MODE`-type alternative full-scale mapping not touched (defaults assumed after reset).
- BMP585: no FIFO, interrupts or OOR detection; `CHIP_ID 0x51` from the Bosch API must be confirmed.
- MMC5983MA: control registers are write-only, so configuration cannot be read back; continuous-mode
  timing untested; temperature is the chip's uncalibrated value. No hard/soft-iron calibration.
- NEO-M9N: configuration key IDs taken from the u-blox M9 interface description must be re-verified;
  UART baud-rate change not implemented (default 38400 assumed); only NAV-PVT decoded.
- TF02-Pro: invalid-reading thresholds and default unit/rate from the manual; out-of-range sentinel not
  confirmed on a real unit.
- W25Q128: no status-register protection management, no quad/fast read, no power-down entry, no chip
  erase API (deliberate). `write(..., verify=true)` uses a 256-byte stack buffer.
- HAL I2C `writeThenRead` with > 2 address bytes falls back to STOP+START (no repeated start).
- HAL UART receive is 1 interrupt per byte; DMA recommended later.

## Unverified behaviour (needs hardware)

Real SPI modes and clock limits; chip-select timing; I2C pull-ups and addresses; reset/boot times;
actual noise/offsets; UART framing errors; GNSS fix quality; LiDAR range/strength behaviour;
flash program/erase times; task timing (`missedDeadlines`, `maxCycleTimeMs`), stack high-water marks,
heap usage, interrupt latency; FreeRTOS start-up on the target.

## Hardware dependencies

Every driver depends on the schematic for bus instance, chip-select/address and interrupt pins —
see `Docs/HARDWARE_INTEGRATION_TODO.md`.

## Outstanding work

1. Schematic → CubeMX peripherals → `BoardConfig.cpp` wiring.
2. Bring-up per HARDWARE_INTEGRATION_TODO (identity → config → data plausibility → rates).
3. Verify EZ-Serial command reference for the CYW20822 firmware, then implement commands/events.
4. Enable stack-overflow / malloc-failed hooks in CubeMX and forward them.
5. DMA for UART RX (GNSS/LiDAR) and, if needed, SPI.
