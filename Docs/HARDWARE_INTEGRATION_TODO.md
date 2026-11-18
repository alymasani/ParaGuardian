# Hardware Integration TODO

Everything here is **unknown until the ParaGuardian schematic exists**. No pin, peripheral instance,
address or electrical value below has been assumed in code. Fill in the "Decision" column from the
schematic, then follow the CubeMX and bring-up steps.

## 1. Required peripheral instances

| Function | Interface options | Peripheral needed | Decision (schematic) |
|---|---|---|---|
| IMU0..2 (LSM6DSRX ×3) | SPI (≤ 10 MHz, modes 0/3) or I2C (≤ 1 MHz) | 1–3 SPI buses or I2C | TBD |
| Baro0..1 (BMP585 ×2) | SPI (max clock/mode: check datasheet) or I2C/I3C | SPI or I2C | TBD |
| Mag (MMC5983MA) | SPI (max clock/mode: check datasheet) or I2C (400 kHz) | SPI or I2C | TBD |
| GNSS (NEO-M9N, MIKROE-3922) | UART (default 38400 8N1) or I2C (DDC 0x42) | USART/UART | TBD |
| LiDAR (TF02-Pro) | UART (default 115200 8N1) | USART/UART | TBD |
| Flash (W25Q128JV) | SPI (03h read ≤ 50 MHz, else 133 MHz with 0Bh) or QSPI | SPI (or QUADSPI later) | TBD |
| Bluetooth (CYW20822 EZ-Serial) | UART (baud per module firmware; check flow control) | USART/UART | TBD |
| Audio (MAX98357A) | I2S / SAI (+ SD_MODE GPIO) | SAI or SPI/I2S | TBD |
| Status LED(s) | GPIO or TIM PWM | GPIO/TIM | TBD |
| Battery voltage | ADC channel through divider | ADC | TBD |
| Debug console (optional) | UART, LL driver (SoarOS UARTDriver) | USART/UART | TBD (currently `DISABLE_DEBUG`) |

The H743 has SPI1–6, I2C1–4, USART1/2/3/6, UART4/5/7/8, LPUART1, SAI1–4, QUADSPI.
Watch kernel-clock domains (SPI123 vs SPI45 vs SPI6) when choosing.

## 2. Pin-mapping requirements (per device)

| Signal | Needed for | Notes |
|---|---|---|
| SCK/MOSI/MISO | each SPI bus | MISO needs a defined level when no device drives it; a floating MISO reads 0xFF (drivers report `InvalidData`/`WrongDeviceId`) |
| CSn | each SPI device | GPIO output, **default high** in CubeMX (GPIO output level = High) so no device is selected at reset |
| SDA/SCL | each I2C bus | open-drain, external pull-ups |
| INT/DRDY | IMUs (INT1/INT2), BMP585 (INT), MMC5983MA (INT) | EXTI inputs; optional — drivers currently poll |
| TX/RX | GNSS, LiDAR, BLE | cross TX↔RX; check voltage levels |
| RESET / PWR_EN | GNSS (RESET_N), BLE (reset), LiDAR power | GPIO if present |
| SAI SD/SCK/FS | MAX98357A | + SD_MODE pin for shutdown/channel select |

## 3. Sensor addresses (I2C, 7-bit) — only if I2C is chosen

| Part | Address options | Set by |
|---|---|---|
| LSM6DSRX | 0x6A (SA0=0) / 0x6B (SA0=1) | SA0 pin → max **2 IMUs per I2C bus** (3 IMUs need 2 buses or SPI) |
| BMP585 | 0x46 / 0x47 | SDO pin |
| MMC5983MA | 0x30 | fixed |
| NEO-M9N DDC | 0x42 | fixed (if I2C used instead of UART) |

## 4. Chip-select signals

One CSn GPIO per SPI device: IMU0, IMU1, IMU2, BARO0, BARO1, MAG, FLASH (+ any others).
`HalSpiDevice(hspi, csPort, csPin, timeoutMs, busLock)` takes them as constructor arguments.

## 5. Interrupt lines (optional now, recommended later)

IMU INT1 (data-ready), BMP585 INT (data-ready), MMC5983MA INT (measurement done), GNSS TIMEPULSE (1 PPS).
If an EXTI ISR signals a task with a FreeRTOS `...FromISR` call, its NVIC priority must be **5..15**.

## 6. UART configurations

| Device | Baud | Format | Flow control | Notes |
|---|---|---|---|---|
| NEO-M9N | 38400 default | 8N1 | none | send `NeoM9n::buildEnableNavPvtFrame()` after boot; check ACK |
| TF02-Pro | 115200 default | 8N1 | none | 100 Hz frames default |
| CYW20822 EZ-Serial | per module firmware docs (verify) | 8N1 (verify) | verify RTS/CTS | |
In CubeMX: Asynchronous, HAL driver for these ports (`HalUartPort`), NVIC interrupt enabled, priority ≥ 5.

## 7. Electrical constraints to check on the schematic

- Supply ranges (typical datasheet values quoted from memory — **confirm each in the current datasheet**): LSM6DSRX 1.71–3.6 V; BMP585 VDD 1.65–3.6 V / VDDIO 1.08–3.6 V; MMC5983MA 2.8–3.6 V(VDD), VDDIO 1.62 V–VDD; W25Q128JV 2.7–3.6 V; TF02-Pro 5 V supply with 3.3 V TTL UART (verify); MIKROE-3922 3.3 V (verify jumper).
- 3.3 V logic on all MCU-facing pins (H743 GPIOs are 3.3 V; most are 5 V tolerant "FT" — check each pin).
- Decoupling per datasheets; magnetometer away from current loops / speaker magnet.
- MAX98357A supply and speaker current; keep away from the magnetometer.

## 8. Clock requirements

- Currently **HSI 64 MHz, no PLL, no external crystal** (from the `.ioc`). Do not add HSE unless the schematic has one.
- SPI kernel clocks in the `.ioc`: SPI123 129 MHz (PLL-derived values shown by CubeMX but PLL is off —
  re-check in the Clock Configuration tab once SPI is enabled; choose prescalers so SCK ≤ device max).
- UART baud accuracy from 64 MHz HSI is fine for 38400/115200; HSI accuracy ±1–2 % over temperature.

## 9. DMA configuration (later)

- UART RX for GNSS/LiDAR: `HAL_UARTEx_ReceiveToIdle_DMA`, circular.
- DMA buffers must be in **RAM_D1/D2** (not DTCMRAM at 0x20000000 — DMA1/2 cannot reach DTCM).
  The linker script already places `.bss` in RAM_D1 (0x24000000).
- If the D-cache is ever enabled, DMA buffers need cache maintenance or an MPU non-cacheable region.

## 10. CubeMX changes needed (in this order)

1. Enable the chosen SPI / I2C / UART peripherals and their pins; set CS GPIOs to output, level High.
2. UARTs used by `HalUartPort`: HAL driver, global interrupt enabled, priority ≥ 5.
3. Debug UART (optional): LL driver (SoarOS `UARTDriver`), then remove `DISABLE_DEBUG` in `Components/SystemDefines.hpp`.
4. **DONE (2026-11-17 hardening):** `CHECK_FOR_STACK_OVERFLOW = 2` and `USE_MALLOC_FAILED_HOOK = 1` are set in the `.ioc` and the generated hooks forward to `Components/SysCore/FaultHooks.h`. Still to decide:
   consider `TOTAL_HEAP_SIZE` ≥ 32 KB if SoarOS CubeTask/UARTTask are started.
   Then, in the generated hooks in `Src/freertos.c` (USER CODE), call
   `paraguardian_on_stack_overflow(xTask, pcTaskName)` / `paraguardian_on_malloc_failed()`.
5. (Optional) CRC peripheral if SoarOS `Utils::` functions are wanted; then remove the `Core/CubeUtils.cpp` exclusion.
6. Generate code. **Afterwards verify** in `.cproject`: G++ include paths, `-include` compat header,
   GNU++17, source entries `Components`/`SoarOS` and their exclusions, `ccnature` in `.project`.
   Verify `Src/main.c` USER CODE blocks still contain the ParaGuardian calls.
7. If a UART uses `HalUartPort`, add to `Src/main.c` USER CODE 4:
   ```c
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) { paraguardian_uart_rx_complete(huart); }
   void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)  { paraguardian_uart_error(huart); }
   ```
   (and declare those two functions — they are in `Components/BSP/HalUartPort.hpp`, C linkage).
8. Wire the drivers in `Components/BSP/BoardConfig.cpp` (template inside `#if 0`).

## 11. Physical bring-up procedure (one device at a time)

General order: power rails → SWD connect → firmware boots (breakpoint in `paraguardian_create_tasks`) →
`g_systemHealth.reportCount` increasing in Live Expressions → add ONE device in BoardConfig → next.

| Step | Check | Pass criterion | If it fails |
|---|---|---|---|
| 1 | Logic analyser on SCK/CS/MOSI/MISO | CS low only during transfers; SCK frequency ≤ limit | CubeMX SPI mode/prescaler, CS default level |
| 2 | Identity read | LSM6DSRX 0x6B, BMP585 0x51, MMC5983MA 0x30, W25Q EF 40 18 | `WrongDeviceId`: wiring, SPI mode (CPOL/CPHA), CS, MISO pull |
| 3 | `init()` result | `Status::Ok` (watch `sources[i].lastError`, `initFailures`) | `VerifyFailed` → SPI write path; `Timeout` → reset bit never clears |
| 4 | Static plausibility | IMU at rest: \|a\| ≈ 9.81 m/s², gyro ≈ 0 ± few mdps; baro ≈ local pressure (~88–90 kPa in Calgary); mag \|B\| ≈ 50–60 µT; temps ≈ room | axis/sign issues → record board orientation |
| 5 | Dynamic | rotate board 90°: gravity moves to the expected axis; gyro sign matches right-hand rule | PCB orientation table for the BSP |
| 6 | Rates | `acquisitionCycles` ≈ 100/s; `missedDeadlines`, `cycleOverruns` stay 0; `maxCycleTimeMs` < 10 | slow SPI, blocking calls |
| 7 | GNSS | UART bytes arrive (`parserStats().discardedBytes` grows with NMEA); after VALSET → ACK; outdoors NAV-PVT `gnssFixOk` | baud rate, TX/RX swap |
| 8 | LiDAR | `framesOk` ≈ 100/s; distance vs tape measure at 1, 5, 10 m | unit (cm/mm), baud |
| 9 | Flash | JEDEC; erase a **test sector only**; write/verify a pattern across a page boundary; read back after power cycle | WP#/HOLD# pins must be high |
| 10 | Stack/heap | `tasks[i].stackHighWaterWords` > 64; `minimumEverFreeHeapBytes` sensible | increase stacks / heap |

**Do not** treat a successful bench bring-up as evidence the device is safe for paragliding use.

## 12. Sensor preprocessing: decisions that need the hardware (Milestone 2)

Details: `Docs/SENSOR_PREPROCESSING_DESIGN.md`. None of these can be settled without the PCB.

| Item | What to do | Where it goes |
|---|---|---|
| Sensor → body mountings | From the PCB layout/enclosure drawing, write each IMU's and the magnetometer's axis map; verify with the board level, then on each side (gravity on the expected body axis, sign by right-hand rule) | `pipeline.alignment().setMounting(id, R, /*verified=*/true)` at start-up (e.g. in `BoardConfig`); until then frames carry `quality::MountingUnverified` |
| Stationary / calibration thresholds | Record ≥ 60 s still on a table and while a person stands still wearing the module; set limits to 3–5× the measured std and segment spread | `StationaryConfig`, `GyroCalibrationConfig`, `BaroCalibrationConfig` |
| Redundancy tolerances | Measure the steady IMU-to-IMU differences (accel offsets, gyro residuals) and the barometer difference over temperature; set tolerances above them with margin | `PreprocessingConfig::accelVoter / gyroVoter / baroVoter` |
| Lever arm | IMU spacing on the PCB × expected max rotation rate → required accel tolerance | `accelVoter.absTolerance` |
| Sensor latencies | Measure per-sensor delay (data-ready pins / scope) if timing matters for fusion | `FixedLatencyTimeModel` |
| Processing load | Toggle a spare GPIO around ingest/buildFrame; read `processingMaxTimeMs`, `SensorProc` stack high-water mark | `TaskConfig::PROCESSING_STACK_WORDS` (can likely shrink) |
| Critical section length | Measure `ProcessedFrameMailbox::publish` (420 B copy with BASEPRI raised) | switch to a double buffer if interrupt latency matters |
| Accelerometer calibration | Multi-orientation (6-face) procedure; fill `AccelCalibration` per IMU | `pipeline.setAccelCalibration()` |
| Magnetometer calibration | Hard/soft-iron calibration with the module in its enclosure | future module; frames carry `MagNotIronCalibrated` |
| Barometric altitude model | Decide `StandardSeaLevelScale` (specified) vs `IsaAtReference` after comparing with GNSS altitude at a high site | `BaroCalibrationConfig::altitudeModel` |
| GNSS time | Route the NEO-M9N TIMEPULSE to a timer input-capture pin if MCU↔GPS time alignment is needed | new `IMeasurementTimeModel` implementation |
