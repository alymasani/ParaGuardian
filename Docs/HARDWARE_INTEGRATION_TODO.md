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

## 13. Attitude estimation: decisions that need the hardware (Milestone 3)

Details: `Docs/MADGWICK_ATTITUDE_DESIGN.md`, `Docs/MADGWICK_VALIDATION_REPORT.md`. The attitude
output is **simulation-validated only**.

| Item | What to do | Where it goes |
|---|---|---|
| Verified mountings (prerequisite) | Section 12 first: every attitude output carries `attitude::MountingUnverified` until all mountings are verified | `AxisAlignment::setMounting(..., true)` |
| Sign/convention check | Board level, nose (body +x) north: `rollCentiDeg ≈ pitchCentiDeg ≈ 0`, `verticalAccel_mmps2 ≈ 0`. Raise the nose: pitch positive. Right side down: roll positive. Lift the board quickly: vertical acceleration positive | `g_systemHealth` in Live Expressions |
| Gain `beta_radps` | Record IMU data in calm and turbulent flight; replay through the host estimator; choose beta from tilt error vs a reference (e.g. GNSS-derived or camera) | `AttitudeConfig::beta_radps` |
| Turn gate | Measure typical thermalling turn rates and bank angles; the gate (0.10 rad/s) must sit below the slowest turn that matters | `AttitudeConfig::accelRejectionRate_radps` |
| Gyro bias stability | Measure bias drift over temperature: in long turns/turbulence the attitude is gyro-only for long periods | possibly periodic re-calibration, temperature model |
| In-flight restart | A reset in flight cannot run the 5 s still gyro calibration: attitude stays `Uninitialized`. Decide whether to persist the last gyro biases in flash (W25Q128) and accept them with a "stale calibration" flag | new feature |
| Magnetometer | Hard/soft-iron calibration in the enclosure, check field strength/dip at the flying site, magnetic interference from harness metal, speaker magnet (MAX98357A), battery currents | `MagPolicy::CalibratedOnly` stays the default until then |
| Magnetic declination | Heading is MAGNETIC; add declination (e.g. from GNSS position) if true north is needed | future |
| Centripetal compensation | With GNSS velocity, the turn-induced specific force (w x v) can be removed so the accelerometer correction works during turns instead of being gated | future milestone |
| Timing | Toggle a GPIO around `attitude_.update()`; confirm the 25–80 µs estimate | MADGWICK_VALIDATION_REPORT.md §10 |


## 14. Vertical navigation (UKF): decisions that need the hardware (Milestone 4)

Details: `Docs/VERTICAL_UKF_DESIGN.md`, `Docs/VERTICAL_UKF_VALIDATION_REPORT.md`. The navigation
output is **simulation-validated only**, and the process model is a placeholder.

| Item | What to do | Where it goes |
|---|---|---|
| Sign check | Device still: `navVerticalVelocity_mmps ≈ 0`, `navAltitude_cm ≈ 0` after calibration. Lift it ~1 m quickly: altitude rises, velocity positive during the lift, then ≈ 0 | `g_systemHealth` in Live Expressions |
| Execution time | Toggle a GPIO or read DWT `CYCCNT` around `navigation_.update()` and `navigationOut_.publish()`; confirm the 50–150 µs estimate and the mailbox critical section (a few µs) | VERTICAL_UKF_VALIDATION_REPORT.md §12 |
| Stack | Check the SensorProc high-water mark (`tasks[]` in `g_systemHealth`); 928 B is the static estimate of the deepest navigation chain | `TaskConfig::PROCESSING_STACK_WORDS` |
| Barometer noise on a worn device | Record both BMP585s at rest, walking, and in flight; measure white and **correlated** error (airflow, body); set `baroSigma_m`; decide whether a barometer-error state (N = 4) or an output σ floor is needed (altitude σ is over-confident under correlated error in simulation) | `NavigationConfig::baroSigma_m` |
| Accelerometer bias | Measure the LSM6DSRX zero-g offset per IMU (and over temperature). A 0.2 m/s² bias biases vz by ≈ 0.27 m/s with the current 3-state filter. Calibrate in preprocessing (removes `AccelNotCalibrated`), or add a bias state | preprocessing calibration / future model |
| Process noise | Tune `jerkPsd_m2ps5`, `accelErrorPsd_m2ps3`, the gates and the attitude-quality factors on recorded flights (replay through the host estimator); check NEES/NIS on real data | `NavigationConfig` |
| Measurement latency | BMP585 conversion time and NEO-M9N navigation latency (time pulse needs a verified pin): apply them as measurement times instead of read-completion times | `IMeasurementTimeModel` (preprocessing) |
| GNSS vertical velocity | Compare NAV-PVT `velD` / `sAcc` with the barometer in flight; set `gnssSpeedAccuracyScale`, `gnssMinSigma_mps`; confirm the minus sign (velD positive down → vz positive up) on real data | `NavigationConfig` GNSS fields |
| GNSS altitude | Not used (MSL vs barometric reference). Fusing it needs a datum/offset state | future |
| Process model | Replace `ConstantAccelerationModel` with a paraglider model (polar, bank angle) once one exists and is validated: implement `IVerticalProcessModel`, change `navModel_` | `SensorProcessingTask.hpp` |
| Fallback policy | Decide when the safety layer should use `BaroAlphaBetaEstimator` instead (e.g. UKF `NumericalFault` / `Unavailable`); the estimate already carries `estimator` | future safety package |

## 15. Flight state, LiDAR clearance and landing warnings: decisions that need the hardware (Milestone 5)

Details: `Docs/FLIGHT_LANDING_DESIGN.md`, `Docs/FLIGHT_LANDING_VALIDATION_REPORT.md`. Everything is
**simulation-validated only**; every threshold is a provisional development value; nothing is
safety-qualified.

| Item | What to do | Where it goes |
|---|---|---|
| LiDAR mounting (prerequisite) | Measure the TF02-Pro boresight in BODY axes on the real harness/enclosure (e.g. hang the device level over a floor, compare range with a tape measure at several tilts); then set `GroundClearanceConfig::beamBody` and `mountingVerified = true`. A 10-degree error gives up to 0.85 m error at 10 m | `SafetyConfig.clearance` |
| LiDAR on the ground | Measure the standing / sitting clearance (harness height) and its spread; check `groundMaxClearance_m` (2.5 m) and `unknownPhaseMinClearance_m` | `FlightStateConfig`, `LandingSafetyConfig` |
| LiDAR surfaces | Record signal strength and dropouts over grass, crops, water, snow, rock, in direct sun; confirm the driver's 0.1-40 m range and the out-of-range / weak-signal behaviour on the real module | driver + `GroundClearanceConfig` |
| Beam tilt limit | Check usable tilt (40 degrees) against real thermalling bank angles; availability in steep turns | `maxBeamTiltDeg` |
| Phase thresholds | Record launches (incl. aborted), walking with the wing, car rides, landings, packing up: tune stillness, GNSS speeds, dwell times; count false / missed transitions | `FlightStateConfig` |
| Warning thresholds | Review descent / closure / TTC / clearance thresholds with experienced pilots and instructors for the target wings; record approaches | `LandingSafetyConfig` |
| Timing | Toggle a GPIO or read DWT `CYCCNT` around `safety_.update()`; confirm the 40-130 us estimate and the stack high-water mark | RTOS_TIMING_ANALYSIS.md 4c |
| Event consumer | Implement the audio / logging task that pops `safetyEvents()`; size the queue for its latency; add a second queue for logging (`SafetyMonitor::addSink`) | future milestone |
| Flare indication | Not implemented; would need wing/airspeed/wind knowledge and validation | future, optional, disabled by default |

## 16. Crash detection, emergency / SOS and alert outputs: decisions that need the hardware (Milestone 6)

Details: `Docs/CRASH_EMERGENCY_DESIGN.md`, `Docs/CRASH_EMERGENCY_VALIDATION_REPORT.md`. Everything is
**simulation-validated only**. The transport, audio, LED and persistence backends in firmware are
explicitly UNCONFIGURED (`UnconfiguredSosTransport`, `UnconfiguredAudioAlert`, `UnconfiguredStatusLed`,
`NullEmergencyStore`); nothing claims a speaker, LED, BLE link, phone or emergency contact works.

| Item | What to do | Where it goes |
|---|---|---|
| Impact data (prerequisite) | Record real drops, hard landings, bumps, harness handling and (ethically sourced / dummy) impacts with the final enclosure and mounting; replace synthetic pulse shapes; re-run the sweeps and freeze new held-out sets | `CrashConfig`, Tests/sim/CrashSimulator.hpp |
| IMU sampling for impacts | The IMUs run at 416 Hz but are polled at 100 Hz: pulses shorter than ~10 ms can be missed or under-read. Options: read the LSM6DSRX FIFO (all samples), use its wake-up / free-fall interrupt engine, or poll faster; anti-alias filter (LPF1/LPF2 bandwidth) must be chosen on purpose | driver config + `ImuChannelInfo` peaks |
| IMU full scale | 16 g full scale saturates in hard impacts (detected and flagged, value = lower bound). Decide whether one IMU should run at a different range | `Lsm6dsrxConfig`, `ProcessingConfig::accelLimit_mps2` |
| Mounting | Body-axis mounting of the IMUs (still placeholders) affects the tilt evidence (`abnormalTilt_rad`) | `AxisAlignment` |
| Countdown durations | 60 s (automatic) / 5 s (manual) are placeholders: decide with pilots, instructors and the rescue organisation; check audibility of the countdown in wind | `EmergencyConfig` |
| Pilot input | Decide the physical cancel / manual-SOS control (button, long press, phone only?); implement a debounced command source calling `EmergencyAlertTask::submitCommand()` | new driver + `CommandSource::HardwareInput` |
| BLE transport | Verify the CYW20822 EZ-Serial command set (HARDWARE_INTEGRATION_TODO.md section 9); implement `ISosTransport` on `EzSerialLink` (non-blocking submit, write-complete events -> `TransportReport`), GATT service for `SosRequest` frames (64 B, CRC-32) | Components/Communications/Bluetooth |
| Phone app protocol | Implement the app side: decode `SosRequest` v1, deduplicate by (device, incidentId), report PhoneAccepted / PhoneSendAttempted / DeliveryConfirmed / DeliveryFailed / CancelAccepted with the request revision; never report "confirmed" for an API success alone | next milestone |
| Persistence | Implement `IEmergencyStore` on the W25Q128 (two alternating records, asynchronous write from a low-priority task, never from SensorProc); verify power-loss behaviour; until then SOS continuation across a reset is NOT possible | Components/Storage |
| Audio | Configure SAI/I2S + DMA for the MAX98357A (pins / SD_MODE / gain from the schematic); implement `IAudioAlert` (tone synthesis, non-blocking); check loudness and tone distinctness in flight noise | Components/Output/Audio |
| LED | Pins, count, colour (RGB?), polarity; implement `IStatusLed` reproducing `LedPatternScheduler::levelAt()` timing; daylight visibility | Components/Output/LED |
| Battery in SOS | Implement `IPowerMonitor`; put battery mV into `SosRequest` (bit `sosvalid::Battery`) | Components/Power |
| Timing / stack | Measure `crash_.update()` and an EmergencyAlert wake with DWT `CYCCNT`; read the stack high-water marks (`g_systemHealth.tasks[]`); the 3 KiB EmergencyAlert stack can likely be reduced after measurement | RTOS_TIMING_ANALYSIS.md 4d |


## 17. BLE and Android companion (Milestone 7)

The emergency task now uses the bounded M7 BLE bridge with an unconfigured production packet port. The Android app has a corresponding GATT client, and both protocol directions were exercised with simulated packets. The device schematic, module image, UART pin/flow-control mapping, GATT database, authenticated pairing and persistent incident namespace are required for physical acceptance. See [BLE hardware integration](BLE_ANDROID_HARDWARE_INTEGRATION.md) for the bench sequence and phone test matrix.

## 18. Persistence, logging, power, outputs, FIFO, watchdog (Milestone 8)

The portable code is implemented and host-tested. The firmware still runs `UnconfiguredNorFlash`,
`UnconfiguredPowerSwitch` (5 V rail and amplifier), `UnconfiguredAudioAlert` and `UnconfiguredStatusLed`. Battery
thresholds are unset, so the level is Unknown. The LSM6DSRX FIFO is not enabled and the IWDG is not enabled.
Every item needing the schematic or measurements is listed in
[MILESTONE8_HARDWARE_BRINGUP.md](MILESTONE8_HARDWARE_BRINGUP.md):
- flash SPI and chip select, erase and program timing, and a 1,000-cycle power-cut test;
- battery divider, ADC and chemistry thresholds; charger pin polarity;
- regulator and amplifier enable polarity and settle times; SAI/I2S and DMA;
- LED pins; FIFO registers and INT pins;
- IWDG timeout; on-target stacks and timing;
- the phone device matrix; the real provider setup and the safe live test.
