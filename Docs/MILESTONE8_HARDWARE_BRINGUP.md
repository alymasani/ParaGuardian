# Milestone 8 – Hardware Bring-up Checklist

Everything here depends on the schematic, the CubeMX configuration or measurements on the PCB. **Nothing in this list has been done.** Pins, polarities and timings are deliberately not written down until the schematic exists. Each item names the code that waits for it.

## 1. External flash (W25Q128JV)
- [ ] From the schematic: SPI instance, SCK/MISO/MOSI pins, chip select pin (and whether CS is active low through a buffer), maximum SPI clock for the trace length, and whether DMA is available.
- [ ] CubeMX: enable the SPI peripheral and regenerate. In `BoardConfig.cpp`, create `HalSpiDevice` → `W25q128` → `W25qNorFlash` and pass it to `StorageTask` in place of `UnconfiguredNorFlash` (`Components/Tasks/StorageTask.hpp`).
- [ ] `W25q128::init()` reads the JEDEC ID `EF 40 18` (or `EF 70 18`). Check the status register protect bits (`SR1_PROTECT_MASK`) are clear.
- [ ] **Measure** page-program and 4 KiB erase times with a GPIO toggle or logic analyser. Compare with the datasheet (tPP typ 0.4 ms / max 3 ms; tSE typ 45 ms / max 400 ms). Record the worst storage unit in `g_systemHealth.storageMaxUnitMs`.
- [ ] Partition check: after first boot, dump 0x010000–0x011FFF (journal) and 0x012000–0x01FFFF (critical log). Decode with `python3 Tools/pglog/pglog_decode.py dump.bin --region --json`. Confirm 0x000000–0x00FFFF is untouched.
- [ ] Power-cut test on silicon: cut the supply with a relay at random times during repeated commits (≥ 1,000 cycles). After each boot, check that `journalMount == Recovered` and `nextIncidentId` never decreases.

## 2. Battery sensing
- [ ] From the schematic: battery chemistry, cell count, divider ratio, ADC channel, reference voltage, and whether the divider can be switched off (leakage).
- [ ] Write the board adapter that produces `PowerInputs` (`Components/Power/PowerManager.hpp`): millivolts, validity range, sample time.
- [ ] Characterise the discharge curve under the real load. Only then set `BatteryConfig::lowMv` / `criticalMv` (now 0 = unconfigured). Do not derive a state of charge from voltage without this.
- [ ] If a fuel gauge exists: provide `gaugePercent` / `gaugeValid`.

## 3. Charger status
- [ ] Charger IC part number, charging and charge-complete pin polarity (open drain?), pull-ups, and the meaning while no battery is present.
- [ ] Map them to `Signal::Active` / `Inactive` in the adapter. Test that "both active" is reported as `ChargeState::Fault`, and that "no supply" gives `Disconnected`.

## 4. 5 V regulator enable
- [ ] Enable pin and polarity, default state at reset (pull resistor), any power-good output.
- [ ] Implement an `IPowerSwitch` and replace `UnconfiguredPowerSwitch rail5vSwitch_` in `EmergencyAlertTask.hpp`.
- [ ] **Measure** the rail rise time with the LiDAR load. Set `RailConfig::settleMs` to the measurement plus margin (placeholder 20 ms).
- [ ] Measure the LiDAR's own warm-up before its readings are valid. Confirm the acquisition source handles the gap.

## 5. Audio amplifier (MAX98357A)
- [ ] SD_MODE / enable pin and polarity, gain pin strapping, which supply powers the amplifier (5 V rail?), speaker impedance.
- [ ] Measure the turn-on and turn-off times and pops. Set the amplifier `settleMs` and `PowerPolicyConfig::ampHoldMs` from them.
- [ ] SAI/I2S instance, pins (BCLK, LRCLK, DIN), DMA stream, and the clock tree for the chosen sample rate (16 kHz in `PcmToneConfig`).
- [ ] Write the HAL backend: a circular DMA double buffer of `int16_t[2][256]` (1 KiB). In the half-complete and complete callbacks call `PcmToneOutput::render()`. Feed `setAmplifierReady(PowerSnapshot::audioReady)` and `service()` from the emergency task. Replace `UnconfiguredAudioAlert`.
- [ ] Measure the sound pressure of each alert pattern at 1 m. Adjust `maxAmplitude` and the volume table to a safe, audible level.

## 6. LEDs
- [ ] Count, colours, pins and polarity (sink or source). Implement `IDigitalOutput` per pin and replace `UnconfiguredStatusLed` with `GpioStatusLed`.
- [ ] Check every pattern visually against `LedPatternScheduler::levelAt`.

## 7. CYW20822 / BLE (from M7, still open)
- [ ] Provisioning, authenticated LE Secure Connections association, a `PhysicalTrust` implementation (Android) that returns a verified peer. Without it, physical incidents never reach the phone engine.
- [ ] After association, check that `PresenceService` is bound when the device comes into range (Android 12+). Check that START_STICKY, BOOT_COMPLETED and presence restarts happen on real phones (§11).

## 8. LiDAR mounting
- [ ] Verify beam axis versus the configured axis (an M5 open item). Verify that power-cycling the 5 V rail recovers the sensor.

## 9. Device identity and re-provisioning
- [ ] Decide the device identity used by Android and the backend, for example a serial number in the reserved flash region 0x000000–0x00FFFF or in the MCU's OTP.
- [ ] Procedure after a flash replacement or wipe: provision a **new** device id on the backend (`python3 -m paraguardian_sos provision-device <id>`) so restarted incident ids cannot collide with history. The firmware flags `IdentityUncertain` only when it sees corrupt data. It cannot detect a cleanly wiped chip.

## 10. IMU FIFO / data-ready
- [ ] Confirm the FIFO register addresses and bit fields in `Lsm6dsrxFifo.hpp` against the exact LSM6DSRX datasheet revision: FIFO_CTRL3/4, FIFO_STATUS1/2, FIFO_DATA_OUT_TAG and the tag codes.
- [ ] Confirm whether INT1/INT2 are wired to EXTI pins (watermark interrupt), and confirm the SPI bandwidth for 3 IMUs × ~9 words × 7 bytes per 10 ms.
- [ ] Enable it in `ImuSource`: configure after `init()`, drain each poll, and feed the batch peak into the per-IMU peak fields used by `CrashDetector`. Keep the polled sample for everything else.
- [ ] **Measure** with a drop rig: the impact peak from FIFO vs polling. Only then claim an improvement.

## 11. Hardware watchdog (IWDG)
- [ ] Measure the boot time to the first health collection. Choose an IWDG timeout greater than the supervisor's worst detection time (2 s + 1 s sampling + the 5 s emergency deferral).
- [ ] Feed the IWDG from `HealthMonitorTask` **only** when `TaskSupervisor::feedWatchdog()` is true.
- [ ] Check that `RCC->RSR` IWDG1RSTF is captured (`g_systemHealth.resetFlags`), and that after a watchdog reset an open incident is restored from the journal (the emergency state must already have been committed).

## 12. On-target measurements
- [ ] Stack high-water marks of all 5 tasks after a long run, including a flight-log rotation and an SOS (`g_systemHealth.tasks[]`). The storage task is sized at 3 KiB with an estimated need below 1 KB.
- [ ] CPU time per task: DWT cycle counter around `StorageService::step`, `FlightRecorder` calls and `PowerManager::update`.
- [ ] Measure the queue high-water marks (log queue drops in `g_systemHealth.logQueueDrops*`).

## 13. Android physical-device matrix
At least: Android 10 (API 29), 12 (31), 13 (33), 14 (34), 15 (35), 16 (36) on two vendors with aggressive battery management (for example Samsung and Xiaomi). For each, test:
- monitoring after process kill (`adb shell am kill`);
- swipe from Recents;
- reboot;
- Bluetooth toggle;
- permission revocation;
- battery saver;
- force stop (expect "user action required").

## 14. Real delivery provider (only when deliberately decided)
1. Create a provider account (for example Twilio), buy or verify a sender number, and register it as required for the destination countries (A2P / sender ID rules).
2. Deploy the backend behind TLS on a host you control:
   - `PG_TLS_CERT` / `PG_TLS_KEY`, or a TLS-terminating proxy;
   - `PG_PROVIDER=twilio`, `TWILIO_ACCOUNT_SID`, `TWILIO_AUTH_TOKEN`, `TWILIO_FROM` and `PG_PUBLIC_BASE_URL`, all from a secrets manager.
3. Keep `PG_LIVE_DELIVERY_ENABLED=false` until the end-to-end test below passes with the fake provider on the same deployment.
4. Provision each device and enter its id and secret in the app (Settings → Unattended emergency delivery).
5. Set `PG_LIVE_DELIVERY_ENABLED=true` only for the controlled test in §15.

## 15. Safe end-to-end emergency test procedure
1. Use only consenting test contacts (your own phones). Inform them beforehand. **Never** use emergency service numbers. Mark incidents as tests: simulation incidents are always `test=true` and never reach a live provider.
2. Run 1, fake provider: crash simulation → countdown → SOS → reset the PCB mid-send → recovery with the same incident id → BLE → app → backend → fake provider → simulated delivery callback → status back to the device.
3. Run 2, live provider, test contacts only: a manual SOS from the device. Check provider-accepted vs delivered reporting and that no duplicate SMS arrives after a forced app kill and a phone reboot mid-submission.
4. Cancel-after-send: check that the cancellation notice reaches exactly the contacts who received the alert.
5. Afterwards set `PG_LIVE_DELIVERY_ENABLED=false` and purge the test data (`python3 -m paraguardian_sos purge`).
