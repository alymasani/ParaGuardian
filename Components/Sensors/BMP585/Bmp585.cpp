/**
 ******************************************************************************
 * @file    Bmp585.cpp
 * @brief   BMP585 driver implementation. Sequence follows the Bosch BMP5
 *          Sensor API (bmp5_init / bmp5_soft_reset / bmp5_set_osr_odr_press_config
 *          / bmp5_set_iir_config / bmp5_set_power_mode / bmp5_get_sensor_data).
 ******************************************************************************
 */
#include "Sensors/BMP585/Bmp585.hpp"

#include "Common/ByteOrder.hpp"
#include "Sensors/BMP585/Bmp585Registers.hpp"

namespace pg {

using namespace bmp585;

float Bmp585::rawToPressurePa(uint32_t raw24)
{
    return static_cast<float>(raw24) / PRESS_LSB_PER_PA;
}

float Bmp585::rawToTemperatureC(int32_t raw24signed)
{
    return static_cast<float>(raw24signed) / TEMP_LSB_PER_DEGC;
}

uint8_t Bmp585::encodeOsrConfig(Bmp585Oversampling p, Bmp585Oversampling t)
{
    return static_cast<uint8_t>(OSR_PRESS_EN | (static_cast<uint8_t>(p) << 3) | static_cast<uint8_t>(t));
}

uint8_t Bmp585::encodeOdrConfig(Bmp585Odr odr, Bmp585PowerMode mode)
{
    // deep_dis = 1: never enter "deep standby" (which would lose the configuration).
    return static_cast<uint8_t>(ODR_DEEP_DISABLE | ((static_cast<uint8_t>(odr) & 0x1Fu) << 2) |
                                (static_cast<uint8_t>(mode) & ODR_PWR_MODE_MASK));
}

uint8_t Bmp585::encodeDspIir(Bmp585IirFilter p, Bmp585IirFilter t)
{
    return static_cast<uint8_t>((static_cast<uint8_t>(p) << 3) | static_cast<uint8_t>(t));
}

Status Bmp585::track(Status s)
{
    if (isOk(s)) {
        health_.recordSuccess(clock_.millis());
    } else {
        health_.recordFailure(s);
    }
    return s;
}

Status Bmp585::readChipId(uint8_t& id)
{
    return bus_.readRegister(REG_CHIP_ID, id);
}

Status Bmp585::setPowerMode(Bmp585PowerMode mode)
{
    return bus_.writeRegister(REG_ODR_CONFIG, encodeOdrConfig(config_.odr, mode));
}

Status Bmp585::init(const Bmp585Config& config)
{
    health_.initialized = false;
    config_ = config;
    Status s;

    // 1. Soft reset, then wait the specified 2 ms.
    if (!isOk(s = bus_.writeRegister(REG_CMD, CMD_SOFT_RESET))) {
        return track(s);
    }
    clock_.delayMs(SOFT_RESET_DELAY_MS);

    // 2. Identify. The first read after reset is a dummy read: on SPI the
    //    first chip-select falling edge switches the chip's interface into SPI
    //    mode, so its result is not trusted. The second read is checked.
    uint8_t chipId = 0u;
    if (!isOk(s = readChipId(chipId)) || !isOk(s = readChipId(chipId))) {
        return track(s);
    }
    if (chipId != CHIP_ID_BMP585) {
        return track(Status::WrongDeviceId);
    }

    // 3. Power-up check: NVM loaded without error, and the POR/soft-reset flag set.
    uint8_t status = 0u;
    if (!isOk(s = bus_.readRegister(REG_STATUS, status))) {
        return track(s);
    }
    if ((status & STATUS_NVM_RDY) == 0u || (status & STATUS_NVM_ERR) != 0u) {
        return track(Status::NotReady);
    }
    uint8_t intStatus = 0u;
    if (!isOk(s = bus_.readRegister(REG_INT_STATUS, intStatus))) {  // also clears it
        return track(s);
    }
    if ((intStatus & INT_STATUS_POR) == 0u) {
        return track(Status::NotReady);
    }

    // 4. Configuration must be written in STANDBY.
    if (!isOk(s = setPowerMode(Bmp585PowerMode::Standby))) {
        return track(s);
    }

    // 5. IIR filter: route filtered values into the data registers, then set coefficients.
    const uint8_t dspConfig = static_cast<uint8_t>(DSP_SHDW_SEL_IIR_T | DSP_SHDW_SEL_IIR_P | DSP_IIR_FLUSH_FORCED_EN);
    const uint8_t dspIir = encodeDspIir(config.pressureIir, config.temperatureIir);
    const uint8_t osr = encodeOsrConfig(config.pressureOsr, config.temperatureOsr);
    if (!isOk(s = bus_.writeRegister(REG_DSP_CONFIG, dspConfig)) ||
        !isOk(s = bus_.writeRegister(REG_DSP_IIR, dspIir)) ||
        !isOk(s = bus_.writeRegister(REG_OSR_CONFIG, osr))) {
        return track(s);
    }

    // 6. Verify the writes.
    uint8_t rbIir = 0u;
    uint8_t rbOsr = 0u;
    if (!isOk(s = bus_.readRegister(REG_DSP_IIR, rbIir)) || !isOk(s = bus_.readRegister(REG_OSR_CONFIG, rbOsr))) {
        return track(s);
    }
    if (rbIir != dspIir || rbOsr != osr) {
        return track(Status::VerifyFailed);
    }

    // 7. Ask the chip whether this OSR/ODR combination fits in the measurement
    //    period. Too much oversampling at a high ODR is rejected (odr_is_valid = 0).
    uint8_t osrEff = 0u;
    if (!isOk(s = bus_.readRegister(REG_OSR_EFF, osrEff))) {
        return track(s);
    }
    if (config.mode == Bmp585PowerMode::Normal && (osrEff & OSR_EFF_ODR_IS_VALID) == 0u) {
        return track(Status::VerifyFailed);
    }

    // 8. Enter the requested mode (Forced is triggered later by measureForced()).
    if (config.mode != Bmp585PowerMode::Forced && config.mode != Bmp585PowerMode::Standby) {
        if (!isOk(s = setPowerMode(config.mode))) {
            return track(s);
        }
        uint8_t rbOdr = 0u;
        if (!isOk(s = bus_.readRegister(REG_ODR_CONFIG, rbOdr))) {
            return track(s);
        }
        if (rbOdr != encodeOdrConfig(config.odr, config.mode)) {
            return track(Status::VerifyFailed);
        }
    }

    health_.initialized = true;
    return track(Status::Ok);
}

Status Bmp585::read(BaroSample& out)
{
    out.header.id = id_;
    out.header.valid = false;
    if (!health_.initialized) {
        return Status::NotInitialized;
    }

    // One 6-byte burst keeps temperature and pressure from the same conversion:
    // [0..2] = TEMP XLSB, LSB, MSB   [3..5] = PRESS XLSB, LSB, MSB
    uint8_t buf[6] = {};
    const Status s = bus_.readRegisters(REG_TEMP_DATA_XLSB, buf, sizeof(buf));
    out.header.timestampMs = clock_.millis();
    if (!isOk(s)) {
        return track(s);
    }

    const int32_t rawT = bytes::le_i24(&buf[0]);
    const uint32_t rawP = bytes::le_u24(&buf[3]);

    // Pressure register stays 0 until the first conversion completes (or press_en=0).
    // All-ones means a floating/disconnected bus.
    if (rawP == 0u || rawP == 0xFFFFFFu) {
        return track(rawP == 0u ? Status::NotReady : Status::InvalidData);
    }

    out.pressure_pa = rawToPressurePa(rawP);
    out.temperature_c = rawToTemperatureC(rawT);
    out.pressureInRange = (out.pressure_pa >= PRESSURE_MIN_PA) && (out.pressure_pa <= PRESSURE_MAX_PA);
    out.header.sequence = ++sequence_;
    out.header.valid = out.pressureInRange;
    return track(out.pressureInRange ? Status::Ok : Status::OutOfRange);
}

Status Bmp585::measureForced(BaroSample& out, uint32_t timeoutMs)
{
    if (!health_.initialized) {
        return Status::NotInitialized;
    }
    Status s = setPowerMode(Bmp585PowerMode::Forced);
    if (!isOk(s)) {
        return track(s);
    }
    // The chip performs one conversion and goes back to standby by itself.
    const uint32_t start = clock_.millis();
    for (;;) {
        clock_.delayMs(1u);
        uint8_t odr = 0u;
        if (!isOk(s = bus_.readRegister(REG_ODR_CONFIG, odr))) {
            return track(s);
        }
        if ((odr & ODR_PWR_MODE_MASK) == static_cast<uint8_t>(Bmp585PowerMode::Standby)) {
            break;
        }
        if ((clock_.millis() - start) >= timeoutMs) {
            return track(Status::Timeout);
        }
    }
    return read(out);
}

}  // namespace pg
