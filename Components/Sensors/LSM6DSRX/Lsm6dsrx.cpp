/**
 ******************************************************************************
 * @file    Lsm6dsrx.cpp
 * @brief   LSM6DSRX driver implementation. See Lsm6dsrx.hpp.
 ******************************************************************************
 */
#include "Sensors/LSM6DSRX/Lsm6dsrx.hpp"

#include "Common/ByteOrder.hpp"
#include "Sensors/LSM6DSRX/Lsm6dsrxRegisters.hpp"

namespace pg {

using namespace lsm6dsrx;

// ---------------------------------------------------------------------------
// Conversion helpers (values from datasheet table "Mechanical characteristics")
// ---------------------------------------------------------------------------
float Lsm6dsrx::accelSensitivity_mg_per_lsb(Lsm6dsrxAccelRange range)
{
    switch (range) {
        case Lsm6dsrxAccelRange::G2:  return 0.061f;
        case Lsm6dsrxAccelRange::G4:  return 0.122f;
        case Lsm6dsrxAccelRange::G8:  return 0.244f;
        case Lsm6dsrxAccelRange::G16: return 0.488f;
    }
    return 0.0f;
}

float Lsm6dsrx::gyroSensitivity_mdps_per_lsb(Lsm6dsrxGyroRange range)
{
    switch (range) {
        case Lsm6dsrxGyroRange::Dps125:  return 4.375f;
        case Lsm6dsrxGyroRange::Dps250:  return 8.75f;
        case Lsm6dsrxGyroRange::Dps500:  return 17.50f;
        case Lsm6dsrxGyroRange::Dps1000: return 35.0f;
        case Lsm6dsrxGyroRange::Dps2000: return 70.0f;
        case Lsm6dsrxGyroRange::Dps4000: return 140.0f;
    }
    return 0.0f;
}

float Lsm6dsrx::rawToTemperatureC(int16_t raw)
{
    return static_cast<float>(raw) / TEMP_LSB_PER_DEGC + TEMP_OFFSET_DEGC;
}

uint8_t Lsm6dsrx::encodeCtrl1Xl(Lsm6dsrxOdr odr, Lsm6dsrxAccelRange range)
{
    // ODR_XL in bits 7..4, FS_XL in bits 3..2, LPF2_XL_EN (bit 1) left 0.
    return static_cast<uint8_t>((static_cast<uint8_t>(odr) << 4) | (static_cast<uint8_t>(range) << 2));
}

uint8_t Lsm6dsrx::encodeCtrl2G(Lsm6dsrxOdr odr, Lsm6dsrxGyroRange range)
{
    uint8_t fsBits = 0u;
    switch (range) {
        case Lsm6dsrxGyroRange::Dps125:  fsBits = CTRL2_FS_125; break;         // FS_125 = 1
        case Lsm6dsrxGyroRange::Dps250:  fsBits = 0x0u << 2; break;            // FS_G = 00
        case Lsm6dsrxGyroRange::Dps500:  fsBits = 0x1u << 2; break;            // FS_G = 01
        case Lsm6dsrxGyroRange::Dps1000: fsBits = 0x2u << 2; break;            // FS_G = 10
        case Lsm6dsrxGyroRange::Dps2000: fsBits = 0x3u << 2; break;            // FS_G = 11
        case Lsm6dsrxGyroRange::Dps4000: fsBits = CTRL2_FS_4000; break;        // FS_4000 = 1
    }
    return static_cast<uint8_t>((static_cast<uint8_t>(odr) << 4) | fsBits);
}

// ---------------------------------------------------------------------------
Status Lsm6dsrx::track(Status s)
{
    if (isOk(s)) {
        health_.recordSuccess(clock_.millis());
    } else {
        health_.recordFailure(s);
    }
    return s;
}

Status Lsm6dsrx::readWhoAmI(uint8_t& value)
{
    return bus_.readRegister(REG_WHO_AM_I, value);
}

Status Lsm6dsrx::readStatus(uint8_t& status)
{
    return bus_.readRegister(REG_STATUS, status);
}

Status Lsm6dsrx::init(const Lsm6dsrxConfig& config)
{
    health_.initialized = false;

    // 1. Identify. A wrong value usually means wrong chip-select, wrong SPI
    //    mode (must be mode 3 or mode 0), a dead/absent part, or a floating MISO.
    uint8_t who = 0u;
    Status s = readWhoAmI(who);
    if (!isOk(s)) {
        return track(s);
    }
    if (who != WHO_AM_I_VALUE) {
        return track(Status::WrongDeviceId);
    }

    // 2. Software reset: restores every control register to its default.
    //    SW_RESET is self-clearing; poll until the chip clears it.
    s = bus_.writeRegister(REG_CTRL3_C, CTRL3_SW_RESET);
    if (!isOk(s)) {
        return track(s);
    }
    const uint32_t start = clock_.millis();
    for (;;) {
        clock_.delayMs(1u);
        uint8_t ctrl3 = 0u;
        s = bus_.readRegister(REG_CTRL3_C, ctrl3);
        if (!isOk(s)) {
            return track(s);
        }
        if ((ctrl3 & CTRL3_SW_RESET) == 0u) {
            break;
        }
        if ((clock_.millis() - start) >= kResetTimeoutMs) {
            return track(Status::Timeout);
        }
    }

    // 3. Block Data Update + address auto-increment (needed for the 14-byte burst).
    const uint8_t ctrl3 = static_cast<uint8_t>(CTRL3_BDU | CTRL3_IF_INC);
    const uint8_t ctrl1 = encodeCtrl1Xl(config.accelOdr, config.accelRange);
    const uint8_t ctrl2 = encodeCtrl2G(config.gyroOdr, config.gyroRange);

    if (!isOk(s = bus_.writeRegister(REG_CTRL3_C, ctrl3)) ||
        !isOk(s = bus_.writeRegister(REG_CTRL1_XL, ctrl1)) ||
        !isOk(s = bus_.writeRegister(REG_CTRL2_G, ctrl2))) {
        return track(s);
    }

    // 4. Verify: read the three registers back in one burst (0x10, 0x11, 0x12).
    uint8_t readBack[3] = {};
    s = bus_.readRegisters(REG_CTRL1_XL, readBack, sizeof(readBack));
    if (!isOk(s)) {
        return track(s);
    }
    if (readBack[0] != ctrl1 || readBack[1] != ctrl2 || readBack[2] != ctrl3) {
        return track(Status::VerifyFailed);
    }

    // 5. Pre-compute the scale factors once so read() is just multiplications.
    config_ = config;
    accelScale_mps2_per_lsb_ =
        accelSensitivity_mg_per_lsb(config.accelRange) * 1e-3f * units::kStandardGravity_mps2;
    gyroScale_radps_per_lsb_ =
        gyroSensitivity_mdps_per_lsb(config.gyroRange) * 1e-3f * units::kDegToRad;

    health_.initialized = true;
    return track(Status::Ok);
}

Status Lsm6dsrx::readRaw(Lsm6dsrxRaw& raw)
{
    uint8_t status = 0u;
    Status s = readStatus(status);
    if (!isOk(s)) {
        return s;
    }

    uint8_t buf[OUTPUT_BLOCK_LEN] = {};
    s = bus_.readRegisters(REG_OUT_TEMP_L, buf, sizeof(buf));
    if (!isOk(s)) {
        return s;
    }

    // A disconnected/floating MISO typically reads all 0x00 or all 0xFF.
    // A real still sensor never gives exactly 0 on every axis AND temperature
    // (gravity alone puts ~1 g on some axis), so treat that as invalid data.
    bool allZero = true;
    bool allOnes = true;
    for (uint8_t b : buf) {
        allZero = allZero && (b == 0x00u);
        allOnes = allOnes && (b == 0xFFu);
    }
    if (allZero || allOnes) {
        return Status::InvalidData;
    }

    // Byte layout of the burst (all little-endian int16):
    //   [0,1] TEMP   [2,3] GX  [4,5] GY  [6,7] GZ   [8,9] AX  [10,11] AY  [12,13] AZ
    raw.status = status;
    raw.temperature = bytes::le_i16(&buf[0]);
    for (int axis = 0; axis < 3; ++axis) {
        raw.gyro[axis] = bytes::le_i16(&buf[2 + 2 * axis]);
        raw.accel[axis] = bytes::le_i16(&buf[8 + 2 * axis]);
    }
    return Status::Ok;
}

Status Lsm6dsrx::read(ImuSample& out)
{
    out.header.id = id_;
    out.header.valid = false;

    if (!health_.initialized) {
        return Status::NotInitialized;
    }

    Lsm6dsrxRaw raw{};
    const Status s = readRaw(raw);
    out.header.timestampMs = clock_.millis();
    if (!isOk(s)) {
        return track(s);
    }

    auto isSaturated = [](int16_t v) { return v == INT16_MAX || v == INT16_MIN; };

    out.accel_mps2 = {raw.accel[0] * accelScale_mps2_per_lsb_, raw.accel[1] * accelScale_mps2_per_lsb_,
                      raw.accel[2] * accelScale_mps2_per_lsb_};
    out.gyro_radps = {raw.gyro[0] * gyroScale_radps_per_lsb_, raw.gyro[1] * gyroScale_radps_per_lsb_,
                      raw.gyro[2] * gyroScale_radps_per_lsb_};
    out.temperature_c = rawToTemperatureC(raw.temperature);
    out.accelFresh = (raw.status & STATUS_XLDA) != 0u;
    out.gyroFresh = (raw.status & STATUS_GDA) != 0u;
    out.accelSaturated = isSaturated(raw.accel[0]) || isSaturated(raw.accel[1]) || isSaturated(raw.accel[2]);
    out.gyroSaturated = isSaturated(raw.gyro[0]) || isSaturated(raw.gyro[1]) || isSaturated(raw.gyro[2]);
    out.header.sequence = ++sequence_;
    out.header.valid = true;
    return track(Status::Ok);
}

}  // namespace pg
