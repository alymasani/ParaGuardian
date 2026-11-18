/**
 ******************************************************************************
 * @file    Mmc5983ma.cpp
 * @brief   MMC5983MA driver implementation. See Mmc5983ma.hpp.
 ******************************************************************************
 */
#include "Sensors/MMC5983MA/Mmc5983ma.hpp"

namespace pg {

using namespace mmc5983ma;

Mmc5983maRaw Mmc5983ma::decode18Bit(const uint8_t regs[7])
{
    // Register 0x06 (XYZout2) holds the two least-significant bits of each axis:
    //   bits 7:6 = X[1:0]   bits 5:4 = Y[1:0]   bits 3:2 = Z[1:0]
    // Full value = MSB(8 bits) << 10 | middle(8 bits) << 2 | low(2 bits)
    Mmc5983maRaw raw{};
    raw.x = (static_cast<uint32_t>(regs[0]) << 10) | (static_cast<uint32_t>(regs[1]) << 2) |
            ((static_cast<uint32_t>(regs[6]) >> 6) & 0x3u);
    raw.y = (static_cast<uint32_t>(regs[2]) << 10) | (static_cast<uint32_t>(regs[3]) << 2) |
            ((static_cast<uint32_t>(regs[6]) >> 4) & 0x3u);
    raw.z = (static_cast<uint32_t>(regs[4]) << 10) | (static_cast<uint32_t>(regs[5]) << 2) |
            ((static_cast<uint32_t>(regs[6]) >> 2) & 0x3u);
    return raw;
}

float Mmc5983ma::countsToMicroTesla(uint32_t counts18)
{
    // Offset binary: subtract the null-field count to get a signed value.
    const float signedCounts = static_cast<float>(static_cast<int32_t>(counts18) - static_cast<int32_t>(NULL_FIELD_18BIT));
    return (signedCounts / COUNTS_PER_GAUSS_18BIT) * units::kGaussToMicroTesla;
}

bool Mmc5983ma::isSaturated(uint32_t counts18)
{
    return counts18 == 0u || counts18 >= MAX_18BIT;
}

float Mmc5983ma::rawToTemperatureC(uint8_t raw)
{
    return TEMP_OFFSET_DEGC + TEMP_DEGC_PER_LSB * static_cast<float>(raw);
}

Status Mmc5983ma::track(Status s)
{
    if (isOk(s)) {
        health_.recordSuccess(clock_.millis());
    } else {
        health_.recordFailure(s);
    }
    return s;
}

Status Mmc5983ma::readProductId(uint8_t& id)
{
    return bus_.readRegister(REG_PRODUCT_ID, id);
}

uint32_t Mmc5983ma::measurementTimeoutMs() const
{
    // Nominal measurement time per bandwidth, times ~2, plus 2 ms margin.
    switch (config_.bandwidth) {
        case Mmc5983maBandwidth::Hz100: return 18u;
        case Mmc5983maBandwidth::Hz200: return 10u;
        case Mmc5983maBandwidth::Hz400: return 6u;
        case Mmc5983maBandwidth::Hz800: return 3u;
    }
    return 18u;
}

Status Mmc5983ma::init(const Mmc5983maConfig& config)
{
    health_.initialized = false;
    Status s;

    // 1. Software reset (all registers to defaults), then wait the power-on time.
    if (!isOk(s = bus_.writeRegister(REG_CTRL1, CTRL1_SW_RST))) {
        return track(s);
    }
    clock_.delayMs(POWER_ON_TIME_MS);

    // 2. Identify.
    uint8_t productId = 0u;
    if (!isOk(s = readProductId(productId))) {
        return track(s);
    }
    if (productId != PRODUCT_ID_VALUE) {
        return track(Status::WrongDeviceId);
    }

    // 3. Continuous 200/1000 Hz are only allowed with sufficiently high bandwidth.
    if ((config.continuousRate == Mmc5983maContinuousRate::Hz200 && config.bandwidth == Mmc5983maBandwidth::Hz100) ||
        (config.continuousRate == Mmc5983maContinuousRate::Hz1000 && config.bandwidth != Mmc5983maBandwidth::Hz800)) {
        return track(Status::InvalidArgument);
    }
    config_ = config;
    ctrl0Persistent_ = config.autoSetReset ? CTRL0_AUTO_SR_EN : 0u;

    // 4. Bandwidth (write-only register, cannot be verified).
    if (!isOk(s = bus_.writeRegister(REG_CTRL1, static_cast<uint8_t>(config.bandwidth)))) {
        return track(s);
    }

    // 5. One SET pulse to put the sensing elements into a known magnetisation.
    if (!isOk(s = pulseSet())) {
        return track(s);
    }

    // 6. Persistent CTRL0 bits (Auto_SR_en) and optional continuous mode.
    if (!isOk(s = bus_.writeRegister(REG_CTRL0, ctrl0Persistent_))) {
        return track(s);
    }
    if (config.continuousRate != Mmc5983maContinuousRate::Off) {
        const uint8_t ctrl2 = static_cast<uint8_t>(CTRL2_CMM_EN | static_cast<uint8_t>(config.continuousRate));
        if (!isOk(s = bus_.writeRegister(REG_CTRL2, ctrl2))) {
            return track(s);
        }
    }

    health_.initialized = true;
    return track(Status::Ok);
}

Status Mmc5983ma::pulseSet()
{
    const Status s = bus_.writeRegister(REG_CTRL0, static_cast<uint8_t>(ctrl0Persistent_ | CTRL0_SET));
    clock_.delayMs(SET_RESET_TIME_MS);
    return s;
}

Status Mmc5983ma::pulseReset()
{
    const Status s = bus_.writeRegister(REG_CTRL0, static_cast<uint8_t>(ctrl0Persistent_ | CTRL0_RESET));
    clock_.delayMs(SET_RESET_TIME_MS);
    return s;
}

Status Mmc5983ma::triggerAndWait(uint8_t triggerBit, uint8_t doneBit, uint32_t timeoutMs)
{
    // Clear a stale "done" flag (write-1-to-clear), then start the measurement.
    Status s = bus_.writeRegister(REG_STATUS, doneBit);
    if (!isOk(s)) {
        return s;
    }
    if (!isOk(s = bus_.writeRegister(REG_CTRL0, static_cast<uint8_t>(ctrl0Persistent_ | triggerBit)))) {
        return s;
    }
    const uint32_t start = clock_.millis();
    for (;;) {
        uint8_t status = 0u;
        if (!isOk(s = bus_.readRegister(REG_STATUS, status))) {
            return s;
        }
        if ((status & doneBit) != 0u) {
            return Status::Ok;
        }
        if ((clock_.millis() - start) >= timeoutMs) {
            return Status::Timeout;
        }
        clock_.delayMs(1u);
    }
}

Status Mmc5983ma::readRawBlock(Mmc5983maRaw& raw)
{
    uint8_t regs[7] = {};
    const Status s = bus_.readRegisters(REG_XOUT0, regs, sizeof(regs));
    if (!isOk(s)) {
        return s;
    }
    bool allOnes = true;
    for (uint8_t b : regs) {
        allOnes = allOnes && (b == 0xFFu);
    }
    if (allOnes) {
        return Status::InvalidData;  // floating bus
    }
    raw = decode18Bit(regs);
    return Status::Ok;
}

void Mmc5983ma::fillSample(MagSample& out, const Mmc5983maRaw& raw, bool offsetCancelled)
{
    out.header.timestampMs = clock_.millis();
    out.field_uT = {countsToMicroTesla(raw.x), countsToMicroTesla(raw.y), countsToMicroTesla(raw.z)};
    out.saturated = isSaturated(raw.x) || isSaturated(raw.y) || isSaturated(raw.z);
    out.offsetCancelled = offsetCancelled;
    out.header.sequence = ++sequence_;
    out.header.valid = !out.saturated;
}

Status Mmc5983ma::measure(MagSample& out)
{
    out.header.id = id_;
    out.header.valid = false;
    if (!health_.initialized) {
        return Status::NotInitialized;
    }
    Status s = triggerAndWait(CTRL0_TM_M, STATUS_MEAS_M_DONE, measurementTimeoutMs());
    if (!isOk(s)) {
        return track(s);
    }
    Mmc5983maRaw raw{};
    if (!isOk(s = readRawBlock(raw))) {
        return track(s);
    }
    fillSample(out, raw, false);
    return track(Status::Ok);
}

Status Mmc5983ma::readLatest(MagSample& out)
{
    out.header.id = id_;
    out.header.valid = false;
    if (!health_.initialized) {
        return Status::NotInitialized;
    }
    uint8_t status = 0u;
    Status s = bus_.readRegister(REG_STATUS, status);
    if (!isOk(s)) {
        return track(s);
    }
    if ((status & STATUS_MEAS_M_DONE) == 0u) {
        return Status::NotReady;  // not a fault: no new data yet
    }
    Mmc5983maRaw raw{};
    if (!isOk(s = readRawBlock(raw))) {
        return track(s);
    }
    if (!isOk(s = bus_.writeRegister(REG_STATUS, STATUS_MEAS_M_DONE))) {
        return track(s);
    }
    fillSample(out, raw, false);
    return track(Status::Ok);
}

Status Mmc5983ma::startMeasurement()
{
    if (!health_.initialized) {
        return Status::NotInitialized;
    }
    if (measurementPending_) {
        return Status::Busy;
    }
    Status s = bus_.writeRegister(REG_STATUS, STATUS_MEAS_M_DONE);  // clear stale flag (write 1 to clear)
    if (isOk(s)) {
        s = bus_.writeRegister(REG_CTRL0, static_cast<uint8_t>(ctrl0Persistent_ | CTRL0_TM_M));
    }
    if (!isOk(s)) {
        return track(s);
    }
    measurementPending_ = true;
    measurementStartMs_ = clock_.millis();
    return Status::Ok;
}

Status Mmc5983ma::readMeasurementIfReady(MagSample& out)
{
    out.header.id = id_;
    out.header.valid = false;
    if (!health_.initialized) {
        return Status::NotInitialized;
    }
    if (!measurementPending_) {
        return Status::NotReady;
    }
    uint8_t status = 0u;
    Status s = bus_.readRegister(REG_STATUS, status);
    if (!isOk(s)) {
        measurementPending_ = false;
        return track(s);
    }
    if ((status & STATUS_MEAS_M_DONE) == 0u) {
        if ((clock_.millis() - measurementStartMs_) >= measurementTimeoutMs()) {
            measurementPending_ = false;
            return track(Status::Timeout);
        }
        return Status::NotReady;  // still converting: not a fault
    }
    measurementPending_ = false;
    Mmc5983maRaw raw{};
    if (!isOk(s = readRawBlock(raw))) {
        return track(s);
    }
    fillSample(out, raw, false);
    return track(Status::Ok);
}

Status Mmc5983ma::measureOffsetCancelled(MagSample& out)
{
    out.header.id = id_;
    out.header.valid = false;
    if (!health_.initialized) {
        return Status::NotInitialized;
    }
    if (config_.continuousRate != Mmc5983maContinuousRate::Off) {
        return Status::Unsupported;
    }
    // Auto_SR must be off during this sequence, otherwise the chip would SET
    // again before the "RESET" measurement. Temporarily clear it.
    const uint8_t savedPersistent = ctrl0Persistent_;
    ctrl0Persistent_ = static_cast<uint8_t>(ctrl0Persistent_ & ~CTRL0_AUTO_SR_EN);

    Mmc5983maRaw afterSet{};
    Mmc5983maRaw afterReset{};
    Status s = pulseSet();
    if (isOk(s)) s = triggerAndWait(CTRL0_TM_M, STATUS_MEAS_M_DONE, measurementTimeoutMs());
    if (isOk(s)) s = readRawBlock(afterSet);
    if (isOk(s)) s = pulseReset();
    if (isOk(s)) s = triggerAndWait(CTRL0_TM_M, STATUS_MEAS_M_DONE, measurementTimeoutMs());
    if (isOk(s)) s = readRawBlock(afterReset);
    if (isOk(s)) s = pulseSet();  // leave the sensor in the SET state again

    // Restore the persistent bits in the shadow AND in the chip: CTRL0 is
    // write-only and the last write above (SET pulse) carried Auto_SR_en = 0.
    ctrl0Persistent_ = savedPersistent;
    const Status restore = bus_.writeRegister(REG_CTRL0, ctrl0Persistent_);
    if (isOk(s)) {
        s = restore;
    }
    if (!isOk(s)) {
        return track(s);
    }

    // After SET the output is (+H + offset); after RESET it is (-H + offset).
    // H = (Mset - Mreset) / 2, expressed back as an 18-bit offset-binary count.
    Mmc5983maRaw field{};
    auto half = [](uint32_t a, uint32_t b) {
        const int32_t diff = static_cast<int32_t>(a) - static_cast<int32_t>(b);
        return static_cast<uint32_t>(static_cast<int32_t>(NULL_FIELD_18BIT) + diff / 2);
    };
    field.x = half(afterSet.x, afterReset.x);
    field.y = half(afterSet.y, afterReset.y);
    field.z = half(afterSet.z, afterReset.z);
    fillSample(out, field, true);
    // Saturation must be judged on the raw readings, not on the difference.
    out.saturated = isSaturated(afterSet.x) || isSaturated(afterSet.y) || isSaturated(afterSet.z) ||
                    isSaturated(afterReset.x) || isSaturated(afterReset.y) || isSaturated(afterReset.z);
    out.header.valid = !out.saturated;
    return track(Status::Ok);
}

Status Mmc5983ma::measureTemperature(float& temperatureC)
{
    if (!health_.initialized) {
        return Status::NotInitialized;
    }
    Status s = triggerAndWait(CTRL0_TM_T, STATUS_MEAS_T_DONE, measurementTimeoutMs());
    if (!isOk(s)) {
        return track(s);
    }
    uint8_t raw = 0u;
    if (!isOk(s = bus_.readRegister(REG_TOUT, raw))) {
        return track(s);
    }
    temperatureC = rawToTemperatureC(raw);
    return track(Status::Ok);
}

}  // namespace pg
