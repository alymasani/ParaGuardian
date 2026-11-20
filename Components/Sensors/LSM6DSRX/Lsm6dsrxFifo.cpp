/**
 ******************************************************************************
 * @file    Lsm6dsrxFifo.cpp
 * @brief   See Lsm6dsrxFifo.hpp.
 ******************************************************************************
 */
#include "Sensors/LSM6DSRX/Lsm6dsrxFifo.hpp"

#include <cmath>
#include <limits>

namespace pg {

FifoWord decodeFifoWord(const uint8_t* w)
{
    FifoWord f{};
    f.rawTag = static_cast<uint8_t>(w[0] >> 3);
    switch (f.rawTag) {
        case lsm6fifo::TAG_GYRO: f.tag = FifoTag::Gyro; break;
        case lsm6fifo::TAG_ACCEL: f.tag = FifoTag::Accel; break;
        case lsm6fifo::TAG_TEMPERATURE: f.tag = FifoTag::Temperature; break;
        case lsm6fifo::TAG_TIMESTAMP: f.tag = FifoTag::Timestamp; break;
        case lsm6fifo::TAG_CONFIG_CHANGE: f.tag = FifoTag::ConfigChange; break;
        default: f.tag = FifoTag::Unknown; break;
    }
    f.x = static_cast<int16_t>(static_cast<uint16_t>(w[1]) | (static_cast<uint16_t>(w[2]) << 8));
    f.y = static_cast<int16_t>(static_cast<uint16_t>(w[3]) | (static_cast<uint16_t>(w[4]) << 8));
    f.z = static_cast<int16_t>(static_cast<uint16_t>(w[5]) | (static_cast<uint16_t>(w[6]) << 8));
    return f;
}

Status Lsm6dsrxFifo::configure(const FifoConfig& cfg)
{
    if (cfg.maxWordsPerDrain == 0u) return Status::InvalidArgument;
    maxWords_ = cfg.maxWordsPerDrain;
    const uint32_t gy = static_cast<uint32_t>(cfg.gyroBatchRate) & 0x0Fu;
    const uint32_t xl = static_cast<uint32_t>(cfg.accelBatchRate) & 0x0Fu;
    const uint8_t ctrl3 = static_cast<uint8_t>((gy << 4) | xl);   // BDR_GY[7:4] | BDR_XL[3:0]
    Status s = bus_.writeRegister(lsm6fifo::REG_FIFO_CTRL3, ctrl3);
    if (isOk(s)) s = bus_.writeRegister(lsm6fifo::REG_FIFO_CTRL4, lsm6fifo::MODE_CONTINUOUS);
    configured_ = isOk(s);
    return s;
}

Status Lsm6dsrxFifo::disable()
{
    configured_ = false;
    return bus_.writeRegister(lsm6fifo::REG_FIFO_CTRL4, lsm6fifo::MODE_BYPASS);
}

Status Lsm6dsrxFifo::drain(uint32_t nowMs, FifoBatch& out)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    out = FifoBatch{};
    out.timestampMs = nowMs;
    out.peakAccel_mps2 = out.peakRate_radps = nan;
    if (!configured_) return Status::NotInitialized;
    uint8_t st[2] = {};
    Status s = bus_.readRegisters(lsm6fifo::REG_FIFO_STATUS1, st, sizeof(st));
    if (!isOk(s)) return s;
    const uint16_t unread = static_cast<uint16_t>(st[0] | ((st[1] & 0x03u) << 8));
    out.overflow = (st[1] & (lsm6fifo::STATUS2_OVR | lsm6fifo::STATUS2_OVR_LATCHED)) != 0u;
    if (out.overflow && overflows_ != UINT32_MAX) ++overflows_;
    const uint16_t n = unread < maxWords_ ? unread : maxWords_;
    float peakA2 = -1.0f, peakG2 = -1.0f;
    for (uint16_t i = 0; i < n; ++i) {
        uint8_t w[lsm6fifo::kWordBytes] = {};
        s = bus_.readRegisters(lsm6fifo::REG_FIFO_DATA_OUT_TAG, w, sizeof(w));
        if (!isOk(s)) {   // partial drain: what was read is still valid and reported
            if (partialReads_ != UINT32_MAX) ++partialReads_;
            break;
        }
        ++out.wordsRead;
        const FifoWord f = decodeFifoWord(w);
        const bool sat = f.x == INT16_MAX || f.x == INT16_MIN || f.y == INT16_MAX || f.y == INT16_MIN ||
                         f.z == INT16_MAX || f.z == INT16_MIN;
        const float fx = static_cast<float>(f.x), fy = static_cast<float>(f.y), fz = static_cast<float>(f.z);
        const float n2 = fx * fx + fy * fy + fz * fz;
        if (f.tag == FifoTag::Accel) {
            ++out.accelSamples;
            out.accelSaturated = out.accelSaturated || sat;
            if (n2 > peakA2) peakA2 = n2;
        } else if (f.tag == FifoTag::Gyro) {
            ++out.gyroSamples;
            out.gyroSaturated = out.gyroSaturated || sat;
            if (n2 > peakG2) peakG2 = n2;
        } else if (f.tag == FifoTag::Unknown) {
            ++out.unknownWords;
        }
    }
    out.wordsLeft = static_cast<uint16_t>(unread - out.wordsRead);
    if (peakA2 >= 0.0f) out.peakAccel_mps2 = std::sqrt(peakA2) * accelScale_;
    if (peakG2 >= 0.0f) out.peakRate_radps = std::sqrt(peakG2) * gyroScale_;
    return s;
}

}  // namespace pg
