/**
 ******************************************************************************
 * @file    Tf02Pro.cpp
 * @brief   TF02-Pro frame parser implementation. See Tf02Pro.hpp.
 ******************************************************************************
 */
#include "Sensors/TF02Pro/Tf02Pro.hpp"

#include "Common/ByteOrder.hpp"

namespace pg {
namespace tf02 {

float rawToTemperatureC(uint16_t raw)
{
    return static_cast<float>(raw) / 8.0f - 256.0f;
}

void Parser::reset()
{
    count_ = 0u;
    stats_ = ParserStats{};
}

void Parser::resyncAfterBadFrame()
{
    // The 9 bytes we hold were not a valid frame. A real frame may start
    // somewhere inside them, so look for the next "0x59 0x59" (or a trailing
    // single 0x59) after position 0 and keep the bytes from there.
    size_t start = FRAME_LEN;
    for (size_t i = 1; i < FRAME_LEN; ++i) {
        if (buf_[i] == HEADER && (i == FRAME_LEN - 1u || buf_[i + 1u] == HEADER)) {
            start = i;
            break;
        }
    }
    stats_.discardedBytes += static_cast<uint32_t>(start);
    count_ = FRAME_LEN - start;
    for (size_t i = 0; i < count_; ++i) {
        buf_[i] = buf_[start + i];
    }
}

bool Parser::feed(uint8_t byte, RawFrame& out)
{
    // Header hunting: byte 0 and byte 1 must both be 0x59.
    if (count_ == 0u && byte != HEADER) {
        ++stats_.discardedBytes;
        return false;
    }
    if (count_ == 1u && byte != HEADER) {
        stats_.discardedBytes += 2u;
        count_ = 0u;
        return false;
    }
    buf_[count_++] = byte;
    if (count_ < FRAME_LEN) {
        return false;
    }

    uint32_t sum = 0u;
    for (size_t i = 0; i < FRAME_LEN - 1u; ++i) {
        sum += buf_[i];
    }
    if (static_cast<uint8_t>(sum & 0xFFu) != buf_[FRAME_LEN - 1u]) {
        ++stats_.checksumErrors;
        resyncAfterBadFrame();
        return false;
    }

    out.distance = bytes::le_u16(&buf_[2]);
    out.strength = bytes::le_u16(&buf_[4]);
    out.temperatureRaw = bytes::le_u16(&buf_[6]);
    ++stats_.framesOk;
    count_ = 0u;
    return true;
}

}  // namespace tf02

void Tf02Pro::frameToSample(const tf02::RawFrame& frame, const Tf02ProConfig& config, LidarSample& out)
{
    const float scale = (config.unit == Tf02DistanceUnit::Centimetre) ? 0.01f : 0.001f;
    out.distance_m = static_cast<float>(frame.distance) * scale;
    out.signalStrength = frame.strength;
    out.temperature_c = tf02::rawToTemperatureC(frame.temperatureRaw);
    out.weakSignal = frame.strength < tf02::MIN_RELIABLE_STRENGTH || frame.strength == tf02::OVEREXPOSED_STRENGTH;
    out.outOfRange = out.distance_m > config.maxRange_m;
    out.header.valid = !out.weakSignal && !out.outOfRange;
}

size_t Tf02Pro::processBytes(const uint8_t* data, size_t len, uint32_t nowMs)
{
    if (data == nullptr) {
        return 0u;
    }
    size_t produced = 0u;
    const uint32_t ckBefore = parser_.stats().checksumErrors;
    for (size_t i = 0; i < len; ++i) {
        tf02::RawFrame frame{};
        if (parser_.feed(data[i], frame)) {
            frameToSample(frame, config_, latest_);
            latest_.header.id = id_;
            latest_.header.timestampMs = nowMs;
            latest_.header.sequence = ++sequence_;
            haveMeasurement_ = true;
            lastMeasurementMs_ = nowMs;
            health_.initialized = true;
            if (latest_.header.valid) {
                health_.recordSuccess(nowMs);
            } else {
                // Weak/out-of-range readings are expected (sky, water) and are
                // not a device fault; count them separately from bus errors.
                health_.recordFailure(Status::OutOfRange);
            }
            ++produced;
        }
    }
    for (uint32_t k = ckBefore; k < parser_.stats().checksumErrors; ++k) {
        health_.recordFailure(Status::ChecksumError);
    }
    return produced;
}

void Tf02Pro::poll(uint32_t nowMs)
{
    if (haveMeasurement_ && !hasFreshMeasurement(nowMs) && latest_.header.valid) {
        latest_.header.valid = false;
        health_.recordFailure(Status::Timeout);
    }
}

bool Tf02Pro::hasFreshMeasurement(uint32_t nowMs) const
{
    return haveMeasurement_ && (nowMs - lastMeasurementMs_) <= config_.staleTimeoutMs;
}

}  // namespace pg
