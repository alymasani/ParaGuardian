/**
 ******************************************************************************
 * @file    TimeBase.hpp
 * @brief   Timestamp convention and wrap-safe time arithmetic for preprocessing.
 *
 * CONVENTION
 *   Every SampleHeader::timestampMs is the MCU time (IClock::millis() =
 *   HAL_GetTick(), 1 ms resolution) at which the driver FINISHED reading the
 *   sample. It is NOT the physical instant the sensor measured. Differences
 *   between the two (sensor filter group delay, BMP585 conversion time, GNSS
 *   navigation latency) are not known for this hardware yet; the
 *   IMeasurementTimeModel interface below is where they will be applied.
 *
 *   Two samples read in the same 10 ms acquisition cycle carry DIFFERENT
 *   timestamps (or the same millisecond by coincidence) and are NOT assumed
 *   to describe the same physical instant.
 *
 * WRAPAROUND
 *   The counter is uint32 ms and wraps after 2^32 ms ~= 49.7 days. All
 *   comparisons use the signed difference (int32)(a - b), which is correct
 *   whenever the true interval is below 2^31 ms ~= 24.8 days. Every interval
 *   the preprocessing compares (sample ages, gaps, windows) is at most a few
 *   seconds, so this assumption holds with a huge margin. Stored "last seen"
 *   timestamps of a sensor that has been silent for > 24.8 days could alias;
 *   they are cleared by staleness long before that (see LatestValidStore).
 *
 * GNSS TIME
 *   GnssSample::gnssTimeOfWeek_ms is in the receiver's GPS time domain; its
 *   header.timestampMs is the MCU time the UBX message was parsed. The two
 *   are never combined: no MCU<->GPS clock mapping exists yet (it would need
 *   the receiver's time pulse, which has no verified pin).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_TIME_BASE_HPP
#define PARAGUARDIAN_PROCESSING_TIME_BASE_HPP

#include <cstdint>

#include "Processing/ProcessingTypes.hpp"
#include "Sensors/Common/SensorTypes.hpp"

namespace pg {
namespace timebase {

/// Largest interval (ms) for which the signed-difference comparisons are valid.
constexpr uint32_t kMaxComparableIntervalMs = 0x7FFFFFFFu;

/// later - earlier as a signed number (wrap-safe for |interval| < 2^31 ms).
constexpr int32_t diffMs(uint32_t later, uint32_t earlier)
{
    return static_cast<int32_t>(later - earlier);
}

/// True if @p a is strictly later than @p b.
constexpr bool isAfter(uint32_t a, uint32_t b) { return diffMs(a, b) > 0; }

/**
 * Age of a timestamp at @p nowMs. A timestamp in the future (negative age)
 * returns 0 here; callers that must detect future timestamps use diffMs().
 */
constexpr uint32_t ageMs(uint32_t timestampMs, uint32_t nowMs)
{
    return (diffMs(nowMs, timestampMs) > 0) ? static_cast<uint32_t>(diffMs(nowMs, timestampMs)) : 0u;
}

}  // namespace timebase

/**
 * Maps a sample's read-completion timestamp to the best estimate of the
 * instant the quantity was physically measured. Future work (hardware):
 * per-sensor latency, interrupt timestamps from data-ready pins, or GNSS
 * time-pulse alignment can be implemented behind this interface without
 * touching any consumer.
 */
class IMeasurementTimeModel {
public:
    virtual uint32_t measurementTimeMs(const SampleHeader& h) const = 0;

protected:
    ~IMeasurementTimeModel() = default;
};

/**
 * Default model: measurement time = read-completion time minus a fixed,
 * configurable per-sensor latency. All latencies default to 0 because no
 * latency has been measured on this hardware (documented as an open item).
 */
class FixedLatencyTimeModel final : public IMeasurementTimeModel {
public:
    void setLatencyMs(SensorId id, uint32_t latencyMs)
    {
        if (sensorIndex(id) < kSensorCount) {
            latencyMs_[sensorIndex(id)] = latencyMs;
        }
    }
    uint32_t latencyMs(SensorId id) const
    {
        return (sensorIndex(id) < kSensorCount) ? latencyMs_[sensorIndex(id)] : 0u;
    }
    uint32_t measurementTimeMs(const SampleHeader& h) const override
    {
        return h.timestampMs - latencyMs(h.id);  // unsigned: wraps consistently
    }

private:
    uint32_t latencyMs_[kSensorCount] = {};
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_TIME_BASE_HPP
