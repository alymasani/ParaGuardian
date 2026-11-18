/**
 ******************************************************************************
 * @file    SensorHealth.hpp
 * @brief   Per-sensor success / error counters.
 *
 * Every driver owns one SensorHealth. Each bus operation records either a
 * success or a failure. The health monitor task reads these counters and the
 * (future) algorithms use isHealthy() to decide whether to trust a sensor.
 * Counters saturate at their maximum instead of wrapping back to 0.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_SENSORS_COMMON_SENSOR_HEALTH_HPP
#define PARAGUARDIAN_SENSORS_COMMON_SENSOR_HEALTH_HPP

#include <cstdint>

#include "Common/Status.hpp"

namespace pg {

struct SensorHealth {
    uint32_t successCount = 0;
    uint32_t failureCount = 0;
    uint32_t busErrorCount = 0;
    uint32_t timeoutCount = 0;
    uint32_t invalidDataCount = 0;
    uint16_t consecutiveFailures = 0;
    Status lastError = Status::Ok;
    bool initialized = false;
    uint32_t lastSuccessMs = 0;

    void recordSuccess(uint32_t nowMs)
    {
        increment(successCount);
        consecutiveFailures = 0;
        lastSuccessMs = nowMs;
    }

    void recordFailure(Status s)
    {
        if (s == Status::Ok) {
            return;
        }
        increment(failureCount);
        if (consecutiveFailures != UINT16_MAX) {
            ++consecutiveFailures;
        }
        lastError = s;
        switch (s) {
            case Status::BusError:      increment(busErrorCount); break;
            case Status::Timeout:       increment(timeoutCount); break;
            case Status::InvalidData:
            case Status::ChecksumError:
            case Status::OutOfRange:    increment(invalidDataCount); break;
            default: break;
        }
    }

    /// Healthy = initialised and fewer than @p maxConsecutiveFailures failures in a row.
    bool isHealthy(uint16_t maxConsecutiveFailures = 3u) const
    {
        return initialized && consecutiveFailures < maxConsecutiveFailures;
    }

private:
    static void increment(uint32_t& counter)
    {
        if (counter != UINT32_MAX) {
            ++counter;
        }
    }
};

}  // namespace pg

#endif  // PARAGUARDIAN_SENSORS_COMMON_SENSOR_HEALTH_HPP
