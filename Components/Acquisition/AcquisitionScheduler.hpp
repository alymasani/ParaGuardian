/**
 ******************************************************************************
 * @file    AcquisitionScheduler.hpp
 * @brief   One acquisition cycle: service every source once, with an init
 *          budget, and measure how long the cycle took.
 *
 * This is the body of SensorAcquisitionTask's loop, moved into portable code
 * so its timing behaviour can be tested on a laptop with a fake clock whose
 * time advances whenever a driver "waits". The task keeps the RTOS-specific
 * part (xTaskDelayUntil, missed-deadline counting).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ACQUISITION_SCHEDULER_HPP
#define PARAGUARDIAN_ACQUISITION_SCHEDULER_HPP

#include <cstddef>
#include <cstdint>

#include "Acquisition/AcquisitionSources.hpp"
#include "Acquisition/SensorDataHub.hpp"
#include "Common/BusInterfaces.hpp"

namespace pg {

class AcquisitionScheduler {
public:
    static constexpr size_t kMaxSources = 10u;  // 3 IMU + 2 baro + mag + GNSS + LiDAR + spare

    struct Stats {
        uint32_t cycles;
        uint32_t cycleOverruns;         ///< cycles that took >= periodMs
        uint32_t maxCycleTimeMs;
        uint32_t lastCycleTimeMs;
        uint32_t initAttemptsDeferred;  ///< init attempts postponed by the per-cycle budget
    };

    AcquisitionScheduler(IClock& clock, uint32_t periodMs, uint8_t initAttemptsPerCycle = 1u)
        : clock_(clock), periodMs_(periodMs), initAttemptsPerCycle_(initAttemptsPerCycle) {}

    /// Adds a source. Returns false if null or full. Not thread-safe: call before the loop starts.
    bool add(IAcquisitionSource* source);

    /// Runs one cycle; returns the cycle's duration in ms (clock resolution).
    uint32_t runCycle(ISampleSink& sink);

    const Stats& stats() const { return stats_; }
    size_t sourceCount() const { return count_; }
    IAcquisitionSource* source(size_t i) const { return (i < count_) ? sources_[i] : nullptr; }
    uint32_t periodMs() const { return periodMs_; }

private:
    IClock& clock_;
    uint32_t periodMs_;
    uint8_t initAttemptsPerCycle_;
    IAcquisitionSource* sources_[kMaxSources] = {};
    size_t count_ = 0u;
    size_t rotation_ = 0u;  // start index rotates so no source always goes first
    Stats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_ACQUISITION_SCHEDULER_HPP
