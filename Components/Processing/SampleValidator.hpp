/**
 ******************************************************************************
 * @file    SampleValidator.hpp
 * @brief   Per-sample validity, freshness, range, timestamp and sequence checks.
 *
 * Input : one SensorMessage exactly as published by SensorAcquisitionTask
 *         (SI units, sensor axes) and the current MCU time.
 * Output: a ValidationResult that says, with a bit per reason, whether the
 *         whole sample and (for IMUs) each channel may be used.
 *
 * What it deliberately does NOT do:
 *   - It never "repairs" data (no clamping, no zero-filling of missing data).
 *   - It never treats a physically unusual value as a fault: near-zero
 *     specific force (free fall) is accepted, as is any attitude.
 *   - It does not compare sensors with each other (RedundancyVoter does).
 *
 * State: per-sensor sequence/timestamp of the last ACCEPTED sample (for
 * duplicate / out-of-order / backwards-time detection) and counters.
 * Memory: ~0.5 KB, fixed. Execution: O(1) per sample, no allocation.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_SAMPLE_VALIDATOR_HPP
#define PARAGUARDIAN_PROCESSING_SAMPLE_VALIDATOR_HPP

#include <cstdint>

#include "Acquisition/SampleFreshness.hpp"
#include "Acquisition/SensorMessage.hpp"
#include "Processing/ProcessingConfig.hpp"
#include "Processing/ProcessingTypes.hpp"

namespace pg {

struct ValidationResult {
    uint32_t reasons;         ///< reject::* bits (0 = nothing wrong)
    uint32_t ageMs;           ///< age at validation time (0 if timestamp in the future)
    uint32_t sequenceGap;     ///< samples of this sensor never received before this one
    SequenceKind sequence;
    bool accepted;            ///< no whole-sample reason: the sample may be used
    bool accelUsable;         ///< IMU only: accepted && no accel channel reason
    bool gyroUsable;          ///< IMU only: accepted && no gyro channel reason

    bool has(uint32_t reasonBits) const { return (reasons & reasonBits) != 0u; }
};

class SampleValidator {
public:
    explicit SampleValidator(const ValidationConfig& cfg) : cfg_(cfg) {}

    /// Validates @p msg at time @p nowMs and updates sequence state for accepted samples.
    ValidationResult validate(const SensorMessage& msg, uint32_t nowMs);

    /// Pure check of the sample contents only (no time / sequence state). Exposed for tests.
    uint32_t contentReasons(const SensorMessage& msg) const;

    const ValidationConfig& config() const { return cfg_; }

    // ---- diagnostics ----
    uint32_t acceptedCount(SensorId id) const { return ok(id) ? stats_[sensorIndex(id)].accepted : 0u; }
    uint32_t rejectedCount(SensorId id) const { return ok(id) ? stats_[sensorIndex(id)].rejected : 0u; }
    uint32_t lastRejectReasons(SensorId id) const { return ok(id) ? stats_[sensorIndex(id)].lastReasons : 0u; }
    uint32_t lostCount(SensorId id) const { return sequence_.lost(id); }
    /// Number of samples (any sensor) that had reason bit @p bit (0..reject::kReasonBits-1) set.
    uint32_t reasonCount(unsigned bit) const { return bit < reject::kReasonBits ? reasonCounts_[bit] : 0u; }
    uint32_t totalRejected() const { return totalRejected_; }

private:
    static bool ok(SensorId id) { return sensorIndex(id) < kSensorCount; }
    static bool idMatchesType(const SensorMessage& msg, SensorId id);
    void record(SensorId id, uint32_t reasons, bool accepted);

    struct PerSensor {
        bool haveAccepted;
        uint32_t lastTimestampMs;
        uint32_t lastSequence;
        uint32_t accepted;
        uint32_t rejected;
        uint32_t lastReasons;
    };

    ValidationConfig cfg_;
    SequenceTracker sequence_{};
    PerSensor stats_[kSensorCount] = {};
    uint32_t reasonCounts_[reject::kReasonBits] = {};
    uint32_t totalRejected_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_SAMPLE_VALIDATOR_HPP
