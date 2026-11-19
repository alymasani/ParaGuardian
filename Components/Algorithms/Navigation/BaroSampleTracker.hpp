/**
 ******************************************************************************
 * @file    BaroSampleTracker.hpp
 * @brief   Decides whether the voted barometric altitude in a frame is a NEW
 *          observation, so each barometer sample is assimilated at most once.
 *
 * PROBLEM: preprocessing republishes the voted altitude in every 10 ms frame,
 * the barometers run at 50 Hz, and the two barometers are read in DIFFERENT
 * 10 ms cycles. The vote's newest timestamp therefore changes every 10 ms
 * although each change reuses one of the previous samples. Assimilating every
 * timestamp change would feed each barometer sample to the filter twice and
 * treat the two barometers' vote as more independent observations than exist.
 *
 * RULE: an observation is new only when EVERY contributing barometer's sample
 * is newer than the sample of that barometer used in the previous observation.
 * With two healthy barometers this yields one observation per 20 ms containing
 * two fresh samples (one vote = one observation, never two). With one
 * barometer it yields that barometer's 50 Hz rate.
 *
 * The observation time is the midpoint of the contributing samples
 * (newest - spread / 2): the vote is their mean.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_BARO_SAMPLE_TRACKER_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_BARO_SAMPLE_TRACKER_HPP

#include <cstdint>

#include "Algorithms/Navigation/NavigationTypes.hpp"
#include "Processing/TimeBase.hpp"

namespace pg {

class BaroSampleTracker {
public:
    enum class Verdict : uint8_t { New, NotNew, OutOfOrder };

    /// Classifies the vote in @p in (caller has checked in.baroValid). Does not record it.
    Verdict classify(const NavigationInput& in) const
    {
        bool anyOld = false;
        for (uint32_t j = 0; j < kBaroCount; ++j) {
            if ((in.baroContributingMask & (1u << j)) == 0u || !used_[j]) continue;
            const int32_t d = timebase::diffMs(in.baroChannelTimestampMs[j], lastTs_[j]);
            if (d < 0) return Verdict::OutOfOrder;
            if (d == 0) anyOld = true;
        }
        return anyOld ? Verdict::NotNew : Verdict::New;
    }

    /// Records the contributing samples of @p in as used.
    void markUsed(const NavigationInput& in)
    {
        for (uint32_t j = 0; j < kBaroCount; ++j) {
            if ((in.baroContributingMask & (1u << j)) == 0u) continue;
            lastTs_[j] = in.baroChannelTimestampMs[j];
            used_[j] = true;
        }
    }

    void reset()
    {
        for (uint32_t j = 0; j < kBaroCount; ++j) {
            used_[j] = false;
            lastTs_[j] = 0u;
        }
    }

    /// Observation time of the vote: midpoint of the contributing samples.
    static uint32_t observationTime(const NavigationInput& in) { return in.baroTimestampMs - in.baroSpreadMs / 2u; }

private:
    uint32_t lastTs_[kBaroCount] = {};
    bool used_[kBaroCount] = {};
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_BARO_SAMPLE_TRACKER_HPP
