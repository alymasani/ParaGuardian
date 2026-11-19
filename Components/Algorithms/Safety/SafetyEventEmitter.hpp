/**
 ******************************************************************************
 * @file    SafetyEventEmitter.hpp
 * @brief   Numbers events / episodes and hands them to the sinks.
 *
 * Shared by every producer in one processing context so event ids are one
 * increasing sequence (a gap seen by a consumer = an event it lost). Up to
 * kMaxSinks sinks (e.g. audio queue + logging queue); a sink that drops is
 * counted, the others still receive the event. Never blocks.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_EMITTER_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_EMITTER_HPP

#include <cstdint>

#include "Algorithms/Safety/SafetyEvent.hpp"

namespace pg {

class SafetyEventEmitter {
public:
    static constexpr uint8_t kMaxSinks = 4u;

    /// Adds a sink (call before the scheduler starts). False if full or null.
    bool addSink(ISafetyEventSink* s)
    {
        if (s == nullptr || sinkCount_ >= kMaxSinks) return false;
        sinks_[sinkCount_++] = s;
        return true;
    }

    /// Assigns the event id and publishes to every sink.
    void emit(SafetyEvent e)
    {
        e.eventId = ++nextEventId_;
        inc(published_);
        for (uint8_t i = 0; i < sinkCount_; ++i) {
            if (!sinks_[i]->publish(e)) inc(rejected_);
        }
    }

    uint32_t newEpisode() { return ++nextEpisodeId_; }
    uint32_t published() const { return published_; }
    uint32_t rejectedBySinks() const { return rejected_; }
    uint32_t lastEventId() const { return nextEventId_; }

private:
    static void inc(uint32_t& c)
    {
        if (c != UINT32_MAX) ++c;
    }
    ISafetyEventSink* sinks_[kMaxSinks] = {};
    uint8_t sinkCount_ = 0u;
    uint32_t nextEventId_ = 0u;
    uint32_t nextEpisodeId_ = 0u;
    uint32_t published_ = 0u;
    uint32_t rejected_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_EMITTER_HPP
