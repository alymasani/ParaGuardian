/**
 ******************************************************************************
 * @file    SampleFreshness.hpp
 * @brief   Consumer-side helpers: sample age, staleness and lost-sample
 *          detection from sequence numbers.
 *
 * Every sample carries header.timestampMs (time the read completed) and
 * header.sequence (incremented by the driver for every sample it produced).
 * A consumer can therefore tell, without any shared state with the producer:
 *   - how old a sample is when it is used (sampleAgeMs),
 *   - whether it is too old to act on (isFresh),
 *   - how many samples of that sensor it never received (SequenceTracker),
 *     whether they were dropped by a full queue, overwritten, or never read.
 * All time arithmetic is unsigned, so it stays correct across the 49.7-day
 * wrap of the millisecond counter.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ACQUISITION_SAMPLE_FRESHNESS_HPP
#define PARAGUARDIAN_ACQUISITION_SAMPLE_FRESHNESS_HPP

#include <cstddef>
#include <cstdint>

#include "Acquisition/SensorMessage.hpp"

namespace pg {

/// Age of a sample at time @p nowMs (wrap-safe).
constexpr uint32_t sampleAgeMs(const SampleHeader& h, uint32_t nowMs)
{
    return nowMs - h.timestampMs;
}

/// True if the sample is valid and not older than @p maxAgeMs.
constexpr bool isFresh(const SampleHeader& h, uint32_t nowMs, uint32_t maxAgeMs)
{
    return h.valid && sampleAgeMs(h, nowMs) <= maxAgeMs;
}

/// Classification of one received sample's sequence number.
enum class SequenceKind : uint8_t {
    First = 0,    ///< first sample this consumer saw from that sensor
    InOrder,      ///< exactly the next sequence number
    Gap,          ///< newer, but samples in between were never received
    Duplicate,    ///< same sequence number as the last one seen
    OutOfOrder,   ///< older than the last one seen
    Invalid,      ///< no sample / unknown sensor id
};

struct SequenceObservation {
    SequenceKind kind;
    uint32_t gap;   ///< samples skipped (only for SequenceKind::Gap)
};

/// Per-sensor sequence tracking for ONE consumer.
class SequenceTracker {
public:
    static constexpr size_t kSensors = static_cast<size_t>(SensorId::Count);

    /**
     * Records a received message.
     * @return number of samples of this sensor that were skipped since the
     *         previous message this consumer saw (0 = no gap).
     */
    uint32_t observe(const SensorMessage& msg)
    {
        const SampleHeader* h = headerOf(msg);
        if (h == nullptr) {
            return 0u;
        }
        return observeHeader(*h).gap;
    }

    /**
     * Same bookkeeping as observe(), but tells the caller WHAT happened, so a
     * duplicate can be told apart from an out-of-order sample.
     * Duplicates and out-of-order samples do not move lastSequence forward.
     * Sequence comparison is wrap-safe (signed difference).
     */
    SequenceObservation observeHeader(const SampleHeader& h)
    {
        if (static_cast<size_t>(h.id) >= kSensors) {
            return SequenceObservation{SequenceKind::Invalid, 0u};
        }
        Entry& e = entries_[static_cast<size_t>(h.id)];
        SequenceObservation obs{SequenceKind::First, 0u};
        if (e.seen) {
            const uint32_t expected = e.lastSequence + 1u;
            if (h.sequence == expected) {
                obs.kind = SequenceKind::InOrder;
            } else if (static_cast<int32_t>(h.sequence - expected) > 0) {
                obs.kind = SequenceKind::Gap;
                obs.gap = h.sequence - expected;   // samples we never got
                e.lost += obs.gap;
            } else {
                ++e.outOfOrder;                    // duplicate or older than last seen
                if (h.sequence == e.lastSequence) {
                    ++e.duplicates;
                    obs.kind = SequenceKind::Duplicate;
                } else {
                    obs.kind = SequenceKind::OutOfOrder;
                }
                return obs;
            }
        }
        e.seen = true;
        e.lastSequence = h.sequence;
        ++e.received;
        return obs;
    }

    uint32_t lost(SensorId id) const { return valid(id) ? entries_[static_cast<size_t>(id)].lost : 0u; }
    uint32_t received(SensorId id) const { return valid(id) ? entries_[static_cast<size_t>(id)].received : 0u; }
    /// Duplicates + older samples (backwards compatible meaning).
    uint32_t outOfOrder(SensorId id) const { return valid(id) ? entries_[static_cast<size_t>(id)].outOfOrder : 0u; }
    /// Exact repeats of the last sequence number (subset of outOfOrder()).
    uint32_t duplicates(SensorId id) const { return valid(id) ? entries_[static_cast<size_t>(id)].duplicates : 0u; }

private:
    static bool valid(SensorId id) { return static_cast<size_t>(id) < kSensors; }
    struct Entry {
        bool seen;
        uint32_t lastSequence;
        uint32_t lost;
        uint32_t received;
        uint32_t outOfOrder;
        uint32_t duplicates;
    };
    Entry entries_[kSensors] = {};
};

}  // namespace pg

#endif  // PARAGUARDIAN_ACQUISITION_SAMPLE_FRESHNESS_HPP
