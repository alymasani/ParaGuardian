/**
 ******************************************************************************
 * @file    SafetyEventQueue.hpp
 * @brief   Bounded, priority-aware FIFO of SafetyEvents (fixed memory, no heap).
 *
 * One producer and one consumer per queue (processing queue: SensorProcessingTask
 * -> EmergencyAlertTask's dispatcher), both under the injected ICriticalSection.
 * Order is FIFO. OVERFLOW POLICY (queue full, new event E):
 *   1. If some queued event has a LOWER severity than E, the OLDEST of the
 *      lowest-severity events is dropped and E is appended.
 *   2. Otherwise (everything queued is at least as severe as E):
 *        - E Critical: the oldest queued event is dropped (critical news is
 *          newer = more relevant), counted in droppedCritical if it was Critical;
 *        - else E itself is dropped.
 * So a critical event is never lost because the queue is full of
 * lower-priority messages. Every drop is counted (stats()).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_QUEUE_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_QUEUE_HPP

#include <cstddef>
#include <cstdint>

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection
#include "Algorithms/Safety/SafetyEvent.hpp"

namespace pg {

struct SafetyQueueStats {
    uint32_t published;       ///< accepted into the queue
    uint32_t droppedNew;      ///< the new event was dropped (queue full of >= severity)
    uint32_t droppedQueued;   ///< a queued lower-severity event was dropped to make room
    uint32_t droppedCritical; ///< a queued CRITICAL event was dropped (queue full of criticals)
    uint32_t highWater;       ///< largest occupancy seen
    uint32_t consumed;
};

template <size_t N>
class SafetyEventQueue final : public ISafetyEventSink {
    static_assert(N >= 2u, "queue too small");

public:
    explicit SafetyEventQueue(ICriticalSection& lock) : lock_(lock) {}

    bool publish(const SafetyEvent& e) override
    {
        lock_.enter();
        bool ok = true;
        if (count_ == N) {
            // Find the oldest event with the lowest severity.
            size_t victim = 0u;
            for (size_t i = 1u; i < count_; ++i) {
                if (at(i).severity < at(victim).severity) victim = i;
            }
            if (at(victim).severity < e.severity) {
                removeAt(victim);
                inc(stats_.droppedQueued);
                ok = false;
            } else if (e.severity == SafetySeverity::Critical) {
                if (at(0u).severity == SafetySeverity::Critical) inc(stats_.droppedCritical);
                removeAt(0u);
                inc(stats_.droppedQueued);
                ok = false;
            } else {
                inc(stats_.droppedNew);
                lock_.exit();
                return false;
            }
        }
        buf_[(head_ + count_) % N] = e;
        ++count_;
        inc(stats_.published);
        if (count_ > stats_.highWater) stats_.highWater = static_cast<uint32_t>(count_);
        lock_.exit();
        return ok;
    }

    /// Takes the oldest event. Returns false if empty.
    bool pop(SafetyEvent& out)
    {
        lock_.enter();
        const bool have = count_ != 0u;
        if (have) {
            out = buf_[head_];
            head_ = (head_ + 1u) % N;
            --count_;
            inc(stats_.consumed);
        }
        lock_.exit();
        return have;
    }

    size_t size() const
    {
        lock_.enter();
        const size_t n = count_;
        lock_.exit();
        return n;
    }

    SafetyQueueStats stats() const
    {
        lock_.enter();
        const SafetyQueueStats s = stats_;
        lock_.exit();
        return s;
    }

    static constexpr size_t capacity() { return N; }

private:
    static void inc(uint32_t& c)
    {
        if (c != UINT32_MAX) ++c;
    }
    SafetyEvent& at(size_t i) { return buf_[(head_ + i) % N]; }
    void removeAt(size_t i)
    {
        for (size_t k = i; k + 1u < count_; ++k) at(k) = at(k + 1u);
        --count_;
    }

    ICriticalSection& lock_;
    SafetyEvent buf_[N] = {};
    size_t head_ = 0u;
    size_t count_ = 0u;
    SafetyQueueStats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_QUEUE_HPP
