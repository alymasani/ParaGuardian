/**
 ******************************************************************************
 * @file    SensorDataHub.hpp
 * @brief   Minimal ParaGuardian-owned publish/subscribe for sensor samples.
 *
 * WHY NOT SoarOS DataBroker?
 *   DataBroker only knows the rocket's message types (IMUData, BaroData...).
 *   Adding ParaGuardian types would require editing SoarOS, and every publish
 *   heap-allocates a copy (Command::CopyDataToCommand -> pvPortMalloc) while
 *   the CubeMX FreeRTOS heap is only 15 KB. This hub instead copies a fixed
 *   size message into each subscriber's statically allocated storage.
 *
 * SUBSCRIBER KINDS AND OVERFLOW POLICIES (choose per consumer):
 *   Queue, OverflowPolicy::DropNewest  (default, original behaviour)
 *       Full queue -> the NEW sample is not delivered; drops counted.
 *       Keeps an unbroken chronological prefix. Use for LOGGING / DIAGNOSTICS.
 *   Queue, OverflowPolicy::DropOldest
 *       Full queue -> the OLDEST queued sample is discarded to make room for
 *       the new one; overwrites counted. The consumer always finds the most
 *       recent samples, still in order. Use for NAVIGATION.
 *   LatestSampleMailbox
 *       One slot per sensor, always overwritten. Never a backlog.
 *       Use for SAFETY MONITORING (always check the sample age).
 *   Loss is ALSO visible to every consumer through sequence gaps
 *   (SampleFreshness.hpp, SequenceTracker).
 *
 * The publisher never blocks. Subscribing happens during start-up (before the
 * scheduler starts); publish() is then called from the single acquisition task.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ACQUISITION_SENSOR_DATA_HUB_HPP
#define PARAGUARDIAN_ACQUISITION_SENSOR_DATA_HUB_HPP

#include <cstddef>
#include <cstdint>

#include "Acquisition/LatestSampleMailbox.hpp"
#include "Acquisition/SampleQueue.hpp"
#include "Acquisition/SensorMessage.hpp"

namespace pg {

/// Anything that accepts published samples (the hub, or a test double).
class ISampleSink {
public:
    virtual void publish(const SensorMessage& msg) = 0;

protected:
    ~ISampleSink() = default;
};

enum class OverflowPolicy : uint8_t {
    DropNewest = 0,
    DropOldest = 1,
};

class SensorDataHub final : public ISampleSink {
public:
    static constexpr size_t kMaxSubscribers = 4u;
    static constexpr size_t kMaxMailboxes = 2u;

    /// Adds a subscriber queue. Returns false if full or @p queue is null/duplicate.
    bool subscribe(ISampleQueue* queue, OverflowPolicy policy = OverflowPolicy::DropNewest);

    /// Adds a latest-value mailbox. Returns false if full or null/duplicate.
    bool subscribeMailbox(LatestSampleMailbox* mailbox);

    void publish(const SensorMessage& msg) override;

    size_t subscriberCount() const { return count_; }
    const ISampleQueue* subscriber(size_t i) const { return (i < count_) ? subs_[i].queue : nullptr; }
    OverflowPolicy policyFor(size_t i) const { return (i < count_) ? subs_[i].policy : OverflowPolicy::DropNewest; }
    /// Samples that were NOT delivered to subscriber @p i.
    uint32_t dropsFor(size_t i) const { return (i < count_) ? subs_[i].drops : 0u; }
    /// Older samples discarded from subscriber @p i to make room (DropOldest only).
    uint32_t overwritesFor(size_t i) const { return (i < count_) ? subs_[i].overwrites : 0u; }
    uint32_t publishedCount() const { return published_; }
    uint32_t totalDrops() const;
    uint32_t totalOverwrites() const;
    size_t mailboxCount() const { return mailboxCount_; }

private:
    struct Subscription {
        ISampleQueue* queue;
        OverflowPolicy policy;
        uint32_t drops;
        uint32_t overwrites;
    };
    static void saturatingIncrement(uint32_t& v)
    {
        if (v != UINT32_MAX) {
            ++v;
        }
    }

    Subscription subs_[kMaxSubscribers] = {};
    size_t count_ = 0u;
    LatestSampleMailbox* mailboxes_[kMaxMailboxes] = {};
    size_t mailboxCount_ = 0u;
    uint32_t published_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ACQUISITION_SENSOR_DATA_HUB_HPP
