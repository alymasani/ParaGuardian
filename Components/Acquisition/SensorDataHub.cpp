/**
 ******************************************************************************
 * @file    SensorDataHub.cpp
 * @brief   See SensorDataHub.hpp.
 ******************************************************************************
 */
#include "Acquisition/SensorDataHub.hpp"

namespace pg {

bool SensorDataHub::subscribe(ISampleQueue* queue, OverflowPolicy policy)
{
    if (queue == nullptr || count_ >= kMaxSubscribers) {
        return false;
    }
    for (size_t i = 0; i < count_; ++i) {
        if (subs_[i].queue == queue) {
            return false;
        }
    }
    subs_[count_++] = Subscription{queue, policy, 0u, 0u};
    return true;
}

bool SensorDataHub::subscribeMailbox(LatestSampleMailbox* mailbox)
{
    if (mailbox == nullptr || mailboxCount_ >= kMaxMailboxes) {
        return false;
    }
    for (size_t i = 0; i < mailboxCount_; ++i) {
        if (mailboxes_[i] == mailbox) {
            return false;
        }
    }
    mailboxes_[mailboxCount_++] = mailbox;
    return true;
}

void SensorDataHub::publish(const SensorMessage& msg)
{
    saturatingIncrement(published_);
    for (size_t i = 0; i < count_; ++i) {
        Subscription& s = subs_[i];
        if (s.queue->tryPush(msg)) {
            continue;
        }
        if (s.policy == OverflowPolicy::DropOldest && s.queue->discardOldest()) {
            // Only this (single) producer adds items, so after removing one
            // there is room - unless the queue has capacity 0, impossible here.
            saturatingIncrement(s.overwrites);
            if (s.queue->tryPush(msg)) {
                continue;
            }
        }
        saturatingIncrement(s.drops);
    }
    for (size_t i = 0; i < mailboxCount_; ++i) {
        mailboxes_[i]->store(msg);
    }
}

uint32_t SensorDataHub::totalDrops() const
{
    uint32_t total = 0u;
    for (size_t i = 0; i < count_; ++i) {
        total = (UINT32_MAX - total < subs_[i].drops) ? UINT32_MAX : total + subs_[i].drops;
    }
    return total;
}

uint32_t SensorDataHub::totalOverwrites() const
{
    uint32_t total = 0u;
    for (size_t i = 0; i < count_; ++i) {
        total = (UINT32_MAX - total < subs_[i].overwrites) ? UINT32_MAX : total + subs_[i].overwrites;
    }
    return total;
}

}  // namespace pg
