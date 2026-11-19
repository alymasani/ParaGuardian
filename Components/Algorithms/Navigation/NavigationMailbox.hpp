/**
 ******************************************************************************
 * @file    NavigationMailbox.hpp
 * @brief   Latest-value hand-over of the NavigationEstimate (same pattern as
 *          AttitudeMailbox): one slot, overwritten by the producer
 *          (SensorProcessingTask), copied out by readers under the injected
 *          critical section. Never blocks the producer; fixed memory.
 *
 * This is how future flight-state, landing, crash and SOS logic will read
 * navigation. Use `sequence` to detect a new estimate (and missed ones).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_MAILBOX_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_MAILBOX_HPP

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection
#include "Algorithms/Navigation/NavigationTypes.hpp"

namespace pg {

class NavigationMailbox {
public:
    explicit NavigationMailbox(ICriticalSection& lock) : lock_(lock) {}

    void publish(const NavigationEstimate& e)
    {
        lock_.enter();
        value_ = e;
        present_ = true;
        lock_.exit();
    }

    /// Copies the newest estimate. Returns false if none has been published yet.
    bool read(NavigationEstimate& out) const
    {
        lock_.enter();
        const bool p = present_;
        if (p) {
            out = value_;
        }
        lock_.exit();
        return p;
    }

private:
    ICriticalSection& lock_;
    NavigationEstimate value_{};
    bool present_ = false;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_MAILBOX_HPP
