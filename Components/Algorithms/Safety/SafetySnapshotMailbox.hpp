/**
 ******************************************************************************
 * @file    SafetySnapshotMailbox.hpp
 * @brief   Latest-value hand-over of the SafetySnapshot (same pattern as
 *          AttitudeMailbox / NavigationMailbox). Consumers that need the
 *          CURRENT warning state (e.g. a tone that repeats while a warning is
 *          active) read this; transitions come through SafetyEventQueue.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_SNAPSHOT_MAILBOX_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_SNAPSHOT_MAILBOX_HPP

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection
#include "Algorithms/Landing/LandingSafetyTypes.hpp"

namespace pg {

class SafetySnapshotMailbox {
public:
    explicit SafetySnapshotMailbox(ICriticalSection& lock) : lock_(lock) {}

    void publish(const SafetySnapshot& s)
    {
        lock_.enter();
        value_ = s;
        present_ = true;
        lock_.exit();
    }

    bool read(SafetySnapshot& out) const
    {
        lock_.enter();
        const bool p = present_;
        if (p) out = value_;
        lock_.exit();
        return p;
    }

private:
    ICriticalSection& lock_;
    SafetySnapshot value_{};
    bool present_ = false;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_SNAPSHOT_MAILBOX_HPP
