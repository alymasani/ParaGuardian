/**
 ******************************************************************************
 * @file    AttitudeMailbox.hpp
 * @brief   Latest-value hand-over of the AttitudeEstimate (same pattern as
 *          ProcessedFrameMailbox): one slot, overwritten by the producer
 *          (SensorProcessingTask), copied out by readers under the injected
 *          critical section. Never blocks the producer, fixed memory.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_MAILBOX_HPP
#define PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_MAILBOX_HPP

#include <cstdint>

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection
#include "Algorithms/Attitude/AttitudeTypes.hpp"

namespace pg {

class AttitudeMailbox {
public:
    explicit AttitudeMailbox(ICriticalSection& lock) : lock_(lock) {}

    void publish(const AttitudeEstimate& e)
    {
        lock_.enter();
        value_ = e;
        present_ = true;
        lock_.exit();
    }

    /// Copies the newest estimate. Returns false if none has been published yet.
    bool read(AttitudeEstimate& out) const
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
    AttitudeEstimate value_{};
    bool present_ = false;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_MAILBOX_HPP
