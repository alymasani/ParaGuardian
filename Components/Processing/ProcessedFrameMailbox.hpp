/**
 ******************************************************************************
 * @file    ProcessedFrameMailbox.hpp
 * @brief   Bounded, non-blocking hand-over of the newest ProcessedSensorFrame.
 *
 * One slot, always overwritten by the producer (SensorProcessingTask).
 * Readers copy it out under the injected critical section and use
 * frame.sequence to see whether it is new and how many frames they missed.
 * Fixed memory (one frame), never blocks the producer, never queues stale
 * frames - the freshest-data policy chosen for navigation/safety consumers.
 * A future Madgwick task that must see EVERY frame can be given its own
 * bounded queue behind IProcessedFrameSink without changing the producer.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_PROCESSED_FRAME_MAILBOX_HPP
#define PARAGUARDIAN_PROCESSING_PROCESSED_FRAME_MAILBOX_HPP

#include <cstdint>

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection
#include "Processing/ProcessedSensorFrame.hpp"

namespace pg {

class IProcessedFrameSink {
public:
    virtual void publish(const ProcessedSensorFrame& frame) = 0;

protected:
    ~IProcessedFrameSink() = default;
};

class ProcessedFrameMailbox final : public IProcessedFrameSink {
public:
    explicit ProcessedFrameMailbox(ICriticalSection& lock) : lock_(lock) {}

    void publish(const ProcessedSensorFrame& frame) override
    {
        lock_.enter();
        frame_ = frame;
        present_ = true;
        ++writes_;
        lock_.exit();
    }

    /// Copies the newest frame. Returns false if none has been published yet.
    bool read(ProcessedSensorFrame& out) const
    {
        lock_.enter();
        const bool p = present_;
        if (p) {
            out = frame_;
        }
        lock_.exit();
        return p;
    }

    uint32_t writes() const
    {
        lock_.enter();
        const uint32_t w = writes_;
        lock_.exit();
        return w;
    }

private:
    ICriticalSection& lock_;
    ProcessedSensorFrame frame_{};
    bool present_ = false;
    uint32_t writes_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_PROCESSED_FRAME_MAILBOX_HPP
