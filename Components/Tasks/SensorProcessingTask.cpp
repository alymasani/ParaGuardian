/**
 ******************************************************************************
 * @file    SensorProcessingTask.cpp
 * @brief   See SensorProcessingTask.hpp.
 ******************************************************************************
 */
#include "Tasks/SensorProcessingTask.hpp"

#include "SystemDefines.hpp"
#include "Acquisition/SensorDataHub.hpp"
#include "BSP/RtosClock.hpp"

SensorProcessingTask::SensorProcessingTask()
    : Task(0u),  // no SoarOS command queue: data arrives through queue_
      pipeline_(pg::PreprocessingConfig{}),
      frames_(frameLock_)
{
}

void SensorProcessingTask::InitTask()
{
    SOAR_ASSERT(false, "SensorProcessingTask::InitTask(hub) must be used");
}

void SensorProcessingTask::InitTask(pg::SensorDataHub& hub)
{
    SOAR_ASSERT(rtTaskHandle == nullptr, "SensorProcessingTask created twice");

    // Static queue: no FreeRTOS heap use, cannot fail for lack of memory.
    const bool queueOk = queue_.create();
    SOAR_ASSERT(queueOk, "SensorProcessingTask: queue creation failed");

    // Subscribed before the scheduler starts, so no locking is needed.
    const bool subscribed = hub.subscribe(&queue_, pg::OverflowPolicy::DropOldest);
    SOAR_ASSERT(subscribed, "SensorProcessingTask: hub subscription failed");
    (void)queueOk;
    (void)subscribed;

    rtTaskHandle = xTaskCreateStatic(&SensorProcessingTask::RunTask,
                                     "SensorProc",
                                     TaskConfig::PROCESSING_STACK_WORDS,  // WORDS
                                     this,
                                     TaskConfig::PROCESSING_PRIORITY,
                                     stack_,
                                     &tcb_);
    SOAR_ASSERT(rtTaskHandle != nullptr, "SensorProcessingTask: xTaskCreateStatic failed");
}

SensorProcessingTask::Stats SensorProcessingTask::stats() const
{
    return stats_;
}

void SensorProcessingTask::Run(void* /*pvParams*/)
{
    pg::IClock& clock = pg::bsp::systemClock();
    pipeline_.start(clock.millis());

    const TickType_t idleTicks = pdMS_TO_TICKS(TaskConfig::PROCESSING_IDLE_TIMEOUT_MS);
    for (;;) {
        pg::SensorMessage msg;
        // Block (no CPU use) until a message arrives or the idle timeout passes.
        const bool got = queue_.popWait(msg, idleTicks);
        const uint32_t start = clock.millis();

        if (calibrationRequested_.exchange(false)) {
            pipeline_.requestCalibration(start);
        }

        uint32_t n = 0u;
        if (got) {
            pipeline_.ingest(msg, start);
            n = 1u;
            // Bounded drain: at most one queue's worth per wake, so a producer
            // that keeps publishing cannot keep this loop running forever.
            while (n < TaskConfig::PROCESSING_QUEUE_DEPTH && queue_.tryPop(msg)) {
                pipeline_.ingest(msg, clock.millis());
                ++n;
            }
        } else if (stats_.idleTimeouts != UINT32_MAX) {
            ++stats_.idleTimeouts;
        }

        pipeline_.buildFrame(clock.millis(), work_);
        frames_.publish(work_);

        const uint32_t elapsed = clock.millis() - start;
        stats_.lastProcessingTimeMs = elapsed;
        if (elapsed > stats_.maxProcessingTimeMs) stats_.maxProcessingTimeMs = elapsed;
        if (n > stats_.maxBatch) stats_.maxBatch = n;
        stats_.messages = (UINT32_MAX - stats_.messages < n) ? UINT32_MAX : stats_.messages + n;
        if (stats_.batches != UINT32_MAX) ++stats_.batches;
    }
}
