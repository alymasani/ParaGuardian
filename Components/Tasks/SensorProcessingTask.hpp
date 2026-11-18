/**
 ******************************************************************************
 * @file    SensorProcessingTask.hpp
 * @brief   RTOS wrapper around pg::PreprocessingPipeline.
 *
 * DATA FLOW
 *   SensorAcquisitionTask --hub.publish()--> RtosSampleQueue<64> (DropOldest)
 *        --popWait--> PreprocessingPipeline::ingest() (every queued message)
 *        --buildFrame--> ProcessedFrameMailbox (latest frame, readers poll)
 *
 * WHY A SEPARATE TASK
 *   The acquisition task must keep its 10 ms rhythm. Validation, calibration,
 *   alignment, voting and the barometric pow() are done here, at a LOWER
 *   priority, so they can never delay a sensor read. The hub's publish() only
 *   copies 88 bytes into this task's queue and never blocks.
 *
 * QUEUE POLICY: DropOldest - for navigation the newest data matter; anything
 *   discarded is counted by the hub (overwritesFor) and shows up as sequence
 *   gaps (frame.samplesLostSinceLastFrame / quality::SamplesLost).
 *
 * The loop blocks in popWait() (no busy-waiting). It wakes when data arrives
 * or after PROCESSING_IDLE_TIMEOUT_MS, then drains at most the queue capacity
 * (bounded work per wake) and publishes one frame.
 *
 * Static allocation only (xTaskCreateStatic, xQueueCreateStatic).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_SENSOR_PROCESSING_TASK_HPP
#define PARAGUARDIAN_TASKS_SENSOR_PROCESSING_TASK_HPP

#include <atomic>
#include <cstdint>

#include "Task.hpp"   // SoarOS

#include "Acquisition/SensorDataHub.hpp"
#include "Processing/PreprocessingPipeline.hpp"
#include "Processing/ProcessedFrameMailbox.hpp"
#include "Tasks/RtosCriticalSection.hpp"
#include "Tasks/RtosSampleQueue.hpp"
#include "Tasks/TaskConfig.hpp"

class SensorProcessingTask : public Task {
public:
    struct Stats {
        uint32_t batches;               ///< loop iterations (one frame each)
        uint32_t messages;              ///< messages taken from the queue
        uint32_t idleTimeouts;          ///< wakes with no data (sensors silent)
        uint32_t maxBatch;              ///< most messages drained in one wake
        uint32_t lastProcessingTimeMs;  ///< ingest + buildFrame of the last batch (1 ms resolution)
        uint32_t maxProcessingTimeMs;
    };

    static SensorProcessingTask& Inst()
    {
        static SensorProcessingTask inst;
        return inst;
    }

    /**
     * Creates the queue, subscribes it to @p hub (DropOldest) and creates the
     * thread. Call once, after osKernelInitialize(), before osKernelStart().
     */
    void InitTask(pg::SensorDataHub& hub);
    void InitTask() override;   // not used: the hub is required (asserts)

    /// Asks for a new calibration session (any task; takes effect at the next wake).
    void requestCalibration() { calibrationRequested_.store(true); }

    /// Newest processed frame (thread-safe copy).
    const pg::ProcessedFrameMailbox& frames() const { return frames_; }
    /// Counter snapshot (each field one aligned 32-bit word).
    Stats stats() const;
    TaskHandle_t handle() const { return rtTaskHandle; }
    const pg::ISampleQueue& queue() const { return queue_; }

protected:
    static void RunTask(void* pvParams) { SensorProcessingTask::Inst().Run(pvParams); }
    void Run(void* pvParams);

private:
    SensorProcessingTask();
    SensorProcessingTask(const SensorProcessingTask&) = delete;
    SensorProcessingTask& operator=(const SensorProcessingTask&) = delete;

    pg::RtosSampleQueue<TaskConfig::PROCESSING_QUEUE_DEPTH> queue_{};
    pg::PreprocessingPipeline pipeline_;
    pg::RtosCriticalSection frameLock_{};
    pg::ProcessedFrameMailbox frames_;
    pg::ProcessedSensorFrame work_{};          // built here, then copied into frames_
    std::atomic<bool> calibrationRequested_{false};
    Stats stats_{};

    StackType_t stack_[TaskConfig::PROCESSING_STACK_WORDS] = {};
    StaticTask_t tcb_{};
};

#endif  // PARAGUARDIAN_TASKS_SENSOR_PROCESSING_TASK_HPP
