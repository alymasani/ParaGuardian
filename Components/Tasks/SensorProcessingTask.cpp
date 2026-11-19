/**
 ******************************************************************************
 * @file    SensorProcessingTask.cpp
 * @brief   See SensorProcessingTask.hpp.
 ******************************************************************************
 */
#include "Tasks/SensorProcessingTask.hpp"

#include "SystemDefines.hpp"
#include "Acquisition/SensorDataHub.hpp"
#include "Algorithms/Attitude/AttitudeFrameAdapter.hpp"
#include "Algorithms/Navigation/NavigationInputAdapter.hpp"
#include "Algorithms/Crash/CrashInputAdapter.hpp"
#include "Algorithms/Safety/SafetyInput.hpp"
#include "BSP/RtosClock.hpp"

SensorProcessingTask::SensorProcessingTask()
    : Task(0u),  // no SoarOS command queue: data arrives through queue_
      pipeline_(pg::PreprocessingConfig{}),
      frames_(frameLock_),
      attitude_(pg::AttitudeConfig{}),
      attitudeOut_(attitudeLock_),
      navModel_(pg::ConstantAccelerationModel::fromConfig(pg::NavigationConfig{})),
      navigation_(pg::NavigationConfig{}, navModel_),
      navigationOut_(navigationLock_),
      safety_(pg::SafetyConfig{}),
      safetyOut_(safetyLock_),
      safetyEvents_(safetyEventLock_),
      crash_(pg::CrashConfig{}),
      crashOut_(crashLock_)
{
    safety_.addSink(&safetyEvents_);   // before the scheduler starts: no locking needed
}

void SensorProcessingTask::InitTask()
{
    SOAR_ASSERT(false, "SensorProcessingTask::InitTask(hub) must be used");
}

void SensorProcessingTask::InitTask(pg::SensorDataHub& hub)
{
    SOAR_ASSERT(rtTaskHandle == nullptr, "SensorProcessingTask created twice");
    SOAR_ASSERT(attitude_.configValid(), "SensorProcessingTask: invalid AttitudeConfig");
    SOAR_ASSERT(navigation_.configValid(), "SensorProcessingTask: invalid NavigationConfig / process model");
    SOAR_ASSERT(safety_.configValid(), "SensorProcessingTask: invalid SafetyConfig");
    SOAR_ASSERT(crash_.configValid(), "SensorProcessingTask: invalid CrashConfig");

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

        // Attitude: one estimator update per frame. A gyro sample is integrated
        // only if its timestamp is newer than the last one (no double integration).
        attitude_.update(pg::attitudeInputFromFrame(work_));
        attitudeOut_.publish(attitude_.latest());

        // Vertical navigation (UKF): one update per frame, after attitude (it uses
        // this frame's gravity-compensated vertical acceleration). Each barometer,
        // acceleration and GNSS sample is assimilated once, at its own timestamp.
        navigation_.update(pg::navigationInputFrom(work_, attitude_.latest()));
        navigationOut_.publish(navigation_.latest());

        // Flight / landing safety: clearance -> phase -> warnings, from THIS frame's
        // attitude and navigation. Events (transitions only) go to safetyEvents_.
        safetyOut_.publish(safety_.update(pg::safetyInputFrom(work_, attitude_.latest(), navigation_.latest())));

        // Crash detection: per-IMU impact peaks of THIS frame (before any smoothing) plus the
        // context above. Its events share the safety emitter (one event-id sequence per task).
        pg::updateIncidentContext(incident_, work_, navigation_.latest());
        crashReport_.status = crash_.update(
            pg::crashInputFrom(work_, attitude_.latest(), safety_.latest(), navigation_.latest()),
            safety_.eventEmitter());
        crashReport_.context = incident_;
        crashReport_.publishedMs = clock.millis();
        crashOut_.publish(crashReport_);

        const uint32_t elapsed = clock.millis() - start;
        stats_.lastProcessingTimeMs = elapsed;   // includes attitude, navigation and safety updates
        if (elapsed > stats_.maxProcessingTimeMs) stats_.maxProcessingTimeMs = elapsed;
        if (n > stats_.maxBatch) stats_.maxBatch = n;
        stats_.messages = (UINT32_MAX - stats_.messages < n) ? UINT32_MAX : stats_.messages + n;
        if (stats_.batches != UINT32_MAX) ++stats_.batches;
    }
}
