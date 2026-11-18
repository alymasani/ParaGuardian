/**
 ******************************************************************************
 * @file    SensorAcquisitionTask.cpp
 * @brief   See SensorAcquisitionTask.hpp.
 ******************************************************************************
 */
#include "Tasks/SensorAcquisitionTask.hpp"

#include "SystemDefines.hpp"

SensorAcquisitionTask::SensorAcquisitionTask()
    : Task(0u),
      scheduler_(pg::bsp::systemClock(), TaskConfig::ACQUISITION_PERIOD_MS, TaskConfig::INIT_ATTEMPTS_PER_CYCLE),
      latest_(latestLock_)
{
    // Constructed in paraguardian_create_tasks() (before the scheduler), so
    // subscribing here needs no locking.
    hub_.subscribeMailbox(&latest_);
}

void SensorAcquisitionTask::InitTask()
{
    SOAR_ASSERT(rtTaskHandle == nullptr, "SensorAcquisitionTask created twice");

    rtTaskHandle = xTaskCreateStatic(&SensorAcquisitionTask::RunTask,
                                     "SensorAcq",
                                     TaskConfig::ACQUISITION_STACK_WORDS,  // WORDS, not bytes
                                     this,
                                     TaskConfig::ACQUISITION_PRIORITY,
                                     stack_,
                                     &tcb_);

    // With static allocation this can only fail if a buffer pointer is null.
    SOAR_ASSERT(rtTaskHandle != nullptr, "SensorAcquisitionTask: xTaskCreateStatic failed");
}

bool SensorAcquisitionTask::registerSource(pg::IAcquisitionSource* source)
{
    // Sources are read by the task without locking, so the list must be
    // complete before the task exists.
    if (rtTaskHandle != nullptr) {
        return false;
    }
    return scheduler_.add(source);
}

SensorAcquisitionTask::Stats SensorAcquisitionTask::stats() const
{
    const pg::AcquisitionScheduler::Stats& s = scheduler_.stats();
    return Stats{s.cycles, missedDeadlines_, s.cycleOverruns, s.maxCycleTimeMs, s.lastCycleTimeMs,
                 s.initAttemptsDeferred};
}

void SensorAcquisitionTask::Run(void* /*pvParams*/)
{
    const TickType_t period = pdMS_TO_TICKS(TaskConfig::ACQUISITION_PERIOD_MS);

    if (scheduler_.sourceCount() == 0u) {
        SOAR_PRINT("SensorAcq: no sensors registered (board pin map not defined yet)\n");
    }

    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        // Sleep until exactly one period after the previous wake time (fixed
        // rate, no drift). xTaskDelayUntil returns pdFALSE when it did not need
        // to sleep because that time had already passed = a missed deadline.
        if (xTaskDelayUntil(&lastWake, period) == pdFALSE) {
            ++missedDeadlines_;
            // Re-anchor to "now". Otherwise lastWake stays in the past and the
            // following calls also return immediately, running a burst of
            // back-to-back catch-up cycles (and counting each as a miss).
            lastWake = xTaskGetTickCount();
        }
        scheduler_.runCycle(hub_);
    }
}
