/**
 ******************************************************************************
 * @file    StorageTask.cpp
 * @brief   See StorageTask.hpp.
 ******************************************************************************
 */
#include "Tasks/StorageTask.hpp"

#include "SystemDefines.hpp"
#include "BSP/RtosClock.hpp"

StorageTask::StorageTask()
    : Task(0u),   // no SoarOS command queue: work arrives through the store / log queues
      store_(storeLock_),
      critical_(logLock_),
      telemetry_(logLock_),
      service_(flash_, store_, critical_, telemetry_, pg::StorageServiceConfig{}, &pg::bsp::systemClock()),
      recorder_(critical_, telemetry_, pg::LogProfile::Normal),
      statusOut_(statusLock_)
{
}

void StorageTask::InitTask()
{
    SOAR_ASSERT(rtTaskHandle == nullptr, "StorageTask created twice");
    rtTaskHandle = xTaskCreateStatic(&StorageTask::RunTask, "Storage", TaskConfig::STORAGE_STACK_WORDS, this,
                                     TaskConfig::STORAGE_PRIORITY, stack_, &tcb_);
    SOAR_ASSERT(rtTaskHandle != nullptr, "StorageTask: xTaskCreateStatic failed");
}

void StorageTask::notify()
{
    if (rtTaskHandle != nullptr) xTaskNotifyGive(rtTaskHandle);
}

void StorageTask::Run(void* /*pvParams*/)
{
    pg::IClock& clock = pg::bsp::systemClock();
    service_.start(clock.millis());   // publishes the journal mount (also "Unavailable") to the store
    statusOut_.publish(service_.status());
    for (;;) {
        // One bounded unit per step; keep going while work is ready, then sleep (no busy loop).
        bool more = true;
        for (uint32_t i = 0; i < 8u && more; ++i) {
            more = service_.step(clock.millis());
            ++progress_;
        }
        statusOut_.publish(service_.status());
        if (!more) ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TaskConfig::STORAGE_IDLE_MS));
        else taskYIELD();
        ++progress_;
    }
}
