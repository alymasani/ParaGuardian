/**
 ******************************************************************************
 * @file    StorageTask.hpp
 * @brief   Static, low-priority FreeRTOS task that owns the external flash
 *          and runs the StorageService (emergency journal first, then the
 *          critical log, then the flight log).
 *
 * PRODUCERS NEVER BLOCK: the emergency manager saves into the
 * JournaledEmergencyStore (RAM copy + ticket), the processing / emergency
 * tasks push records into bounded LogQueues through the FlightRecorder.
 * Only this task touches the flash.
 *
 * FLASH BACKEND: UnconfiguredNorFlash until the W25Q128 SPI instance and chip
 * select are known (no schematic). The journal then mounts as Unavailable,
 * the store reports it, and the EmergencyManager runs with
 * PersistenceUnavailable / IncidentIdVolatile, exactly as before Milestone 8.
 * Wiring the real chip = construct W25q128 + W25qNorFlash in BoardConfig and
 * pass it here (MILESTONE8_HARDWARE_BRINGUP.md section 1).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_STORAGE_TASK_HPP
#define PARAGUARDIAN_TASKS_STORAGE_TASK_HPP

#include <cstdint>

#include "Task.hpp"   // SoarOS

#include "Algorithms/Safety/ValueMailbox.hpp"
#include "Storage/FlightRecorder.hpp"
#include "Storage/JournaledEmergencyStore.hpp"
#include "Storage/LogRecords.hpp"
#include "Storage/NorFlash.hpp"
#include "Storage/StorageService.hpp"
#include "Tasks/RtosCriticalSection.hpp"
#include "Tasks/TaskConfig.hpp"

class StorageTask : public Task {
public:
    static StorageTask& Inst()
    {
        static StorageTask inst;
        return inst;
    }

    void InitTask() override;

    pg::JournaledEmergencyStore& store() { return store_; }
    pg::FlightRecorder& recorder() { return recorder_; }
    /// Wake the task now (an emergency record was queued). Any task, not ISR.
    void notify();
    const pg::ValueMailbox<pg::StorageStatus>& status() const { return statusOut_; }
    uint32_t progress() const { return progress_; }   ///< supervisor heartbeat
    TaskHandle_t handle() const { return rtTaskHandle; }
    pg::LogQueueStats criticalQueue() const { return critical_.stats(); }
    pg::LogQueueStats telemetryQueue() const { return telemetry_.stats(); }

protected:
    static void RunTask(void* pvParams) { StorageTask::Inst().Run(pvParams); }
    void Run(void* pvParams);

private:
    StorageTask();
    StorageTask(const StorageTask&) = delete;
    StorageTask& operator=(const StorageTask&) = delete;

    pg::UnconfiguredNorFlash flash_{};   ///< honest: no flash wiring known yet
    pg::RtosCriticalSection storeLock_{};
    pg::JournaledEmergencyStore store_;
    pg::RtosCriticalSection logLock_{};
    pg::LogQueue<TaskConfig::STORAGE_CRITICAL_QUEUE_DEPTH> critical_;
    pg::LogQueue<TaskConfig::STORAGE_TELEMETRY_QUEUE_DEPTH> telemetry_;
    pg::StorageService service_;
    pg::FlightRecorder recorder_;
    pg::RtosCriticalSection statusLock_{};
    pg::ValueMailbox<pg::StorageStatus> statusOut_;
    volatile uint32_t progress_ = 0u;

    StackType_t stack_[TaskConfig::STORAGE_STACK_WORDS] = {};
    StaticTask_t tcb_{};
};

#endif  // PARAGUARDIAN_TASKS_STORAGE_TASK_HPP
