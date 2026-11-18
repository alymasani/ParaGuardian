/**
 ******************************************************************************
 * @file    SensorAcquisitionTask.hpp
 * @brief   The single task that samples every sensor and publishes the data.
 *
 * WHY ONE TASK AND NOT ONE PER SENSOR?
 *   Each extra task costs a stack (kilobytes), a TCB and context switches,
 *   and several high-priority tasks fighting over one SPI bus would need
 *   extra locking. A single periodic loop at 100 Hz that services every
 *   source in turn is deterministic, easy to reason about, and each source
 *   still runs at its own rate (IMU every 10 ms, baro every 20 ms, ...).
 *   Docs/RTOS_TIMING_ANALYSIS.md shows the cycle budget; if hardware
 *   measurements later show it is too tight, the IMUs can move to their own
 *   task without changing any driver.
 *
 * The per-cycle work lives in pg::AcquisitionScheduler (portable, unit-tested).
 * This class adds the RTOS part: a fixed-rate xTaskDelayUntil loop and
 * missed-deadline detection.
 *
 * It also owns a LatestSampleMailbox, subscribed to the hub, holding the most
 * recent sample of every sensor (for the health monitor and future safety
 * logic).
 *
 * Follows the SoarOS task pattern (singleton Inst(), InitTask(), static
 * RunTask() trampoline -> Run()), derives from SoarOS Task, but creates the
 * thread with xTaskCreateStatic() so its stack is NOT taken from the 15 KB
 * FreeRTOS heap.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_SENSOR_ACQUISITION_TASK_HPP
#define PARAGUARDIAN_TASKS_SENSOR_ACQUISITION_TASK_HPP

#include <cstddef>
#include <cstdint>

#include "Task.hpp"   // SoarOS

#include "Acquisition/AcquisitionScheduler.hpp"
#include "Acquisition/AcquisitionSources.hpp"
#include "Acquisition/LatestSampleMailbox.hpp"
#include "Acquisition/SensorDataHub.hpp"
#include "BSP/RtosClock.hpp"
#include "Tasks/RtosCriticalSection.hpp"
#include "Tasks/TaskConfig.hpp"

class SensorAcquisitionTask : public Task {
public:
    struct Stats {
        uint32_t cycles;
        uint32_t missedDeadlines;       ///< xTaskDelayUntil found the wake time already past
        uint32_t cycleOverruns;
        uint32_t maxCycleTimeMs;
        uint32_t lastCycleTimeMs;
        uint32_t initAttemptsDeferred;
    };

    static SensorAcquisitionTask& Inst()
    {
        static SensorAcquisitionTask inst;
        return inst;
    }

    /// Creates the RTOS thread. Call once, before the scheduler starts.
    void InitTask() override;

    /// Adds a sensor source. Only valid BEFORE InitTask(); returns false if full/late.
    bool registerSource(pg::IAcquisitionSource* source);

    pg::SensorDataHub& hub() { return hub_; }
    /// Latest sample per sensor (thread-safe reads from any task).
    const pg::LatestSampleMailbox& latest() const { return latest_; }
    /// Snapshot of the counters (each field is one aligned 32-bit word).
    Stats stats() const;
    size_t sourceCount() const { return scheduler_.sourceCount(); }
    pg::IAcquisitionSource* source(size_t i) const { return scheduler_.source(i); }
    TaskHandle_t handle() const { return rtTaskHandle; }

protected:
    static void RunTask(void* pvParams) { SensorAcquisitionTask::Inst().Run(pvParams); }
    void Run(void* pvParams);

private:
    SensorAcquisitionTask();  // Task(0): this task needs no SoarOS command queue
    SensorAcquisitionTask(const SensorAcquisitionTask&) = delete;
    SensorAcquisitionTask& operator=(const SensorAcquisitionTask&) = delete;

    pg::AcquisitionScheduler scheduler_;
    pg::SensorDataHub hub_{};
    pg::RtosCriticalSection latestLock_{};
    pg::LatestSampleMailbox latest_;
    uint32_t missedDeadlines_ = 0u;

    StackType_t stack_[TaskConfig::ACQUISITION_STACK_WORDS] = {};
    StaticTask_t tcb_{};
};

#endif  // PARAGUARDIAN_TASKS_SENSOR_ACQUISITION_TASK_HPP
