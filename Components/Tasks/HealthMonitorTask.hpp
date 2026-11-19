/**
 ******************************************************************************
 * @file    HealthMonitorTask.hpp
 * @brief   Low-priority task that periodically collects FreeRTOS and sensor
 *          diagnostics into g_systemHealth.
 *
 * Collected every TaskConfig::HEALTH_PERIOD_MS:
 *   - stack high-water mark of every watched task (smallest free stack seen)
 *   - FreeRTOS heap free / minimum-ever-free
 *   - acquisition loop cycles, missed deadlines, overruns, worst cycle time
 *   - samples published / dropped, subscriber queue occupancy
 *   - per-sensor success/failure counters
 *   - preprocessing task counters and a summary of the newest ProcessedSensorFrame
 *
 * Uses the SoarOS Command queue: other tasks can send
 *   Command(TASK_SPECIFIC_COMMAND, HEALTH_CMD_COLLECT_NOW)
 * to force an immediate refresh.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_HEALTH_MONITOR_TASK_HPP
#define PARAGUARDIAN_TASKS_HEALTH_MONITOR_TASK_HPP

#include <cstdint>

#include "Task.hpp"   // SoarOS

#include "Algorithms/Attitude/AttitudeTypes.hpp"
#include "Algorithms/Crash/CrashTypes.hpp"
#include "Algorithms/Emergency/EmergencyTypes.hpp"
#include "Algorithms/Safety/AlertTypes.hpp"
#include "Algorithms/Navigation/NavigationTypes.hpp"
#include "Algorithms/Landing/LandingSafetyTypes.hpp"
#include "Processing/ProcessedSensorFrame.hpp"
#include "Tasks/SystemHealth.hpp"
#include "Tasks/TaskConfig.hpp"

enum HEALTH_TASK_COMMANDS : uint16_t {
    HEALTH_CMD_NONE = 0,
    HEALTH_CMD_COLLECT_NOW,
};

class HealthMonitorTask : public Task {
public:
    static HealthMonitorTask& Inst()
    {
        static HealthMonitorTask inst;
        return inst;
    }

    void InitTask() override;

    /// Adds a task to the stack watch list. Call before the scheduler starts.
    bool watchTask(TaskHandle_t handle, const char* name);

    TaskHandle_t handle() const { return rtTaskHandle; }

protected:
    static void RunTask(void* pvParams) { HealthMonitorTask::Inst().Run(pvParams); }
    void Run(void* pvParams);

private:
    HealthMonitorTask() : Task(TaskConfig::HEALTH_QUEUE_DEPTH) {}
    HealthMonitorTask(const HealthMonitorTask&) = delete;
    HealthMonitorTask& operator=(const HealthMonitorTask&) = delete;

    void handleCommand(Command& cm);
    void collect();

    struct Watched {
        TaskHandle_t handle;
        const char* name;
    };
    Watched watched_[pg::kMaxWatchedTasks] = {};
    uint8_t watchedCount_ = 0u;
    pg::ProcessedSensorFrame frameCopy_{};   ///< ~0.5 KB: kept off the 2 KiB task stack
    pg::AttitudeEstimate attitudeCopy_{};    ///< ~0.2 KB: likewise
    pg::NavigationEstimate navigationCopy_{};   ///< ~0.3 KB: likewise
    pg::SafetySnapshot safetyCopy_{};           ///< ~0.2 KB: likewise
    pg::CrashReport crashCopy_{};               ///< ~0.2 KB: likewise
    pg::EmergencyStatus emergencyCopy_{};       ///< ~0.2 KB: likewise
    pg::DispatchStatus dispatchCopy_{};         ///< ~0.1 KB: likewise

    StackType_t stack_[TaskConfig::HEALTH_STACK_WORDS] = {};
    StaticTask_t tcb_{};
};

/* FreeRTOS hook forwarders: see Components/SysCore/FaultHooks.h. */

#endif  // PARAGUARDIAN_TASKS_HEALTH_MONITOR_TASK_HPP
