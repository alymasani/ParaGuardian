/**
 ******************************************************************************
 * @file    EmergencyAlertTask.hpp
 * @brief   Static FreeRTOS task that runs the EmergencyManager and the
 *          SafetyEventDispatcher (audio / LED schedulers).
 *
 * DATA FLOW
 *   SensorProcessingTask --CrashReportMailbox (latest)--------------\
 *   SensorProcessingTask --SafetyEventQueue<16> (transitions)------ +--> this task
 *   SensorProcessingTask --SafetySnapshotMailbox (latest)----------/
 *   any task --submitCommand()--> BoundedInbox<EmergencyCommand, 8>  (+ notify)
 *   transport --submitTransportReport()--> BoundedInbox<TransportReport, 8>
 *   this task --> EmergencyManager --> ISosTransport (UNCONFIGURED in firmware)
 *            --> JournaledEmergencyStore (queued; StorageTask commits it to flash)
 *            --> FlightRecorder (events / emergency / power records -> StorageTask)
 *            --> PowerManager (battery, 5 V rail, amplifier; switches UNCONFIGURED)
 *            --> SafetyEventDispatcher --> AudioAlertScheduler (UNCONFIGURED backend)
 *                                     --> LedPatternScheduler (UNCONFIGURED backend)
 *            --> emergency() / sosRequest() / dispatch() latest-value mailboxes
 *
 * WHY A SEPARATE TASK: the countdown, retries and alert timing must advance
 * even if sensor frames stop (a crash may break a sensor or stall
 * processing). It wakes on a fixed period (ulTaskNotifyTake with timeout,
 * no busy loop) or immediately on a command. Nothing here blocks: the
 * transport and audio backends only copy requests.
 *
 * WORK PER WAKE (bounded): <= 8 commands, <= 8 reports, one crash report,
 * one manager tick, <= 16 + 8 events, one dispatcher update.
 * Static allocation only. Large copies are members, not stack variables.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_EMERGENCY_ALERT_TASK_HPP
#define PARAGUARDIAN_TASKS_EMERGENCY_ALERT_TASK_HPP

#include <cstdint>

#include "Communications/Bluetooth/BleEmergencyBridge.hpp"
#include "Task.hpp"   // SoarOS

#include "Algorithms/Crash/CrashTypes.hpp"
#include "Algorithms/Emergency/EmergencyCommand.hpp"
#include "Algorithms/Emergency/EmergencyManager.hpp"
#include "Algorithms/Emergency/IEmergencyStore.hpp"
#include "Algorithms/Emergency/ISosTransport.hpp"
#include "Algorithms/Emergency/SosRequestMailbox.hpp"
#include "Algorithms/Landing/LandingSafetyTypes.hpp"
#include "Algorithms/Safety/SafetyEventDispatcher.hpp"
#include "Algorithms/Safety/SafetyEventQueue.hpp"
#include "Algorithms/Safety/ValueMailbox.hpp"
#include "Output/Audio/AudioAlertScheduler.hpp"
#include "Output/LED/LedPatternScheduler.hpp"
#include "Power/PowerManager.hpp"
#include "Tasks/RtosCriticalSection.hpp"
#include "Tasks/TaskConfig.hpp"

class EmergencyAlertTask : public Task {
public:
    struct Stats {
        uint32_t wakes;
        uint32_t commandWakes;        ///< woken early by a notification
        uint32_t lastWorkMs;          ///< 1 ms resolution
        uint32_t maxWorkMs;
        uint32_t maxEventsPerWake;
        uint32_t lateWakes;           ///< wake later than 2 periods after the previous one
        uint32_t maxWakeIntervalMs;
    };

    static EmergencyAlertTask& Inst()
    {
        static EmergencyAlertTask inst;
        return inst;
    }

    /// Creates the thread. Call after SensorProcessingTask::InitTask(), before osKernelStart().
    void InitTask() override;

    /// Any task (not ISR). Never blocks; false = inbox full (the command was dropped, counted).
    bool submitCommand(const pg::EmergencyCommand& cmd);
    /// The (future) transport task reports progress. Never blocks.
    bool submitTransportReport(const pg::TransportReport& report);

    const pg::ValueMailbox<pg::EmergencyStatus>& emergency() const { return emergencyOut_; }
    const pg::SosRequestMailbox& sosRequest() const { return sosOut_; }
    const pg::ValueMailbox<pg::ble::Diagnostics>& bleDiagnostics() const { return bleOut_; }
    const pg::ValueMailbox<pg::DispatchStatus>& dispatch() const { return dispatchOut_; }
    const pg::ValueMailbox<pg::PowerSnapshot>& power() const { return powerOut_; }
    uint32_t commandsDropped() const { return commands_.dropped(); }
    uint32_t reportsDropped() const { return reports_.dropped(); }
    uint32_t localEventBacklog() const { return static_cast<uint32_t>(localEvents_.size()); }
    Stats stats() const { return stats_; }
    TaskHandle_t handle() const { return rtTaskHandle; }

protected:
    static void RunTask(void* pvParams) { EmergencyAlertTask::Inst().Run(pvParams); }
    void Run(void* pvParams);

private:
    EmergencyAlertTask();
    EmergencyAlertTask(const EmergencyAlertTask&) = delete;
    EmergencyAlertTask& operator=(const EmergencyAlertTask&) = delete;

    // Backends: honestly UNCONFIGURED until the schematic / CYW20822 protocol / flash are verified.
    pg::ble::UnconfiguredPacketPort packetPort_{};
    pg::ble::EmergencyBridge transport_{packetPort_};
    pg::UnconfiguredAudioAlert audioBackend_{};
    pg::UnconfiguredStatusLed ledBackend_{};
    pg::UnconfiguredPowerSwitch rail5vSwitch_{};      ///< enable pin / polarity unknown
    pg::UnconfiguredPowerSwitch amplifierSwitch_{};
    pg::PowerManager power_;                          ///< battery thresholds unconfigured -> level Unknown

    pg::EmergencyManager manager_;
    pg::SafetyEventDispatcher dispatcher_;

    pg::RtosCriticalSection commandLock_{};
    pg::BoundedInbox<pg::EmergencyCommand, TaskConfig::EMERGENCY_COMMAND_DEPTH> commands_;
    pg::RtosCriticalSection reportLock_{};
    pg::BoundedInbox<pg::TransportReport, TaskConfig::EMERGENCY_REPORT_DEPTH> reports_;
    pg::RtosCriticalSection localEventLock_{};
    pg::SafetyEventQueue<TaskConfig::EMERGENCY_EVENT_DEPTH> localEvents_;   ///< manager -> dispatcher (+ future logger)

    pg::RtosCriticalSection emergencyLock_{};
    pg::ValueMailbox<pg::EmergencyStatus> emergencyOut_;
    pg::ValueMailbox<pg::ble::Diagnostics> bleOut_;
    pg::RtosCriticalSection sosLock_{};
    pg::SosRequestMailbox sosOut_;
    pg::RtosCriticalSection dispatchLock_{};
    pg::ValueMailbox<pg::DispatchStatus> dispatchOut_;
    pg::ValueMailbox<pg::PowerSnapshot> powerOut_;
    uint32_t lastPowerMs_ = 0u;

    pg::CrashReport crashCopy_{};        ///< ~0.2 KB, kept off the stack
    pg::SafetySnapshot safetyCopy_{};    ///< ~0.2 KB
    Stats stats_{};

    StackType_t stack_[TaskConfig::EMERGENCY_STACK_WORDS] = {};
    StaticTask_t tcb_{};
};

#endif  // PARAGUARDIAN_TASKS_EMERGENCY_ALERT_TASK_HPP
