/**
 ******************************************************************************
 * @file    EmergencyAlertTask.cpp
 * @brief   See EmergencyAlertTask.hpp.
 ******************************************************************************
 */
#include "Tasks/EmergencyAlertTask.hpp"

#include <cmath>

#include "SystemDefines.hpp"
#include "BSP/RtosClock.hpp"
#include "Processing/TimeBase.hpp"
#include "Tasks/SensorProcessingTask.hpp"
#include "Tasks/StorageTask.hpp"
#include "Tasks/SystemHealth.hpp"

EmergencyAlertTask::EmergencyAlertTask()
    : Task(0u),   // no SoarOS command queue: commands arrive through commands_
      power_(pg::BatteryConfig{}, pg::PowerPolicyConfig{}, &rail5vSwitch_, &amplifierSwitch_),
      manager_(pg::EmergencyConfig{}, transport_, &StorageTask::Inst().store()),
      dispatcher_(pg::DispatcherConfig{}, &audioBackend_, &ledBackend_),
      commands_(commandLock_),
      reports_(reportLock_),
      localEvents_(localEventLock_),
      emergencyOut_(emergencyLock_),
      bleOut_(emergencyLock_),
      sosOut_(sosLock_),
      dispatchOut_(dispatchLock_),
      powerOut_(dispatchLock_)
{
    manager_.addSink(&localEvents_);   // before the scheduler starts: no locking needed
}

void EmergencyAlertTask::InitTask()
{
    SOAR_ASSERT(rtTaskHandle == nullptr, "EmergencyAlertTask created twice");
    SOAR_ASSERT(manager_.configValid(), "EmergencyAlertTask: invalid EmergencyConfig");
    SOAR_ASSERT(dispatcher_.configValid(), "EmergencyAlertTask: invalid DispatcherConfig");
    rtTaskHandle = xTaskCreateStatic(&EmergencyAlertTask::RunTask,
                                     "EmergencyAlert",
                                     TaskConfig::EMERGENCY_STACK_WORDS,   // WORDS
                                     this,
                                     TaskConfig::EMERGENCY_PRIORITY,
                                     stack_,
                                     &tcb_);
    SOAR_ASSERT(rtTaskHandle != nullptr, "EmergencyAlertTask: xTaskCreateStatic failed");
}

bool EmergencyAlertTask::submitCommand(const pg::EmergencyCommand& cmd)
{
    const bool ok = commands_.push(cmd);
    if (rtTaskHandle != nullptr) xTaskNotifyGive(rtTaskHandle);   // wake now (no busy wait)
    return ok;
}

bool EmergencyAlertTask::submitTransportReport(const pg::TransportReport& report)
{
    const bool ok = reports_.push(report);
    if (rtTaskHandle != nullptr) xTaskNotifyGive(rtTaskHandle);
    return ok;
}

void EmergencyAlertTask::Run(void* /*pvParams*/)
{
    pg::IClock& clock = pg::bsp::systemClock();
    SensorProcessingTask& proc = SensorProcessingTask::Inst();
    StorageTask& storage = StorageTask::Inst();
    pg::FlightRecorder& recorder = storage.recorder();
    // Wait (bounded) for the storage task to mount the journal, then restore the incident.
    // Unconfigured / failed flash mounts as Unavailable: restore() then reports no persistence.
    const uint32_t waitStart = clock.millis();
    while (!storage.store().mounted() &&
           pg::timebase::ageMs(waitStart, clock.millis()) < TaskConfig::STORAGE_MOUNT_WAIT_MS) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    manager_.restore(clock.millis());
    storage.notify();
    {
        pg::StorageStatus ss{};
        storage.status().read(ss);
        recorder.logBoot(pg::g_systemHealth.resetFlags, ss.bootCount, ss.journalMount, ss.criticalMount,
                         ss.flightMount, clock.millis());
    }
    power_.start(clock.millis());

    const TickType_t period = pdMS_TO_TICKS(TaskConfig::EMERGENCY_PERIOD_MS);
    uint32_t lastWake = clock.millis();
    for (;;) {
        // Block until notified (command / report) or one period passed. Never a busy loop.
        const uint32_t notified = ulTaskNotifyTake(pdTRUE, period);
        const uint32_t now = clock.millis();
        if (notified != 0u && stats_.commandWakes != UINT32_MAX) ++stats_.commandWakes;
        const uint32_t interval = pg::timebase::ageMs(lastWake, now);
        lastWake = now;
        if (interval > stats_.maxWakeIntervalMs) stats_.maxWakeIntervalMs = interval;
        if (interval > 2u * TaskConfig::EMERGENCY_PERIOD_MS && stats_.lateWakes != UINT32_MAX) ++stats_.lateWakes;

        // 1. Commands first: a cancel received before this wake is applied before any expiry.
        pg::EmergencyCommand cmd{};
        for (size_t i = 0; i < TaskConfig::EMERGENCY_COMMAND_DEPTH && commands_.pop(cmd); ++i) {
            if (manager_.handleCommand(cmd, now) == pg::CommandResult::ForwardToAlerts) dispatcher_.acknowledge(now);
        }
        // 2. Transport / phone reports.
        pg::TransportReport rep{};
        for (size_t i = 0; i < TaskConfig::EMERGENCY_REPORT_DEPTH && reports_.pop(rep); ++i) {
            manager_.handleReport(rep, now);
        }
        transport_.receive(manager_, now);
        if (transport_.takeAlertAcknowledgment()) dispatcher_.acknowledge(now);
        // 3. Latest crash detector report (heartbeat + incident context).
        const bool haveCrash = proc.crashReports().read(crashCopy_);
        if (haveCrash) manager_.observeCrash(crashCopy_, now);
        // 4. Time-driven transitions: countdown, retries, timeouts (independent of sensor frames).
        manager_.tick(now);
        if (storage.store().hasPending()) storage.notify();   // the journal commit runs in StorageTask
        recorder.logEmergency(manager_.status(), now);

        // 5. Events (bounded drain): processing-task events, then this task's own.
        pg::SafetyEvent e{};
        uint32_t nEvents = 0u;
        while (nEvents < TaskConfig::EMERGENCY_MAX_EVENTS_PER_WAKE && proc.safetyEvents().pop(e)) {
            dispatcher_.consume(e, now);
            recorder.logEvent(e, now);
            ++nEvents;
        }
        for (size_t i = 0; i < TaskConfig::EMERGENCY_EVENT_DEPTH && localEvents_.pop(e); ++i) {
            dispatcher_.consume(e, now);
            recorder.logEvent(e, now);
            ++nEvents;
        }
        if (nEvents > stats_.maxEventsPerWake) stats_.maxEventsPerWake = nEvents;

        // 6. Alerts: state from the latest snapshot / crash / emergency status, then outputs.
        const bool haveSafety = proc.safety().read(safetyCopy_);
        const pg::EmergencyState es = manager_.status().state;
        const bool emergencyOpen = es != pg::EmergencyState::Idle && es != pg::EmergencyState::Unavailable;
        const bool inFlight = haveSafety && pg::isAirbornePhase(safetyCopy_.phase);
        if (pg::timebase::ageMs(lastPowerMs_, now) >= 1000u) {   // power policy at 1 Hz
            lastPowerMs_ = now;
            pg::PowerInputs pin{};   // no battery ADC / charger pins configured: every input Unknown
            pin.sampleMs = now;
            const pg::PowerContext pctx{inFlight, emergencyOpen,
                                        dispatcher_.status().audioState == pg::OutputState::Active};
            const pg::PowerSnapshot& ps = power_.update(pin, pctx, now);
            manager_.observeBattery(ps.millivolts, ps.level != pg::BatteryLevel::Unknown, now);
            powerOut_.publish(ps);
            recorder.logPower(pg::PowerLogSummary{static_cast<uint8_t>(ps.level), ps.millivolts, ps.flags,
                                                  static_cast<uint8_t>(ps.rail5v), static_cast<uint8_t>(ps.amplifier),
                                                  static_cast<uint8_t>(ps.lidarPowerRequested ? 1u : 0u),
                                                  static_cast<uint8_t>(ps.supply)},
                              now);
            pg::SystemAlertInputs sys{};
            sys.batteryLevel = static_cast<uint8_t>(ps.level);
            sys.phoneLinkKnown = true;
            sys.phoneLinkUp = transport_.linkUp();
            sys.inFlight = inFlight;
            sys.storageDegraded = manager_.status().persist == pg::PersistState::Failed;
            sys.supervisorFault = pg::g_systemHealth.supervisorStalledMask != 0u;
            dispatcher_.setSystemInputs(sys, now);
        }
        const pg::DispatcherInputs in{haveSafety ? &safetyCopy_ : nullptr, haveCrash ? &crashCopy_.status : nullptr,
                                      &manager_.status()};
        dispatchOut_.publish(dispatcher_.update(now, in));
        pg::ble::Telemetry telemetry{};
        if (haveSafety && pg::timebase::ageMs(safetyCopy_.timestampMs, now) <= 2000u) {
            telemetry.flight = static_cast<uint8_t>(safetyCopy_.phase);
            telemetry.navigation = static_cast<uint8_t>(safetyCopy_.navState);
            telemetry.warnings = safetyCopy_.activeWarningMask;
            telemetry.validity |= 4u;
        }
        if (haveCrash && pg::timebase::ageMs(crashCopy_.publishedMs, now) <= 2000u) {
            const auto& context = crashCopy_.context;
            telemetry.quality = context.frameQualityFlags;
            if (pg::timebase::ageMs(context.navTimestampMs, now) <= 2000u) {
                if (context.navAltitudeValid && std::isfinite(context.navAltitude_m) &&
                    std::fabs(context.navAltitude_m) < 1000000.f) {
                    telemetry.altitudeDm = static_cast<int32_t>(context.navAltitude_m * 10.f);
                    telemetry.validity |= 1u;
                }
                if (context.navVelocityValid && std::isfinite(context.navVerticalVelocity_mps) &&
                    std::fabs(context.navVerticalVelocity_mps) < 10000.f) {
                    telemetry.speedCmps = static_cast<int32_t>(context.navVerticalVelocity_mps * 100.f);
                    telemetry.validity |= 2u;
                }
            }
        }
        transport_.transmit(manager_, telemetry, now);
        emergencyOut_.publish(manager_.status());
        bleOut_.publish(transport_.diagnostics());
        sosOut_.publish(manager_.pendingRecord());

        const uint32_t work = pg::timebase::ageMs(now, clock.millis());
        stats_.lastWorkMs = work;
        if (work > stats_.maxWorkMs) stats_.maxWorkMs = work;
        if (stats_.wakes != UINT32_MAX) ++stats_.wakes;
    }
}
