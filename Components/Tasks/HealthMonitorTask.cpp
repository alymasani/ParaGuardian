/**
 ******************************************************************************
 * @file    HealthMonitorTask.cpp
 * @brief   See HealthMonitorTask.hpp.
 ******************************************************************************
 */
#include "Tasks/HealthMonitorTask.hpp"

#include "SystemDefines.hpp"
#include "Command.hpp"
#include "Acquisition/SampleFreshness.hpp"
#include "BSP/RtosClock.hpp"
#include "Processing/TimeBase.hpp"
#include "Tasks/EmergencyAlertTask.hpp"
#include "Tasks/SensorAcquisitionTask.hpp"
#include "Tasks/SensorProcessingTask.hpp"
#include "Tasks/StorageTask.hpp"

namespace pg {
// Zero-initialised global; __attribute__((used)) keeps it in the image even if
// nothing else reads it, so the debugger can always find it.
__attribute__((used)) SystemHealthSnapshot g_systemHealth = {};
extern volatile uint32_t g_tickFallbackIncrements;  // Components/SysCore/HalTickFallback.cpp
}  // namespace pg

namespace {
/// value * scale rounded to int32; non-finite or out-of-range -> INT32_MIN (debugger-visible "invalid").
int32_t toScaledInt(float value, float scale)
{
    const float v = value * scale;
    if (!(v > -2.0e9f && v < 2.0e9f)) {  // false for NaN as well
        return INT32_MIN;
    }
    return static_cast<int32_t>(v >= 0.0f ? v + 0.5f : v - 0.5f);
}
}  // namespace

void HealthMonitorTask::InitTask()
{
    SOAR_ASSERT(rtTaskHandle == nullptr, "HealthMonitorTask created twice");
    SOAR_ASSERT(qEvtQueue != nullptr, "HealthMonitorTask queue allocation failed");

    rtTaskHandle = xTaskCreateStatic(&HealthMonitorTask::RunTask,
                                     "Health",
                                     TaskConfig::HEALTH_STACK_WORDS,
                                     this,
                                     TaskConfig::HEALTH_PRIORITY,
                                     stack_,
                                     &tcb_);
    SOAR_ASSERT(rtTaskHandle != nullptr, "HealthMonitorTask: xTaskCreateStatic failed");
    watchTask(rtTaskHandle, "Health");
    // Heartbeat supervision (policy only; the hardware watchdog is NOT enabled). Timeouts are
    // well above each task's period because this task samples the counters once a second.
    supAcq_ = supervisor_.add("SensorAcq", 2000u, pg::TaskCriticality::Critical);
    supProc_ = supervisor_.add("SensorProc", 2000u, pg::TaskCriticality::Critical);
    supEmergency_ = supervisor_.add("EmergencyAlert", 2000u, pg::TaskCriticality::Critical);
    supStorage_ = supervisor_.add("Storage", 5000u, pg::TaskCriticality::Storage);
}

bool HealthMonitorTask::watchTask(TaskHandle_t handle, const char* name)
{
    if (handle == nullptr || watchedCount_ >= pg::kMaxWatchedTasks) {
        ++pg::g_systemHealth.startupErrors;
        return false;
    }
    watched_[watchedCount_++] = Watched{handle, name};
    return true;
}

void HealthMonitorTask::Run(void* /*pvParams*/)
{
    collect();
    for (;;) {
        Command cm;
        // Block for up to one period waiting for a command. Either way we
        // collect afterwards, so the snapshot refreshes at least once a period.
        if (qEvtQueue->Receive(cm, TaskConfig::HEALTH_PERIOD_MS)) {
            handleCommand(cm);
        }
        collect();
    }
}

void HealthMonitorTask::handleCommand(Command& cm)
{
    // Whatever the command, collect() runs right after this returns.
    if (cm.GetCommand() != TASK_SPECIFIC_COMMAND || cm.GetTaskCommand() != HEALTH_CMD_COLLECT_NOW) {
        SOAR_PRINT("Health: unsupported command %d/%d\n", cm.GetCommand(), cm.GetTaskCommand());
    }
    cm.Reset();  // SoarOS rule: every received Command must be Reset() (frees attached data)
}

void HealthMonitorTask::collect()
{
    pg::SystemHealthSnapshot& h = pg::g_systemHealth;
    SensorAcquisitionTask& acq = SensorAcquisitionTask::Inst();
    const uint32_t now = pg::bsp::systemClock().millis();

    h.uptimeMs = now;
    ++h.reportCount;
    h.freeHeapBytes = static_cast<uint32_t>(xPortGetFreeHeapSize());
    h.minimumEverFreeHeapBytes = static_cast<uint32_t>(xPortGetMinimumEverFreeHeapSize());
    h.tickFallbackIncrements = pg::g_tickFallbackIncrements;

    // The acquisition task updates these counters without a lock. Each is a
    // single aligned 32-bit word, so a read is never torn; the snapshot as a
    // whole may mix values from adjacent cycles, which is fine for diagnostics.
    const SensorAcquisitionTask::Stats st = acq.stats();
    h.acquisitionCycles = st.cycles;
    h.missedDeadlines = st.missedDeadlines;
    h.cycleOverruns = st.cycleOverruns;
    h.maxCycleTimeMs = st.maxCycleTimeMs;
    h.lastCycleTimeMs = st.lastCycleTimeMs;
    h.initAttemptsDeferred = st.initAttemptsDeferred;

    const pg::SensorDataHub& hub = acq.hub();
    h.samplesPublished = hub.publishedCount();
    h.sampleDrops = hub.totalDrops();
    h.sampleOverwrites = hub.totalOverwrites();
    h.queueCount = static_cast<uint8_t>(hub.subscriberCount() < pg::kMaxReportedQueues ? hub.subscriberCount()
                                                                                      : pg::kMaxReportedQueues);
    for (uint8_t i = 0; i < h.queueCount; ++i) {
        const pg::ISampleQueue* q = hub.subscriber(i);
        h.queueUsed[i] = static_cast<uint32_t>(q->size());
        h.queueCapacity[i] = static_cast<uint32_t>(q->capacity());
    }

    // Staleness: age of the newest sample of every sensor (mailbox read is locked).
    for (uint8_t id = 0; id < static_cast<uint8_t>(pg::SensorId::Count); ++id) {
        pg::SensorMessage m{};
        const pg::SampleHeader* hdr = nullptr;
        if (acq.latest().read(static_cast<pg::SensorId>(id), m)) {
            hdr = pg::headerOf(m);
        }
        h.latestSampleAgeMs[id] = (hdr != nullptr) ? pg::sampleAgeMs(*hdr, now) : UINT32_MAX;
    }

    // Preprocessing: task counters + summary of the newest processed frame.
    const SensorProcessingTask& proc = SensorProcessingTask::Inst();
    const SensorProcessingTask::Stats ps = proc.stats();
    h.processingBatches = ps.batches;
    h.processingMessages = ps.messages;
    h.processingIdleTimeouts = ps.idleTimeouts;
    h.processingMaxBatch = ps.maxBatch;
    h.processingLastTimeMs = ps.lastProcessingTimeMs;
    h.processingMaxTimeMs = ps.maxProcessingTimeMs;
    if (proc.frames().read(frameCopy_)) {   // locked copy into a member (not the stack)
        const pg::ProcessedSensorFrame& f = frameCopy_;
        h.frameSequence = f.sequence;
        h.frameAgeMs = pg::timebase::ageMs(f.timestampMs, now);
        h.frameQualityFlags = f.qualityFlags;
        h.calibrationState = static_cast<uint8_t>(f.calibrationState);
        h.calibrationFailure = static_cast<uint8_t>(f.lastCalibrationFailure);
        h.calibrationSessionId = f.calibrationSessionId;
        h.accelStatus = static_cast<uint8_t>(f.specificForce_mps2.status);
        h.gyroStatus = static_cast<uint8_t>(f.angularRate_radps.status);
        h.baroStatus = static_cast<uint8_t>(f.baroAltitude_m.status);
        h.accelContributingMask = f.specificForce_mps2.contributingMask;
        h.gyroContributingMask = f.angularRate_radps.contributingMask;
        h.baroContributingMask = f.baroAltitude_m.contributingMask;
        for (uint8_t i = 0; i < 3u; ++i) h.imuGyroCalibration[i] = static_cast<uint8_t>(f.imu[i].gyroCalibration);
        for (uint8_t j = 0; j < 2u; ++j) h.baroReferenceState[j] = static_cast<uint8_t>(f.baro[j].referenceState);
    }
    if (proc.attitude().read(attitudeCopy_)) {   // locked copy into a member
        const pg::AttitudeEstimate& a = attitudeCopy_;
        h.attitudeSequence = a.sequence;
        h.attitudeAgeMs = a.orientationValid ? pg::timebase::ageMs(a.timestampMs, now) : UINT32_MAX;
        h.attitudeQualityFlags = a.qualityFlags;
        h.attitudeState = static_cast<uint8_t>(a.state);
        h.headingReference = static_cast<uint8_t>(a.heading);
        h.accelCorrection = static_cast<uint8_t>(a.accelCorrection);
        h.magCorrection = static_cast<uint8_t>(a.magCorrection);
        h.rollCentiDeg = a.orientationValid ? toScaledInt(a.eulerDeg.roll, 100.0f) : INT32_MIN;
        h.pitchCentiDeg = a.orientationValid ? toScaledInt(a.eulerDeg.pitch, 100.0f) : INT32_MIN;
        h.yawCentiDeg = a.orientationValid ? toScaledInt(a.eulerDeg.yaw, 100.0f) : INT32_MIN;
        h.verticalAccel_mmps2 = a.linearAccel.valid ? toScaledInt(a.linearAccel.vertical_mps2, 1000.0f) : INT32_MIN;
        h.attitudeInitializations = a.initializations;
        h.attitudeInvalidations = a.invalidations;
        h.attitudeRecoveries = a.recoveries;
        h.attitudeGaps = a.gaps;
        h.attitudeAccelRejections = a.accelRejections;
        h.attitudeNumericalFaults = a.numericalFaults;
    }
    if (proc.navigation().read(navigationCopy_)) {   // locked copy into a member
        const pg::NavigationEstimate& n = navigationCopy_;
        h.navSequence = n.sequence;
        h.navState = static_cast<uint8_t>(n.state);
        h.navProcessModel = static_cast<uint8_t>(n.processModel.id);
        h.navBaroOutcome = static_cast<uint8_t>(n.baro.last);
        h.navAccelOutcome = static_cast<uint8_t>(n.accel.last);
        h.navGnssOutcome = static_cast<uint8_t>(n.gnss.last);
        h.navQualityFlags = n.qualityFlags;
        h.navAltitude_cm = n.altitudeValid ? toScaledInt(n.altitude_m, 100.0f) : INT32_MIN;
        h.navVerticalVelocity_mmps = n.velocityValid ? toScaledInt(n.verticalVelocity_mps, 1000.0f) : INT32_MIN;
        h.navVerticalAccel_mmps2 = n.accelerationValid ? toScaledInt(n.verticalAccel_mps2, 1000.0f) : INT32_MIN;
        h.navAltitudeSigma_cm = toScaledInt(n.altitudeSigma_m, 100.0f);          // NaN -> INT32_MIN
        h.navVelocitySigma_mmps = toScaledInt(n.velocitySigma_mps, 1000.0f);
        h.navPredictionAgeMs = n.predictionAgeMs;
        h.navOutageMs = n.outageMs;
        h.navBaroAccepted = n.baro.accepted;
        h.navBaroRejected = n.baro.rejectedGate;
        h.navAccelAccepted = n.accel.accepted;
        h.navAccelRejected = n.accel.rejectedGate + n.accel.rejectedQuality;
        h.navGnssAccepted = n.gnss.accepted;
        h.navRecoveries = n.baro.recoveries + n.accel.recoveries + n.gnss.recoveries;
        h.navInitializations = n.initializations;
        h.navUnavailableEpisodes = n.unavailableEpisodes;
        h.navNumericalFaults = n.numericalFaults;
        h.navLastFaultCode = n.lastFaultCode;
    }
    if (proc.safety().read(safetyCopy_)) {   // locked copy into a member
        const pg::SafetySnapshot& s = safetyCopy_;
        h.safetySequence = s.sequence;
        h.flightPhase = static_cast<uint8_t>(s.phase);
        h.flightPhaseSupport = static_cast<uint8_t>(s.phaseSupport);
        h.clearanceStatus = static_cast<uint8_t>(s.clearanceStatus);
        h.highestWarningSeverity = s.activeWarningMask != 0u ? static_cast<uint8_t>(s.highestSeverity) : 0u;
        h.activeWarningMask = s.activeWarningMask;
        h.clearance_cm = s.clearanceValid ? toScaledInt(s.clearance_m, 100.0f) : INT32_MIN;
        h.closureRate_mmps = s.closureRateValid ? toScaledInt(s.closureRate_mps, 1000.0f) : INT32_MIN;
        h.timeToContact_ms = s.ttcValid ? toScaledInt(s.timeToContact_s, 1000.0f) : INT32_MIN;
        h.safetyQualityFlags = s.qualityFlags;
        h.flightPhaseTransitions = s.phaseTransitions;
        h.takeoffs = s.takeoffs;
        h.landings = s.landings;
        h.warningsRaised = s.warningsRaised;
        h.lowClearanceRaised = s.lowClearanceRaised;
        h.suppressedReactivations = s.suppressedReactivations;
        h.lidarRejectedSamples = s.lidarRejectedSamples;
        h.clearanceStaleUpdates = s.clearanceStaleUpdates;
        h.safetyEventsPublished = s.eventsPublished;
    }
    {
        const pg::SafetyQueueStats q = proc.safetyEvents().stats();
        h.safetyQueueDepth = static_cast<uint32_t>(proc.safetyEvents().size());
        h.safetyQueueHighWater = q.highWater;
        h.safetyQueueDroppedNew = q.droppedNew;
        h.safetyQueueDroppedQueued = q.droppedQueued;
        h.safetyQueueDroppedCritical = q.droppedCritical;
    }
    if (proc.crashReports().read(crashCopy_)) {   // locked copy into a member
        const pg::CrashStatus& c = crashCopy_.status;
        h.crashState = static_cast<uint8_t>(c.state);
        h.crashDecision = static_cast<uint8_t>(c.decision);
        h.crashImpactClass = static_cast<uint8_t>(c.impact.cls);
        h.crashEvidence = c.evidence;
        h.crashFlags = c.flags;
        h.crashPeakAccel_cg = c.impactCandidates != 0u ? toScaledInt(c.impact.peakAccel_mps2 / 9.80665f, 100.0f)
                                                       : INT32_MIN;
        h.impactCandidates = c.impactCandidates;
        h.crashSuspicions = c.suspicions;
        h.crashRejections = c.rejections;
        h.crashSuspicionsCleared = c.suspicionsCleared;
        h.crashSaturationEvents = c.saturationEvents;
        h.crashUncorroboratedPeaks = c.uncorroboratedPeaks;
    }
    const EmergencyAlertTask& emt = EmergencyAlertTask::Inst();
    if (emt.emergency().read(emergencyCopy_)) {
        const pg::EmergencyStatus& e = emergencyCopy_;
        h.emergencyState = static_cast<uint8_t>(e.state);
        h.incidentTrigger = static_cast<uint8_t>(e.trigger);
        h.deliveryStage = static_cast<uint8_t>(e.stage);
        h.cancelNotice = static_cast<uint8_t>(e.cancelNotice);
        h.currentIncidentId = e.incidentId;
        h.countdownRemainingMs = e.countdownRemainingMs;
        h.reportedStages = e.reportedStages;
        h.emergencyFlags = e.flags;
        h.incidents = e.incidents;
        h.manualSosCount = e.manualSosCommands;
        h.cancellationCount = e.cancellations;
        h.cancellationsAfterSend = e.cancellationsAfterSend;
        h.sosRequestCount = e.sosRequestsCreated;
        h.transportAttempts = e.transportAttempts;
        h.transportFailures = e.transportFailures;
        h.deliveriesConfirmed = e.deliveriesConfirmed;
        h.deliveriesFailed = e.deliveriesFailed;
        h.deliveriesUnknown = e.deliveriesUnknown;
        const uint64_t rejected = static_cast<uint64_t>(e.commandsRejected) + e.commandsWrongIncident + e.commandsNotAllowed;
        h.emergencyCommandsRejected = rejected > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(rejected);
    }
    h.emergencyCommandsDropped = emt.commandsDropped();
    if (emt.dispatch().read(dispatchCopy_)) {
        const pg::DispatchStatus& d = dispatchCopy_;
        h.audioAlert = static_cast<uint8_t>(d.audioAlert);
        h.audioState = static_cast<uint8_t>(d.audioState);
        h.ledAlert = static_cast<uint8_t>(d.ledAlert);
        h.ledState = static_cast<uint8_t>(d.ledState);
        h.alertEventsConsumed = d.eventsConsumed;
        h.alertEventGaps = d.eventGaps;
        h.alertMaxLatencyMs = d.maxLatencyMs;
        h.audioFailures = d.audioFailures;
    }
    h.alertBacklog = static_cast<uint32_t>(proc.safetyEvents().size()) + emt.localEventBacklog();
    {
        const EmergencyAlertTask::Stats es = emt.stats();
        h.emergencyWakes = es.wakes;
        h.emergencyMaxWorkMs = es.maxWorkMs;
        h.emergencyMaxWakeIntervalMs = es.maxWakeIntervalMs;
        h.emergencyLateWakes = es.lateWakes;
    }

    // ---- storage (Milestone 8) ----
    StorageTask& sto = StorageTask::Inst();
    if (sto.status().read(storageCopy_)) {
        const pg::StorageStatus& ss = storageCopy_;
        h.journalMount = ss.journalMount;
        h.criticalLogMount = ss.criticalMount;
        h.flightLogMount = ss.flightMount;
        h.journalSequence = ss.journalSequence;
        h.bootCount = ss.bootCount;
        h.emergencyCommits = ss.emergencyCommits;
        h.emergencyCommitFailures = ss.emergencyCommitFailures;
        h.criticalLogRecords = ss.criticalRecords;
        h.telemetryLogRecords = ss.telemetryRecords;
        h.logFailures = ss.logFailures;
        h.logRecordsLost = ss.recordsLost;
        h.storageMaxUnitMs = ss.maxUnitMs;
    }
    h.logQueueDropsCritical = sto.criticalQueue().dropped;
    h.logQueueDropsTelemetry = sto.telemetryQueue().dropped;
    h.emergencyPersist = static_cast<uint8_t>(emergencyCopy_.persist);
    h.emergencyPersistLatencyMaxMs = emergencyCopy_.persistLatencyMaxMs;
    h.nextIncidentId = emergencyCopy_.nextIncidentId;
    // ---- power ----
    if (emt.power().read(powerCopy_)) {
        h.batteryLevel = static_cast<uint8_t>(powerCopy_.level);
        h.chargeState = static_cast<uint8_t>(powerCopy_.charge);
        h.supplyState = static_cast<uint8_t>(powerCopy_.supply);
        h.rail5vState = static_cast<uint8_t>(powerCopy_.rail5v);
        h.amplifierState = static_cast<uint8_t>(powerCopy_.amplifier);
        h.powerFlags = powerCopy_.flags;
        h.batteryMillivolts = powerCopy_.millivolts;
    }
    // ---- task supervision ----
    supervisor_.observe(supAcq_, st.cycles, now);
    supervisor_.observe(supProc_, ps.batches + ps.idleTimeouts, now);
    supervisor_.observe(supEmergency_, emt.stats().wakes, now);
    supervisor_.observe(supStorage_, sto.progress(), now);
    const pg::EmergencyState es = emergencyCopy_.state;
    const pg::SupervisorContext sctx{es != pg::EmergencyState::Idle && es != pg::EmergencyState::Unavailable,
                                     emergencyCopy_.persist == pg::PersistState::Committed};
    const pg::SupervisorVerdict v = supervisor_.evaluate(now, sctx);
    h.supervisorAction = static_cast<uint8_t>(v.action);
    h.supervisorStalledMask = v.stalledMask;
    h.supervisorLateMask = v.lateMask;
    h.supervisorResetDeferred = v.resetDeferred;
    h.supervisorStallEvents = v.stallEvents;

    h.taskCount = watchedCount_;
    for (uint8_t i = 0; i < watchedCount_; ++i) {
        const UBaseType_t freeWords = uxTaskGetStackHighWaterMark(watched_[i].handle);
        h.tasks[i].name = watched_[i].name;
        h.tasks[i].stackHighWaterWords = static_cast<uint32_t>(freeWords);
        h.tasks[i].lowStack = freeWords < TaskConfig::LOW_STACK_WARNING_WORDS;
    }

    const size_t n = acq.sourceCount() < pg::kMaxReportedSources ? acq.sourceCount() : pg::kMaxReportedSources;
    h.sourceCount = static_cast<uint8_t>(n);
    for (size_t i = 0; i < n; ++i) {
        const pg::IAcquisitionSource* src = acq.source(i);
        const pg::SensorHealth& sh = src->health();
        pg::SourceHealthEntry& e = h.sources[i];
        e.name = src->name();
        e.successCount = sh.successCount;
        e.failureCount = sh.failureCount;
        e.initFailures = src->initFailures();
        e.reinitCount = src->reinitCount();
        e.consecutiveFailures = sh.consecutiveFailures;
        e.lastError = static_cast<uint8_t>(sh.lastError);
        e.healthy = sh.isHealthy();
    }

    SOAR_PRINT("Health: up %lu ms, heap %lu B, cycles %lu, missed %lu, drops %lu\n",
               static_cast<unsigned long>(h.uptimeMs), static_cast<unsigned long>(h.freeHeapBytes),
               static_cast<unsigned long>(h.acquisitionCycles), static_cast<unsigned long>(h.missedDeadlines),
               static_cast<unsigned long>(h.sampleDrops));
}
