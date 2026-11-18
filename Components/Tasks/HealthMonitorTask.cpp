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
#include "Tasks/SensorAcquisitionTask.hpp"
#include "Tasks/SensorProcessingTask.hpp"

namespace pg {
// Zero-initialised global; __attribute__((used)) keeps it in the image even if
// nothing else reads it, so the debugger can always find it.
__attribute__((used)) SystemHealthSnapshot g_systemHealth = {};
extern volatile uint32_t g_tickFallbackIncrements;  // Components/SysCore/HalTickFallback.cpp
}  // namespace pg

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
