/**
 ******************************************************************************
 * @file    SystemHealth.hpp
 * @brief   Snapshot of system health, refreshed by HealthMonitorTask.
 *
 * With no debug UART on the board yet, the way to see this data is the
 * STM32CubeIDE debugger: add  g_systemHealth  to "Live Expressions" while the
 * target runs over SWD. Every field is a plain integer so it reads cleanly.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_SYSTEM_HEALTH_HPP
#define PARAGUARDIAN_TASKS_SYSTEM_HEALTH_HPP

#include <cstdint>

#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

constexpr uint8_t kMaxWatchedTasks = 6u;
constexpr uint8_t kMaxReportedSources = 10u;
constexpr uint8_t kMaxReportedQueues = 4u;

struct TaskHealthEntry {
    const char* name;
    uint32_t stackHighWaterWords;  ///< Minimum free stack ever seen (uxTaskGetStackHighWaterMark).
    bool lowStack;                 ///< stackHighWaterWords < LOW_STACK_WARNING_WORDS
};

struct SourceHealthEntry {
    const char* name;
    uint32_t successCount;
    uint32_t failureCount;
    uint32_t initFailures;
    uint32_t reinitCount;          ///< re-initialised after repeated read failures
    uint16_t consecutiveFailures;
    uint8_t lastError;             ///< pg::Status as a number
    bool healthy;
};

struct SystemHealthSnapshot {
    uint32_t uptimeMs;
    uint32_t reportCount;
    // FreeRTOS heap (heap_4, configTOTAL_HEAP_SIZE)
    uint32_t freeHeapBytes;
    uint32_t minimumEverFreeHeapBytes;
    // Acquisition loop
    uint32_t acquisitionCycles;
    uint32_t missedDeadlines;      ///< xTaskDelayUntil() found the next wake time already in the past
    uint32_t cycleOverruns;        ///< one cycle took longer than the period
    uint32_t maxCycleTimeMs;
    uint32_t lastCycleTimeMs;
    uint32_t initAttemptsDeferred; ///< init attempts postponed to a later cycle by the budget
    uint32_t samplesPublished;
    uint32_t sampleDrops;          ///< samples a subscriber queue had no room for (not delivered)
    uint32_t sampleOverwrites;     ///< older queued samples discarded by DropOldest queues
    /// Age of the latest sample per SensorId (ms) at collection time; UINT32_MAX = never received.
    uint32_t latestSampleAgeMs[static_cast<uint8_t>(SensorId::Count)];
    uint8_t queueCount;
    uint32_t queueUsed[kMaxReportedQueues];
    uint32_t queueCapacity[kMaxReportedQueues];
    // Sensor preprocessing (SensorProcessingTask + newest ProcessedSensorFrame)
    uint32_t processingBatches;
    uint32_t processingMessages;
    uint32_t processingIdleTimeouts;   ///< wakes with no sensor data at all
    uint32_t processingMaxBatch;
    uint32_t processingLastTimeMs;
    uint32_t processingMaxTimeMs;
    uint32_t frameSequence;            ///< 0 = no frame yet
    uint32_t frameAgeMs;               ///< age of the newest frame at collection time
    uint32_t frameQualityFlags;        ///< pg::quality::* bits of the newest frame
    uint8_t calibrationState;          ///< pg::CalibrationState
    uint8_t calibrationFailure;        ///< pg::CalibrationFailure of the last failed session
    uint32_t calibrationSessionId;
    uint8_t accelStatus;               ///< pg::RedundancyStatus of specific force
    uint8_t gyroStatus;                ///< pg::RedundancyStatus of angular rate
    uint8_t baroStatus;                ///< pg::RedundancyStatus of barometric altitude
    uint8_t accelContributingMask;     ///< bit i = IMU i used
    uint8_t gyroContributingMask;
    uint8_t baroContributingMask;      ///< bit j = barometer j used
    uint8_t imuGyroCalibration[3];     ///< pg::CalibrationState per IMU
    uint8_t baroReferenceState[2];     ///< pg::CalibrationState per barometer
    // Attitude (newest pg::AttitudeEstimate). Angles are DISPLAY values (Z-Y-X vs NED).
    uint32_t attitudeSequence;         ///< 0 = no estimate yet
    uint32_t attitudeAgeMs;            ///< age of the orientation (last integrated gyro sample)
    uint32_t attitudeQualityFlags;     ///< pg::attitude::* bits
    uint8_t attitudeState;             ///< pg::AttitudeState
    uint8_t headingReference;          ///< pg::HeadingReference
    uint8_t accelCorrection;           ///< pg::CorrectionStatus of the last update
    uint8_t magCorrection;             ///< pg::CorrectionStatus of the last update
    int32_t rollCentiDeg;              ///< INT32_MIN = orientation not valid
    int32_t pitchCentiDeg;
    int32_t yawCentiDeg;               ///< relative to an arbitrary start unless headingReference = MagneticReferenced
    int32_t verticalAccel_mmps2;       ///< gravity-compensated, positive up; INT32_MIN = not valid
    uint32_t attitudeInitializations;
    uint32_t attitudeInvalidations;
    uint32_t attitudeRecoveries;
    uint32_t attitudeGaps;
    uint32_t attitudeAccelRejections;
    uint32_t attitudeNumericalFaults;
    // Vertical navigation (newest pg::NavigationEstimate). Relative altitude, positive up.
    uint32_t navSequence;              ///< 0 = no estimate yet
    uint8_t navState;                  ///< pg::NavState
    uint8_t navProcessModel;           ///< pg::ProcessModelId
    uint8_t navBaroOutcome;            ///< pg::MeasurementOutcome of the last update
    uint8_t navAccelOutcome;
    uint8_t navGnssOutcome;
    uint32_t navQualityFlags;          ///< pg::navflag::* bits
    int32_t navAltitude_cm;            ///< INT32_MIN = not valid
    int32_t navVerticalVelocity_mmps;  ///< INT32_MIN = not valid
    int32_t navVerticalAccel_mmps2;    ///< INT32_MIN = not valid
    int32_t navAltitudeSigma_cm;       ///< INT32_MIN = not initialised
    int32_t navVelocitySigma_mmps;
    uint32_t navPredictionAgeMs;
    uint32_t navOutageMs;              ///< since the last accepted barometer sample; UINT32_MAX = never
    uint32_t navBaroAccepted;
    uint32_t navBaroRejected;          ///< gate rejections
    uint32_t navAccelAccepted;
    uint32_t navAccelRejected;         ///< gate + quality rejections
    uint32_t navGnssAccepted;
    uint32_t navRecoveries;            ///< covariance-inflation recoveries (all sources)
    uint32_t navInitializations;
    uint32_t navUnavailableEpisodes;
    uint32_t navNumericalFaults;
    uint32_t navLastFaultCode;         ///< pg::nav::UkfStatus
    // Flight / landing safety (newest pg::SafetySnapshot + event queue). Provisional thresholds.
    uint32_t safetySequence;           ///< 0 = no snapshot yet
    uint8_t flightPhase;               ///< pg::FlightPhase
    uint8_t flightPhaseSupport;        ///< pg::EvidenceLevel (heuristic, not a probability)
    uint8_t clearanceStatus;           ///< pg::ClearanceStatus
    uint8_t highestWarningSeverity;    ///< pg::SafetySeverity of active warnings (0 = none)
    uint8_t activeWarningMask;         ///< bit i = pg::LandingWarning i active
    int32_t clearance_cm;              ///< LiDAR projected clearance; INT32_MIN = not valid
    int32_t closureRate_mmps;          ///< positive = approaching terrain; INT32_MIN = not valid
    int32_t timeToContact_ms;          ///< INT32_MIN = no finite TTC
    uint32_t safetyQualityFlags;       ///< pg::safetyq::*
    uint32_t flightPhaseTransitions;
    uint32_t takeoffs;
    uint32_t landings;
    uint32_t warningsRaised;           ///< all landing warnings (Raised events)
    uint32_t lowClearanceRaised;
    uint32_t suppressedReactivations;  ///< cooldown: re-activations not re-announced
    uint32_t lidarRejectedSamples;     ///< beam geometry / attitude
    uint32_t clearanceStaleUpdates;
    uint32_t safetyEventsPublished;
    uint32_t safetyQueueDepth;
    uint32_t safetyQueueHighWater;
    uint32_t safetyQueueDroppedNew;    ///< overflow: new event dropped
    uint32_t safetyQueueDroppedQueued; ///< overflow: older lower-severity event dropped
    uint32_t safetyQueueDroppedCritical;
    // Crash detection (newest pg::CrashStatus). Provisional thresholds; "suspected" is NOT "confirmed".
    uint8_t crashState;                ///< pg::CrashState
    uint8_t crashDecision;             ///< pg::CrashDecision of the last finished candidate
    uint8_t crashImpactClass;          ///< pg::ImpactClass of the last candidate
    uint32_t crashEvidence;            ///< pg::crashev::* of the current / last episode
    uint32_t crashFlags;               ///< pg::crashflag::*
    int32_t crashPeakAccel_cg;         ///< last corroborated impact peak, 0.01 g; INT32_MIN = none
    uint32_t impactCandidates;
    uint32_t crashSuspicions;          ///< suspected incidents (detector)
    uint32_t crashRejections;
    uint32_t crashSuspicionsCleared;   ///< flight resumed after a suspicion
    uint32_t crashSaturationEvents;    ///< IMU full scale reached during an impact
    uint32_t crashUncorroboratedPeaks; ///< single-IMU spikes ignored
    // Emergency manager (newest pg::EmergencyStatus). Transport / audio / LED are UNCONFIGURED in firmware.
    uint8_t emergencyState;            ///< pg::EmergencyState
    uint8_t incidentTrigger;           ///< pg::IncidentTrigger
    uint8_t deliveryStage;             ///< pg::DeliveryStage (furthest success stage REPORTED)
    uint8_t cancelNotice;              ///< pg::CancelNotice
    uint32_t currentIncidentId;        ///< 0 = none yet
    uint32_t countdownRemainingMs;
    uint32_t reportedStages;           ///< pg::stagebit::* (acknowledgment provenance)
    uint32_t emergencyFlags;           ///< pg::emflag::*
    uint32_t incidents;
    uint32_t manualSosCount;
    uint32_t cancellationCount;
    uint32_t cancellationsAfterSend;
    uint32_t sosRequestCount;
    uint32_t transportAttempts;
    uint32_t transportFailures;
    uint32_t deliveriesConfirmed;      ///< as REPORTED by the phone app
    uint32_t deliveriesFailed;
    uint32_t deliveriesUnknown;
    uint32_t emergencyCommandsRejected;   ///< malformed + wrong incident + not allowed
    uint32_t emergencyCommandsDropped;    ///< command inbox full
    // Alert dispatcher (newest pg::DispatchStatus) + emergency task timing
    uint8_t audioAlert;                ///< pg::AlertKind driving the audio (0 = silent)
    uint8_t audioState;                ///< pg::OutputState (2 = Unavailable: no audio hardware configured)
    uint8_t ledAlert;                  ///< pg::AlertKind driving the LED
    uint8_t ledState;                  ///< pg::OutputState
    uint32_t alertBacklog;             ///< events waiting (processing queue + emergency queue)
    uint32_t alertEventsConsumed;
    uint32_t alertEventGaps;           ///< event ids lost upstream
    uint32_t alertMaxLatencyMs;        ///< event decision -> dispatch
    uint32_t audioFailures;            ///< audio requests refused by the backend
    uint32_t emergencyWakes;
    uint32_t emergencyMaxWorkMs;
    uint32_t emergencyMaxWakeIntervalMs;
    uint32_t emergencyLateWakes;
    // Start-up problems (task creation / registration)
    uint32_t startupErrors;
    /// RCC->RSR captured at boot (bit 24 SFTRSTF software reset, 26 IWDG1RSTF,
    /// 21 BORRSTF brown-out, 22 PINRSTF, 23 PORRSTF power-on). Survives reset in hardware.
    uint32_t resetFlags;
    /// HAL ticks produced by HalTickFallback while TIM6 was masked (pre-scheduler).
    uint32_t tickFallbackIncrements;
    // Kernel hooks (forwarded from CubeMX-generated hook functions in Src/freertos.c)
    uint32_t stackOverflowCount;
    uint32_t mallocFailedCount;
    char lastStackOverflowTask[16];  ///< configMAX_TASK_NAME_LEN = 16
    // Per task / per sensor
    uint8_t taskCount;
    TaskHealthEntry tasks[kMaxWatchedTasks];
    uint8_t sourceCount;
    SourceHealthEntry sources[kMaxReportedSources];
};

/// Defined in HealthMonitorTask.cpp. Written only by HealthMonitorTask
/// (and the counters by the hook forwarders); read by the debugger.
extern SystemHealthSnapshot g_systemHealth;

}  // namespace pg

#endif  // PARAGUARDIAN_TASKS_SYSTEM_HEALTH_HPP
