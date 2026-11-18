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
