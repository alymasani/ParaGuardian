/**
 ******************************************************************************
 * @file    FlightRecorder.hpp
 * @brief   Decides WHAT is logged and how often (logging profiles), encodes
 *          it into compact versioned records and pushes them into the
 *          bounded log queues. Portable; never blocks; no flash access.
 *
 * ROUTING: emergency transitions, SOS stages, Warning+ safety events, boot
 * and storage faults go to the CRITICAL queue (separate partition, flushed
 * immediately); periodic telemetry goes to the TELEMETRY queue. A telemetry
 * flood can fill only its own queue (drops counted there).
 *
 * PROFILES (periods in ms; 0 = never):
 *               navigation  sensor quality  crash summary  health
 *   Minimal          0          0             on change      60000
 *   Normal        1000       5000             on change      10000
 *   Detailed       100       1000             on change       5000
 *   Emergency      200       1000             on change       5000   (automatic while an incident is open)
 * Raw sensor samples are NOT logged: at 416 Hz x 3 IMUs the bus and flash
 * budget would be dominated by them (see MILESTONE8_SOFTWARE_RELIABILITY_DESIGN.md).
 *
 * Thread use: the processing task calls the navigation / quality / crash
 * methods, the emergency task the event / emergency / power methods. The two
 * sets touch disjoint state; the queues are locked.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_STORAGE_FLIGHT_RECORDER_HPP
#define PARAGUARDIAN_STORAGE_FLIGHT_RECORDER_HPP

#include <cstdint>

#include "Algorithms/Crash/CrashTypes.hpp"
#include "Algorithms/Emergency/EmergencyTypes.hpp"
#include "Algorithms/Navigation/NavigationTypes.hpp"
#include "Algorithms/Safety/SafetyEvent.hpp"
#include "Storage/LogRecords.hpp"

namespace pg {

enum class LogProfile : uint8_t { Minimal = 0, Normal, Detailed, Emergency };

struct LogProfilePeriods {
    uint32_t navigationMs;
    uint32_t qualityMs;
    uint32_t healthMs;
};
LogProfilePeriods logProfilePeriods(LogProfile p);

struct SensorQualitySummary {
    uint32_t qualityFlags;
    uint8_t accelStatus, gyroStatus, baroStatus;
    uint8_t accelMask, gyroMask, baroMask;
    uint16_t sourcesHealthyMask;
};

struct HealthSummary {
    uint32_t freeHeap, minHeap, processingMaxMs, emergencyLateWakes, stackMinWords, faults;
};

struct PowerLogSummary {
    uint8_t batteryState;
    uint16_t millivolts;
    uint8_t flags;
    uint8_t rail;
    uint8_t amplifier;
    uint8_t lidarPower;
    uint8_t sources;
};

class FlightRecorder {
public:
    FlightRecorder(ILogSink& critical, ILogSink& telemetry, LogProfile profile = LogProfile::Normal);

    void setProfile(LogProfile p, uint32_t nowMs);
    LogProfile profile() const { return emergencyActive_ ? LogProfile::Emergency : profile_; }

    // ---- emergency task ----
    void logBoot(uint32_t resetFlags, uint32_t bootCount, uint8_t journalMount, uint8_t criticalMount,
                 uint8_t flightMount, uint32_t nowMs);
    void logEvent(const SafetyEvent& e, uint32_t nowMs);
    /// Logs only when the incident state / stage / revision / notice / persistence changed.
    void logEmergency(const EmergencyStatus& s, uint32_t nowMs);
    void logPower(const PowerLogSummary& p, uint32_t nowMs);   ///< on change only

    // ---- processing task ----
    void logNavigation(const NavigationEstimate& n, uint32_t nowMs);     ///< rate-limited by profile
    void logQuality(const SensorQualitySummary& q, uint32_t nowMs);      ///< rate-limited by profile
    void logCrash(const CrashStatus& c, uint32_t nowMs);                 ///< on state / candidate change
    void logHealth(const HealthSummary& h, uint32_t nowMs);              ///< rate-limited by profile

    uint32_t criticalRefused() const { return criticalRefused_; }
    uint32_t telemetryRefused() const { return telemetryRefused_; }

private:
    bool due(uint32_t& last, bool& started, uint32_t period, uint32_t nowMs);
    void push(bool critical, LogType t, uint32_t nowMs, const LogPayload& p);

    ILogSink& critical_;
    ILogSink& telemetry_;
    LogProfile profile_;
    volatile bool emergencyActive_ = false;   ///< written by the emergency task, read by processing (one byte)
    // emergency-task state
    bool haveEmergency_ = false;
    uint8_t lastState_ = 0u, lastStage_ = 0u, lastNotice_ = 0u, lastPersist_ = 0u;
    uint16_t lastRevision_ = 0u;
    uint32_t lastIncident_ = 0u;
    bool havePower_ = false;
    PowerLogSummary lastPower_{};
    // processing-task state
    uint32_t navLast_ = 0u, qualityLast_ = 0u, healthLast_ = 0u;
    bool navStarted_ = false, qualityStarted_ = false, healthStarted_ = false;
    bool haveCrash_ = false;
    uint8_t crashState_ = 0u;
    uint32_t crashCandidates_ = 0u;
    uint32_t criticalRefused_ = 0u;
    uint32_t telemetryRefused_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_STORAGE_FLIGHT_RECORDER_HPP
