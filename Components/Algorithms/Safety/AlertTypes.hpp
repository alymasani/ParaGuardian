/**
 ******************************************************************************
 * @file    AlertTypes.hpp
 * @brief   Pilot alerts (what to tell the pilot), audio / LED pattern ids and
 *          the dispatcher's status.
 *
 * An ALERT is derived from safety state (landing warnings, crash suspicion,
 * emergency state); several can be active at once and exactly one drives
 * the audio output and one the LED (highest priority, AlertPriorityPolicy).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_ALERT_TYPES_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_ALERT_TYPES_HPP

#include <cstdint>
#include <type_traits>

#include "Algorithms/Safety/SafetyEvent.hpp"
#include "Output/LED/IStatusLed.hpp"

namespace pg {

enum class AlertKind : uint8_t {
    None = 0,
    SystemDegraded,      ///< navigation degraded / terrain clearance unavailable / safety data stale
    LowClearance,        ///< LiDAR advisory
    DescentWarning,      ///< ExcessiveDescent
    ClosureWarning,      ///< RapidGroundClosure (time to contact)
    HazardousApproach,
    SosAccepted,         ///< the phone app accepted / reported delivery (information)
    SosActive,           ///< request created / being sent / awaiting the phone
    SosCommFailure,      ///< transport unavailable, delivery failed or unknown
    CrashSuspected,
    SosCountdown,
    // Milestone 8 (appended: existing values unchanged)
    LowBattery,          ///< Caution: battery Low; Warning: battery Critical (PowerManager)
    SystemFault,         ///< persistence degraded / storage fault / supervisor fault (advisory)
    PhoneDisconnected,   ///< in flight, the phone link was lost (no SOS pending: SosCommFailure covers that)
    Count
};
constexpr uint8_t kAlertKindCount = static_cast<uint8_t>(AlertKind::Count);

/// Audio pattern ids (timing in AudioAlertScheduler.cpp, one table).
enum class AudioPatternId : uint8_t {
    None = 0,
    AdvisoryChime,     ///< 1 short beep, once
    CautionDouble,     ///< 2 beeps, repeated every 10 s
    WarningTriple,     ///< 3 beeps, repeated every 3 s
    CriticalFast,      ///< continuous fast beeps
    CrashAlarm,        ///< long tone + pause, repeated
    CountdownTick,     ///< 1 beep per second
    CountdownFinal,    ///< 2 beeps per second (last seconds)
    SosConfirm,        ///< long tone every 10 s while the request is pending
    CommFailure,       ///< low triple every 5 s
    InfoChime,         ///< 2 soft beeps, once
    Count
};

struct AlertStyle {
    uint8_t priority;          ///< 0 = not alerting
    AudioPatternId audio;
    LedPattern led;
    LedColor color;
    bool acknowledgeable;      ///< the pilot may silence the audio (never for SOS countdown / crash)
};

enum class OutputState : uint8_t {
    Idle = 0,        ///< nothing to output
    Active,          ///< pattern running
    Unavailable,     ///< no backend / backend refuses (no hardware configured)
};

/// Non-safety-snapshot inputs of the dispatcher (latest values, set by the emergency task).
struct SystemAlertInputs {
    uint8_t batteryLevel;     ///< pg::BatteryLevel (0 Unknown, 1 Healthy, 2 Low, 3 Critical)
    bool phoneLinkKnown;      ///< a phone transport exists (configured)
    bool phoneLinkUp;
    bool inFlight;
    bool storageDegraded;     ///< emergency state not durable / storage faults
    bool supervisorFault;     ///< a task missed its heartbeat (TaskSupervisor)
};

struct DispatchStatus {
    uint32_t sequence;
    uint32_t timestampMs;
    AlertKind audioAlert;           ///< alert driving the audio (None = silent)
    AlertKind ledAlert;             ///< alert driving the LED (None = normal status pattern)
    uint8_t topPriority;
    uint16_t activeMask;            ///< bit k = AlertKind k active
    SafetySeverity severity[kAlertKindCount];
    AudioPatternId audioPattern;
    LedPattern ledPattern;
    LedColor ledColor;
    OutputState audioState;
    OutputState ledState;
    bool safetyStale;               ///< the safety snapshot is old (processing silent)
    bool crashStale;
    // counters (saturating)
    uint32_t eventsConsumed;
    uint32_t eventGaps;             ///< event ids skipped (events lost upstream), per emitter
    uint32_t duplicateEvents;       ///< replayed / out-of-order event ids ignored
    uint32_t lastLatencyMs;         ///< decision -> dispatch of the last event
    uint32_t maxLatencyMs;
    uint32_t preemptions;           ///< a higher-priority alert took over the audio
    uint32_t acknowledgements;
    uint32_t audioRequests;
    uint32_t audioFailures;
    uint32_t ledChanges;
    uint32_t transientAlerts;       ///< raised and cleared before a snapshot showed them (still announced)
};
static_assert(std::is_trivially_copyable<DispatchStatus>::value, "status must be memcpy-safe");

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_ALERT_TYPES_HPP
