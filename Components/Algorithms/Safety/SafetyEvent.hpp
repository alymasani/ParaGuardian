/**
 ******************************************************************************
 * @file    SafetyEvent.hpp
 * @brief   Common, fixed-size safety event for every safety producer (flight
 *          state, landing safety, crash detector, emergency / SOS manager) and
 *          every consumer (alert dispatcher today; logging / BLE later).
 *
 * EVENTS ARE TRANSITIONS, NOT SAMPLES. A warning produces:
 *   Raised      (new episode: episodeId is new)
 *   Escalated   (same episode, higher severity)
 *   Deescalated (same episode, lower but non-zero severity)
 *   Cleared     (same episode, condition gone)
 * and one-shot Notices (e.g. a flight-phase change). An ongoing warning
 * produces NO further events; its current state is in the SafetySnapshot.
 * Consumers that need "still active" (e.g. a repeating tone) read the
 * snapshot, so the queue is never flooded every 10 ms.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_HPP

#include <cstdint>
#include <type_traits>

#include "Algorithms/Flight/FlightStateTypes.hpp"

namespace pg {

enum class SafetyEventType : uint8_t {
    None = 0,
    FlightPhaseChanged,          ///< Notice: values carry nothing; phase = new phase
    ExcessiveDescent,            ///< navigation vertical speed beyond the descent thresholds
    LowGroundClearance,          ///< LiDAR clearance low (advisory: informational near terrain)
    RapidGroundClosure,          ///< LiDAR closure rate / time-to-contact (LiDAR only)
    HazardousApproach,           ///< low clearance AND fast descent / closure
    TerrainClearanceUnavailable, ///< airborne near terrain (or descending fast) without usable LiDAR
    NavigationDegraded,          ///< UKF degraded / unavailable while airborne
    // ---- crash detector (source CrashDetector; episodeId = crash episode) ----
    CrashSuspected = 32,         ///< Raised (Critical): enough evidence for escalation (NOT a confirmed crash);
                                 ///< Cleared: flight resumed after the impact
    // ---- emergency manager (source Emergency; episodeId = INCIDENT id) ----
    EmergencyRequest = 33,       ///< Notice (Critical): an SOS request was CREATED (not sent, not delivered)
    ImpactCandidate = 34,        ///< Raised (Advisory): possible impact under evaluation; Cleared: rejected
    EmergencyCountdown = 35,     ///< Raised (Critical): SOS countdown running; Cleared: it ended (expired or cancelled)
    EmergencyCancelled = 36,     ///< Notice: the pilot / a command cancelled the incident (see EmergencyStatus for when)
    SosTransportUnavailable = 37,///< Raised (Warning): request pending, no transport; Cleared: transport back
    SosPhoneAccepted = 38,       ///< Notice: the PHONE APP reported it accepted the request (not delivered!)
    SosDeliveryConfirmed = 39,   ///< Notice: the phone app REPORTED delivery to a contact (firmware cannot verify)
    SosDeliveryFailed = 40,      ///< Notice (Critical): delivery failed / retries exhausted
    SosDeliveryUnknown = 41,     ///< Notice (Warning): accepted by the phone, outcome not reported in time
};

/// Ordered: a larger value is more urgent.
enum class SafetySeverity : uint8_t {
    Info = 0,       ///< notices only
    Advisory = 1,   ///< awareness, no action implied
    Caution = 2,
    Warning = 3,
    Critical = 4,
};

enum class SafetyTransition : uint8_t {
    Notice = 0,     ///< one-shot information
    Raised,
    Escalated,
    Deescalated,
    Cleared,
};

enum class SafetySource : uint8_t {
    FlightState = 0,
    LandingSafety,
    GroundClearance,
    Navigation,
    CrashDetector = 16,   ///< CrashDetector (SensorProcessingTask)
    Emergency = 17,       ///< EmergencyManager (EmergencyAlertTask; its own event-id sequence)
};

namespace safetyq {
/// SafetyEvent::qualityFlags / SafetySnapshot::qualityFlags
enum : uint32_t {
    DegradedEvidence     = 1u << 0,   ///< decided on partial evidence (e.g. descent from navigation, LiDAR rate missing)
    NavigationInvalid    = 1u << 1,
    NavigationDegraded   = 1u << 2,
    ModelPlaceholder     = 1u << 3,   ///< navigation uses the placeholder process model (always, today)
    LidarUnavailable     = 1u << 4,
    LidarMountingUnverified = 1u << 5,
    FlatTerrainAssumed   = 1u << 6,
    ClearanceStepChange  = 1u << 7,
    PhaseUncertain       = 1u << 8,   ///< flight phase Unknown or a candidate state
    CooldownSuppressed   = 1u << 9,   ///< re-activations were suppressed during this episode's cooldown
    ImuMountingUnverified = 1u << 10,
    Unvalidated          = 1u << 11,  ///< thresholds are provisional development values (always, today)
};
}  // namespace safetyq

struct SafetyEvent {
    uint32_t eventId;             ///< increments per published event of ONE emitter (gaps = events lost
                                  ///< downstream). Two emitters exist: SensorProcessingTask (flight, landing,
                                  ///< crash) and EmergencyAlertTask (source Emergency); track gaps per emitter.
    uint32_t episodeId;           ///< one per warning activation; 0 for notices. Emergency events: the incident id
    uint32_t timestampMs;         ///< MCU time of the decision
    SafetyEventType type;
    SafetySeverity severity;      ///< after the transition (Cleared: severity before clearing)
    SafetySeverity previousSeverity;
    SafetyTransition transition;
    SafetySource source;
    FlightPhase phase;            ///< phase at the decision
    uint16_t suppressedCount;     ///< re-activations suppressed (cooldown) during this episode
    uint32_t qualityFlags;        ///< safetyq::*
    // Values at the decision (NaN when not available / not relevant)
    float verticalVelocity_mps;   ///< navigation, positive up
    float clearance_m;            ///< LiDAR projected clearance
    float closureRate_mps;        ///< LiDAR, positive = approaching
    float timeToContact_s;
};

static_assert(std::is_trivially_copyable<SafetyEvent>::value, "event must be memcpy-safe");
static_assert(sizeof(SafetyEvent) <= 48u, "keep the event compact");

/// Anything that accepts events (queue, logger, test recorder). Never blocks.
class ISafetyEventSink {
public:
    /// Returns false if the event (or another one, to make room) was dropped.
    virtual bool publish(const SafetyEvent& e) = 0;

protected:
    ~ISafetyEventSink() = default;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_HPP
