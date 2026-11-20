/**
 ******************************************************************************
 * @file    EmergencyTypes.hpp
 * @brief   Emergency state, SOS delivery stages and the EmergencyStatus
 *          snapshot.
 *
 * THE STAGES ARE NOT INTERCHANGEABLE. Each one is a different fact, observed
 * by a different party, and a success at one stage is NEVER reported as a
 * success at a later one:
 *   Created              the firmware built an SOS request (nothing left the device)
 *   QueuedForTransport   the transport (future BLE task) accepted it into its buffer
 *   TransferredToPhone   the transport reported the write completed (BLE level only)
 *   AcceptedByPhone      the PHONE APP reported it parsed and accepted the request
 *   PhoneSendAttempted   the phone app reported it started an outbound SMS / call / data send
 *   DeliveryConfirmed    the phone app REPORTED a delivery confirmation (the firmware cannot
 *                        verify it; an SMS "delivered" report is not a human reading it)
 *   DeliveryFailed / DeliveryUnknown   explicit outcomes, never silently turned into success
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_TYPES_HPP
#define PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_TYPES_HPP

#include <cstdint>
#include <type_traits>

namespace pg {

enum class EmergencyState : uint8_t {
    Idle = 0,          ///< armed, no incident
    Countdown,         ///< incident open; SOS request will be created when the countdown ends (cancellable)
    SosPending,        ///< request created; waiting for the transport / the next attempt
    AwaitingPhoneAck,  ///< request handed to the transport; waiting for the phone app to accept it
    AwaitingDelivery,  ///< phone accepted; waiting for its outbound send outcome
    Delivered,         ///< the phone app REPORTED delivery (see file header)
    DeliveryUnknown,   ///< phone accepted, no outcome reported in time (can still change)
    DeliveryFailed,    ///< phone reported failure, or no acceptance after every retry
    Cancelled,         ///< cancelled; see EmergencyStatus::cancelledAfterSend / cancelNotice
    Unavailable,       ///< invalid configuration: the manager does nothing
};

enum class IncidentTrigger : uint8_t {
    None = 0,
    AutomaticCrashSuspected,   ///< CrashDetector suspicion
    Manual,                    ///< pilot / phone / diagnostic command (no crash evidence needed)
    DetectorSilence,           ///< strong in-flight impact, then the detector stopped reporting
    Restored,                  ///< incident restored from persistence after a reset
};

/// Ordered: a larger value is a later stage on the SUCCESS path.
enum class DeliveryStage : uint8_t {
    None = 0,
    Created,
    QueuedForTransport,
    TransferredToPhone,
    AcceptedByPhone,
    PhoneSendAttempted,
    DeliveryConfirmed,
};

namespace stagebit {
/// EmergencyStatus::reportedStages: which stages were ACTUALLY reported (provenance).
/// A later stage reported before an earlier one does NOT set the earlier bit.
enum : uint16_t {
    Created            = 1u << 0,
    QueuedForTransport = 1u << 1,
    TransferredToPhone = 1u << 2,
    AcceptedByPhone    = 1u << 3,
    PhoneSendAttempted = 1u << 4,
    DeliveryConfirmed  = 1u << 5,
    DeliveryFailed     = 1u << 6,
    CancelAccepted     = 1u << 7,
};
}  // namespace stagebit

enum class FailureReason : uint8_t {
    None = 0,
    RetriesExhausted,     ///< no phone acceptance after maxAttempts (per cycle)
    PhoneReportedFailure, ///< the phone app reported its outbound send failed
};

enum class CancelNotice : uint8_t {
    None = 0,      ///< cancelled before anything left the device (or not cancelled)
    Pending,       ///< request had left the device: a cancellation notice is being sent
    Sent,          ///< notice handed to the transport; phone acceptance not reported yet
    Accepted,      ///< the phone app reported it accepted the cancellation (contacts may ALREADY have been alerted)
    Failed,        ///< notice not accepted after the retries
};

enum class CancelReason : uint8_t { None = 0, Command, SuspicionCleared };

namespace emflag {
/// EmergencyStatus::flags
enum : uint32_t {
    IncidentIdVolatile    = 1u << 0,   ///< no persistence: incident ids restart after a reset
    PersistenceUnavailable= 1u << 1,   ///< state is NOT preserved across a reset
    RestoredAfterReset    = 1u << 2,
    TransportUnavailable  = 1u << 3,   ///< request pending, transport down / unconfigured
    ProvenanceIncomplete  = 1u << 4,   ///< a later stage was reported without the earlier ones
    ManuallyConfirmed     = 1u << 5,   ///< a manual SOS confirmed an automatic incident
    CountdownOverrun      = 1u << 6,   ///< the countdown was evaluated late (scheduler gap)
    Unvalidated           = 1u << 7,   ///< countdowns / retry policy are placeholders (always, today)
    // Milestone 8 (persistence / identity)
    PersistencePending    = 1u << 8,   ///< the latest state is queued, not yet committed to flash
    PersistenceDegraded   = 1u << 9,   ///< a commit failed after its retries (state may be lost on reset)
    IdentityUncertain     = 1u << 10,  ///< ids restarted from a blank-but-expected / corrupt journal
    IdentityExhausted     = 1u << 11,  ///< 32-bit incident id space wrapped: re-provision the device
    IdentityUnconfirmed   = 1u << 12,  ///< current request left the device before its id was durable
    RepeatedResetEscalated= 1u << 13,  ///< resets kept restarting the countdown: request created at restore
};
}  // namespace emflag

/// The emergency manager's state (latest-value; the dispatcher and the health task read it).
/// Durability of the newest emergency state (EmergencyStatus::persist).
enum class PersistState : uint8_t {
    Disabled = 0,   ///< no store: nothing survives a reset
    Committed,      ///< the newest state is durably committed
    Pending,        ///< queued; a reset now loses the newest transition
    Failed,         ///< commit attempts exhausted for the newest state (store keeps retrying)
};

struct EmergencyStatus {
    uint32_t sequence;
    uint32_t timestampMs;
    EmergencyState state;
    uint32_t stateSinceMs;
    uint32_t incidentId;            ///< 0 = no incident yet
    IncidentTrigger trigger;
    uint32_t crashEpisodeId;        ///< crash detector episode that opened the incident (0 = none)
    uint32_t countdownTotalMs;
    uint32_t countdownRemainingMs;  ///< 0 outside the countdown
    uint32_t countdownOverrunMs;    ///< how late the expiry was evaluated
    uint16_t requestRevision;       ///< 0 = no request created
    DeliveryStage stage;            ///< furthest success stage reported
    uint16_t reportedStages;        ///< stagebit::*
    FailureReason failure;
    CancelReason cancelReason;
    CancelNotice cancelNotice;
    bool requestIssued;             ///< an SOS request was created for this incident
    bool requestLeftDevice;         ///< handed to the transport at least once
    bool cancelledAfterSend;        ///< cancelled after the request had left the device
    bool transportLinkUp;
    uint8_t attempts;               ///< transport attempts in the current cycle
    uint8_t cycles;                 ///< retry cycles started
    uint32_t nextAttemptInMs;       ///< 0 = due / not scheduled
    uint32_t flags;                 ///< emflag::*
    uint8_t additionalImpacts;      ///< further suspicions attached to this incident
    // lifetime counters (saturating)
    uint32_t incidents;
    uint32_t automaticIncidents;
    uint32_t manualSosCommands;
    uint32_t cancellations;
    uint32_t cancellationsAfterSend;
    uint32_t sosRequestsCreated;
    uint32_t transportAttempts;
    uint32_t transportFailures;     ///< timeouts, write failures, phone rejections, submit failures
    uint32_t duplicateAcks;
    uint32_t wrongIncidentReports;
    uint32_t invalidReports;        ///< unknown kind / revision from the future / no request
    uint32_t commandsRejected;      ///< malformed
    uint32_t commandsWrongIncident;
    uint32_t commandsDuplicate;
    uint32_t commandsNotAllowed;
    uint32_t deliveriesConfirmed;
    uint32_t deliveriesFailed;
    uint32_t deliveriesUnknown;
    uint32_t detectorSilenceIncidents;
    uint32_t persistenceWrites;     ///< records COMMITTED (not merely queued)
    uint32_t persistenceFailures;   ///< rejected saves + failed commits
    // Milestone 8
    PersistState persist;
    uint32_t persistLatencyMaxMs;   ///< queue -> commit, worst seen
    uint32_t nextIncidentId;        ///< id the next incident will use
    uint16_t batteryMillivolts;     ///< last valid battery reading given to the manager (0 = unknown)
    uint8_t countdownRestarts;      ///< resets during the current countdown
};
static_assert(std::is_trivially_copyable<EmergencyStatus>::value, "status must be memcpy-safe");

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_TYPES_HPP
