/**
 ******************************************************************************
 * @file    EmergencyManager.hpp
 * @brief   Incident lifecycle: countdown, cancellation, manual SOS, SOS
 *          request creation, transport attempts / retries / acknowledgments,
 *          delivery outcome and re-arming. Portable, no heap, never blocks.
 *
 * INDEPENDENT OF THE SENSORS: tick(now) advances every timer from the clock
 * alone, so a countdown expires and retries continue even if no sensor frame
 * ever arrives again (the emergency task calls tick() on a period).
 *
 * INPUTS (all non-blocking, all in the emergency task):
 *   observeCrash(CrashReport)    latest crash detector status + incident context
 *   handleCommand(command)       manual SOS / cancel / reset / ack / status
 *   handleReport(report)         transport + phone acknowledgments
 *   tick(now)                    time-driven transitions + transport attempts
 *
 * SUSPICION vs ESCALATION: a CrashSuspected (detector) opens an INCIDENT and a
 * cancellable countdown. Only when the countdown ends is an SOS REQUEST
 * created; only a transport report says it left; only the phone app says it
 * was accepted / sent / delivered. See EmergencyTypes.hpp.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_MANAGER_HPP
#define PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_MANAGER_HPP

#include <cstdint>

#include "Algorithms/Crash/CrashTypes.hpp"
#include "Algorithms/Emergency/EmergencyCommand.hpp"
#include "Algorithms/Emergency/EmergencyConfig.hpp"
#include "Algorithms/Emergency/EmergencyTypes.hpp"
#include "Algorithms/Emergency/IEmergencyStore.hpp"
#include "Algorithms/Emergency/ISosTransport.hpp"
#include "Algorithms/Emergency/SosRequestMailbox.hpp"
#include "Algorithms/Safety/SafetyEventEmitter.hpp"

namespace pg {

enum class CommandResult : uint8_t {
    Accepted = 0,
    ForwardToAlerts,   ///< AcknowledgeAlert: handled by the SafetyEventDispatcher
    NoEffect,          ///< valid, but nothing to do (e.g. repeated manual SOS while sending)
    Duplicate,         ///< same (source, sequence) as the previous command of that source
    Malformed,
    WrongIncident,     ///< refers to another / a closed incident
    NotAllowed,        ///< not allowed in the current state (e.g. reset while sending)
};

class EmergencyManager {
public:
    /// @param store nullptr or a store whose available() is false = no persistence.
    EmergencyManager(const EmergencyConfig& cfg, ISosTransport& transport, IEmergencyStore* store = nullptr);

    bool configValid() const { return valid_; }
    bool addSink(ISafetyEventSink* sink) { return emitter_.addSink(sink); }

    /// Loads the persisted incident (if any). Call once, before the first tick().
    void restore(uint32_t nowMs);

    void observeCrash(const CrashReport& report, uint32_t nowMs);
    CommandResult handleCommand(const EmergencyCommand& cmd, uint32_t nowMs);
    void handleReport(const TransportReport& report, uint32_t nowMs);
    const EmergencyStatus& tick(uint32_t nowMs);

    const EmergencyStatus& status() const { return st_; }
    PendingSosRecord pendingRecord() const;
    const SafetyEventEmitter& emitter() const { return emitter_; }

private:
    void openIncident(IncidentTrigger trigger, uint32_t crashEpisode, uint32_t nowMs);
    void expireCountdown(uint32_t nowMs, bool lateIsOverrun);
    void createRequest(uint32_t nowMs);
    void buildRequest(SosRequestKind kind, uint32_t nowMs);
    void cancel(CancelReason reason, uint32_t nowMs);
    void resumeSending(uint32_t nowMs);
    void serviceTransport(uint32_t nowMs);
    void attemptFailed(uint32_t nowMs);
    void enter(EmergencyState s, uint32_t nowMs);
    void markStage(DeliveryStage stage, uint16_t bit);
    void setTransportUnavailable(bool unavailable, uint32_t nowMs);
    void emit(SafetyEventType type, SafetyTransition tr, SafetySeverity sev, SafetySeverity prev, uint32_t nowMs);
    void persist();
    uint32_t backoffMs(uint8_t failedAttempts) const;
    bool sendingActive() const;
    bool awaitingAck() const;

    EmergencyConfig cfg_;
    bool valid_;
    ISosTransport& transport_;
    IEmergencyStore* store_;
    bool persistent_ = false;
    SafetyEventEmitter emitter_;
    EmergencyStatus st_{};
    SosRequest req_{};
    IncidentContext ctx_{};
    ImpactSummary impact_{};
    uint32_t crashEvidence_ = 0u;

    uint32_t nextIncidentId_ = 1u;
    uint32_t countdownDeadlineMs_ = 0u;
    uint32_t eventMs_ = 0u;            ///< incident time (impact / command)
    uint32_t nextAttemptMs_ = 0u;
    uint32_t ackDeadlineMs_ = 0u;
    uint32_t acceptedMs_ = 0u;
    bool transportUnavailableRaised_ = false;

    // command de-duplication (per source)
    bool seqSeen_[kCommandSourceCount] = {};
    uint16_t lastSeq_[kCommandSourceCount] = {};

    // crash detector supervision
    bool haveCrash_ = false;
    uint32_t lastSerial_ = 0u;
    uint32_t lastCleared_ = 0u;
    uint32_t lastReportMs_ = 0u;
    bool strongPending_ = false;
    uint32_t strongEpisode_ = 0u;
    uint32_t silenceHandledEpisode_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_MANAGER_HPP
