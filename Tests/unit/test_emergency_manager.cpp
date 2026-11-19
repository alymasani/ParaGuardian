/**
 ******************************************************************************
 * @file    test_emergency_manager.cpp
 * @brief   EmergencyManager scenarios 26-46, SOS request encoding, persistence
 *          record and command integrity (CRASH_EMERGENCY_VALIDATION_REPORT.md
 *          section 5). Transport, phone and storage are test doubles: nothing
 *          here says anything about a real BLE link or phone.
 ******************************************************************************
 */
#include <cstdio>
#include <cstring>
#include <vector>

#include "TestFramework.hpp"
#include "mocks/MockEmergencyIo.hpp"

#include "Algorithms/Emergency/EmergencyManager.hpp"
#include "Algorithms/Emergency/SosRequest.hpp"
#include "Processing/TimeBase.hpp"

using namespace pg;

namespace {

struct Events final : ISafetyEventSink {
    std::vector<SafetyEvent> ev;
    bool publish(const SafetyEvent& e) override
    {
        ev.push_back(e);
        return true;
    }
    size_t count(SafetyEventType t, SafetyTransition tr) const
    {
        size_t n = 0;
        for (const auto& e : ev) n += (e.type == t && e.transition == tr) ? 1u : 0u;
        return n;
    }
};

CrashReport report(uint32_t now, uint32_t serial, uint32_t episode, CrashState st = CrashState::CrashSuspected,
                   uint32_t cleared = 0u)
{
    CrashReport r{};
    r.publishedMs = now;
    r.status.state = st;
    r.status.suspicionSerial = serial;
    r.status.suspicionsCleared = cleared;
    r.status.episodeId = episode;
    r.status.impact.cls = ImpactClass::Strong;
    r.status.impact.peakAccel_mps2 = 98.0f;
    r.status.impact.peakRate_radps = 10.0f;
    r.status.impact.impacts = 1u;
    r.status.impact.timestampMs = now;
    r.status.evidence = crashev::HighSpecificForce | crashev::AirborneAtImpact | crashev::Inactivity;
    r.context.gnssFixValid = true;
    r.context.latitude_1e7deg = 461234567;
    r.context.longitude_1e7deg = -1141234567;
    r.context.heightMsl_m = 1234.5f;
    r.context.horizontalAccuracy_m = 3.2f;
    r.context.fixType = 3u;
    r.context.numSatellites = 11u;
    r.context.fixTimestampMs = now - 400u;
    r.context.navAltitudeValid = true;
    r.context.navTimestampMs = now;
    r.context.navAltitude_m = 12.3f;
    r.context.navVelocityValid = true;
    r.context.navVerticalVelocity_mps = -0.2f;
    return r;
}

EmergencyCommand cmd(EmergencyCommandType t, uint16_t seq, uint32_t incident, uint32_t issued,
                     CommandSource src = CommandSource::PhoneApp)
{
    return makeEmergencyCommand(t, src, seq, incident, issued);
}

TransportReport rep(TransportReportKind k, uint32_t incident, uint16_t revision, uint32_t now)
{
    return TransportReport{k, revision, incident, now};
}

/// Ticks every `dt` ms over (from, to].
void run(EmergencyManager& m, uint32_t from, uint32_t to, uint32_t dt = 20u)
{
    for (uint32_t t = from + dt; timebase::diffMs(to, t) >= 0; t += dt) m.tick(t);
}

struct Fixture {
    EmergencyConfig cfg{};
    mock::SosTransport tr;
    mock::EmergencyStore store;
    Events events;
    EmergencyManager m;
    explicit Fixture(bool persistent = true, const EmergencyConfig& c = EmergencyConfig{})
        : cfg(c), m(c, tr, persistent ? &store : nullptr)
    {
        m.addSink(&events);
        m.restore(0u);
    }
};

}  // namespace

// ---------------------------------------------------------------------------

TEST(em_26_crash_suspicion_starts_countdown)
{
    Fixture f;
    f.m.observeCrash(report(1000u, 1u, 7u), 1000u);
    const EmergencyStatus& s = f.m.tick(1000u);
    CHECK(s.state == EmergencyState::Countdown);
    CHECK(s.trigger == IncidentTrigger::AutomaticCrashSuspected);
    CHECK_EQ(s.incidentId, 1u);
    CHECK_EQ(s.crashEpisodeId, 7u);
    CHECK_EQ(s.countdownRemainingMs, f.cfg.autoCountdownMs);
    CHECK(!s.requestIssued);
    CHECK_EQ(f.events.count(SafetyEventType::EmergencyCountdown, SafetyTransition::Raised), 1u);
    CHECK(f.events.ev.back().episodeId == 1u && f.events.ev.back().source == SafetySource::Emergency);
    // the same report again (no new serial): nothing new
    f.m.observeCrash(report(1010u, 1u, 7u), 1010u);
    CHECK_EQ(f.m.status().incidents, 1u);
}

TEST(em_27_suspicion_cleared_before_escalation_cancels)
{
    Fixture f;
    f.m.observeCrash(report(1000u, 1u, 7u), 1000u);
    run(f.m, 1000u, 9000u);
    f.m.observeCrash(report(9000u, 1u, 7u, CrashState::Cooldown, 1u), 9000u);
    const EmergencyStatus& s = f.m.tick(9000u);
    CHECK(s.state == EmergencyState::Cancelled);
    CHECK(s.cancelReason == CancelReason::SuspicionCleared);
    CHECK(!s.requestIssued);
    run(f.m, 9000u, 120000u);
    CHECK(f.tr.sent.empty());   // nothing ever left the device
    CHECK(f.m.status().state == EmergencyState::Idle);   // re-armed
}

TEST(em_28_pilot_cancels_countdown_and_rearms)
{
    Fixture f;
    f.m.observeCrash(report(1000u, 1u, 7u), 1000u);
    run(f.m, 1000u, 20000u);
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::CancelSos, 1, 1u, 20000u), 20000u) == CommandResult::Accepted);
    CHECK(f.m.status().state == EmergencyState::Cancelled);
    CHECK(!f.m.status().cancelledAfterSend);
    CHECK(f.m.status().cancelNotice == CancelNotice::None);
    CHECK_EQ(f.events.count(SafetyEventType::EmergencyCancelled, SafetyTransition::Notice), 1u);
    CHECK_EQ(f.events.count(SafetyEventType::EmergencyCountdown, SafetyTransition::Cleared), 1u);
    run(f.m, 20000u, 20000u + f.cfg.cancelRearmMs + 40u);
    CHECK(f.m.status().state == EmergencyState::Idle);
    CHECK(f.tr.sent.empty());
}

TEST(em_29_cancellation_at_the_deadline_and_countdown_accuracy)
{
    // (a) exact expiry time with 1 ms ticks
    {
        Fixture f;
        f.m.observeCrash(report(5000u, 1u, 1u), 5000u);
        uint32_t expiredAt = 0;
        for (uint32_t t = 5001u; t < 5000u + 61000u && expiredAt == 0u; ++t) {
            if (f.m.tick(t).state != EmergencyState::Countdown) expiredAt = t;
        }
        std::printf("    [metric] countdown %u ms: request created %u ms after the start (1 ms ticks)\n",
                    static_cast<unsigned>(f.cfg.autoCountdownMs), static_cast<unsigned>(expiredAt - 5000u));
        CHECK_EQ(expiredAt, 5000u + f.cfg.autoCountdownMs + 1u);
    }
    // (b) cancel issued exactly AT the deadline: in time (inclusive)
    {
        Fixture f;
        f.m.observeCrash(report(5000u, 1u, 1u), 5000u);
        run(f.m, 5000u, 64980u);
        const uint32_t deadline = 5000u + f.cfg.autoCountdownMs;
        CHECK(f.m.handleCommand(cmd(EmergencyCommandType::CancelSos, 1, 1u, deadline), deadline) == CommandResult::Accepted);
        CHECK(f.m.status().state == EmergencyState::Cancelled);
        CHECK(!f.m.status().requestIssued);
    }
    // (c) 1 ms after the deadline: the request exists; nothing has left yet -> withdrawn locally
    {
        Fixture f;
        f.m.observeCrash(report(5000u, 1u, 1u), 5000u);
        run(f.m, 5000u, 64980u);
        const uint32_t late = 5000u + f.cfg.autoCountdownMs + 1u;
        f.m.handleCommand(cmd(EmergencyCommandType::CancelSos, 1, 1u, late), late);
        CHECK(f.m.status().state == EmergencyState::Cancelled);
        CHECK(f.m.status().requestIssued);
        CHECK(!f.m.status().requestLeftDevice);
        CHECK(f.tr.sent.empty());
    }
}

TEST(em_30_manual_sos_without_crash_evidence)
{
    Fixture f;
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 3000u, CommandSource::HardwareInput), 3000u) ==
          CommandResult::Accepted);
    CHECK(f.m.status().state == EmergencyState::Countdown);
    CHECK(f.m.status().trigger == IncidentTrigger::Manual);
    run(f.m, 3000u, 3000u + f.cfg.manualCountdownMs + 40u);
    CHECK(f.m.status().state == EmergencyState::AwaitingPhoneAck);   // created and handed to the transport
    CHECK_EQ(f.tr.sent.size(), 1u);
    const SosRequest& r = f.tr.sent[0].req;
    CHECK(r.trigger == IncidentTrigger::Manual);
    CHECK((r.validity & sosvalid::Impact) == 0u);   // no crash evidence claimed
    CHECK(r.kind == SosRequestKind::Alert);
    std::printf("    [metric] manual SOS: request handed to transport %u ms after the command (countdown %u ms)\n",
                static_cast<unsigned>(f.tr.sent[0].ms - 3000u), static_cast<unsigned>(f.cfg.manualCountdownMs));
}

TEST(em_31_repeated_manual_sos_is_idempotent)
{
    Fixture f;
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 5, 0u, 1000u), 1000u);
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 5, 0u, 1100u), 1100u) == CommandResult::Duplicate);
    // a NEW press during the countdown confirms: escalate immediately, same incident
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 6, 0u, 1500u), 1500u) == CommandResult::Accepted);
    f.m.tick(1500u);
    CHECK(f.m.status().requestIssued);
    CHECK((f.m.status().flags & emflag::ManuallyConfirmed) != 0u);
    // presses while it is being sent: no effect, still ONE incident
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 7, 0u, 2000u), 2000u) == CommandResult::NoEffect);
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 8, 1u, 2100u), 2100u) == CommandResult::NoEffect);
    CHECK_EQ(f.m.status().incidents, 1u);
    CHECK_EQ(f.m.status().sosRequestsCreated, 1u);
    CHECK_EQ(f.m.status().commandsDuplicate, 1u);
}

TEST(em_32_invalid_and_malformed_commands_never_act)
{
    Fixture f;
    f.m.observeCrash(report(1000u, 1u, 1u), 1000u);
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::CancelSos, 1, 2u, 1100u), 1100u) == CommandResult::WrongIncident);
    CHECK(f.m.status().state == EmergencyState::Countdown);
    EmergencyCommand zero{};
    CHECK(f.m.handleCommand(zero, 1200u) == CommandResult::Malformed);
    EmergencyCommand anyCancel = cmd(EmergencyCommandType::CancelSos, 2, 0u, 1200u);   // "cancel whatever"
    CHECK(!isWellFormed(anyCancel));
    CHECK(f.m.handleCommand(anyCancel, 1200u) == CommandResult::Malformed);
    EmergencyCommand corrupt = cmd(EmergencyCommandType::CancelSos, 3, 1u, 1200u);
    corrupt.incidentId ^= 0x10u;   // bit flip after the integrity word was set
    CHECK(f.m.handleCommand(corrupt, 1200u) == CommandResult::Malformed);
    EmergencyCommand badType = cmd(EmergencyCommandType::ManualSos, 4, 0u, 1200u);
    badType.type = static_cast<EmergencyCommandType>(77);
    CHECK(f.m.handleCommand(badType, 1200u) == CommandResult::Malformed);
    CHECK(f.m.status().state == EmergencyState::Countdown);
    CHECK_EQ(f.m.status().commandsRejected, 4u);
    CHECK_EQ(f.m.status().commandsWrongIncident, 1u);
    // in Idle, a malformed manual SOS creates nothing
    Fixture g;
    EmergencyCommand m = cmd(EmergencyCommandType::ManualSos, 1, 0u, 10u);
    m.check = 0u;
    CHECK(g.m.handleCommand(m, 10u) == CommandResult::Malformed);
    CHECK_EQ(g.m.status().incidents, 0u);
}

TEST(em_33_34_expiry_creates_request_transport_unavailable_is_not_an_attempt)
{
    Fixture f;
    f.tr.link = false;
    f.m.observeCrash(report(0u, 1u, 1u), 0u);
    run(f.m, 0u, 60100u);
    const EmergencyStatus& s = f.m.status();
    CHECK(s.state == EmergencyState::SosPending);
    CHECK(s.stage == DeliveryStage::Created);
    CHECK((s.flags & emflag::TransportUnavailable) != 0u);
    CHECK_EQ(f.events.count(SafetyEventType::EmergencyRequest, SafetyTransition::Notice), 1u);
    CHECK_EQ(f.events.count(SafetyEventType::SosTransportUnavailable, SafetyTransition::Raised), 1u);
    // 10 minutes without a link: still pending, no attempt consumed, never "failed"
    run(f.m, 60100u, 660100u, 100u);
    CHECK(f.m.status().state == EmergencyState::SosPending);
    CHECK_EQ(f.m.status().attempts, 0u);
    CHECK_EQ(f.m.status().transportAttempts, 0u);
    // the reliable record still holds the request (independent of any event queue)
    const PendingSosRecord rec = f.m.pendingRecord();
    CHECK(rec.active);
    CHECK_EQ(rec.request.incidentId, 1u);
    // link comes back: sent at once, unavailable cleared
    f.tr.link = true;
    f.m.handleReport(rep(TransportReportKind::LinkUp, 0u, 0u, 660200u), 660200u);
    f.m.tick(660200u);
    CHECK_EQ(f.tr.sent.size(), 1u);
    CHECK(f.m.status().state == EmergencyState::AwaitingPhoneAck);
    CHECK_EQ(f.events.count(SafetyEventType::SosTransportUnavailable, SafetyTransition::Cleared), 1u);
}

TEST(em_35_36_disconnect_reconnect_and_retry_same_incident)
{
    Fixture f;
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    run(f.m, 0u, 5100u);
    CHECK_EQ(f.tr.sent.size(), 1u);
    // write completed at transport level: NOT phone acceptance
    f.m.handleReport(rep(TransportReportKind::WriteCompleted, 1u, 1u, 5200u), 5200u);
    CHECK(f.m.status().stage == DeliveryStage::TransferredToPhone);
    CHECK(f.m.status().state == EmergencyState::AwaitingPhoneAck);
    // phone disconnects before accepting
    f.tr.link = false;
    f.m.handleReport(rep(TransportReportKind::LinkDown, 0u, 0u, 6000u), 6000u);
    CHECK(f.m.status().state == EmergencyState::SosPending);
    run(f.m, 6000u, 30000u);
    CHECK_EQ(f.tr.sent.size(), 1u);
    f.tr.link = true;
    f.m.handleReport(rep(TransportReportKind::LinkUp, 0u, 0u, 30000u), 30000u);
    f.m.tick(30000u);
    CHECK_EQ(f.tr.sent.size(), 2u);
    CHECK_EQ(f.tr.sent[1].req.incidentId, f.tr.sent[0].req.incidentId);   // idempotent: same incident
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 30500u), 30500u);
    CHECK(f.m.status().state == EmergencyState::AwaitingDelivery);
    CHECK_EQ(f.m.status().attempts, 2u);
}

TEST(em_37_duplicate_acknowledgment_is_ignored)
{
    Fixture f;
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    run(f.m, 0u, 5100u);
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 5200u), 5200u);
    const uint32_t evs = static_cast<uint32_t>(f.events.ev.size());
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 5300u), 5300u);
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 5400u), 5400u);
    CHECK_EQ(f.m.status().duplicateAcks, 2u);
    CHECK_EQ(static_cast<uint32_t>(f.events.ev.size()), evs);   // no repeated notice
    CHECK_EQ(f.m.status().incidents, 1u);
    CHECK(f.m.status().state == EmergencyState::AwaitingDelivery);
}

TEST(em_38_ack_timeout_backoff_late_ack_and_retry_exhaustion)
{
    Fixture f;
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    run(f.m, 0u, 5000u + 26100u, 10u);
    // attempts at t1 = 5001 (+1 ms tick granularity), t1 + 10 s timeout + 2 s, then + 10 s + 4 s
    CHECK(f.tr.sent.size() >= 3u);
    const uint32_t d1 = f.tr.sent[1].ms - f.tr.sent[0].ms, d2 = f.tr.sent[2].ms - f.tr.sent[1].ms;
    std::printf("    [metric] retry spacing: %u ms, %u ms (ack timeout %u + backoff %u, %u)\n", static_cast<unsigned>(d1),
                static_cast<unsigned>(d2), static_cast<unsigned>(f.cfg.ackTimeoutMs),
                static_cast<unsigned>(f.cfg.retryBaseMs), static_cast<unsigned>(2u * f.cfg.retryBaseMs));
    CHECK(d1 >= 12000u && d1 <= 12020u);
    CHECK(d2 >= 14000u && d2 <= 14020u);
    // a LATE acceptance of revision 1 is still valid for this incident
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 31200u), 31200u);
    CHECK(f.m.status().state == EmergencyState::AwaitingDelivery);

    // exhaustion: the phone never answers
    Fixture g;
    g.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    uint32_t t = 0;
    while (g.m.status().state != EmergencyState::DeliveryFailed && t < 3600000u) {
        t += 100u;
        g.m.tick(t);
    }
    CHECK(g.m.status().state == EmergencyState::DeliveryFailed);
    CHECK(g.m.status().failure == FailureReason::RetriesExhausted);
    CHECK_EQ(static_cast<uint32_t>(g.tr.sent.size()), static_cast<uint32_t>(g.cfg.maxAttempts));
    std::printf("    [metric] %u attempts without phone acceptance -> DeliveryFailed after %.1f s\n",
                static_cast<unsigned>(g.cfg.maxAttempts), (t - 5000u) * 1e-3);
    // reconnect starts ONE more bounded cycle (same incident)
    g.m.handleReport(rep(TransportReportKind::LinkUp, 0u, 0u, t + 100u), t + 100u);
    CHECK(g.m.status().state == EmergencyState::SosPending);
    CHECK_EQ(g.m.status().cycles, 2u);
    CHECK_EQ(g.m.status().incidents, 1u);
}

TEST(em_39_wrong_incident_or_future_revision_reports_are_ignored)
{
    Fixture f;
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    run(f.m, 0u, 5100u);
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 99u, 1u, 5200u), 5200u);
    f.m.handleReport(rep(TransportReportKind::DeliveryConfirmed, 99u, 1u, 5200u), 5200u);
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 9u, 5200u), 5200u);   // never sent revision 9
    f.m.handleReport(TransportReport{static_cast<TransportReportKind>(200), 1u, 1u, 5200u}, 5200u);
    CHECK_EQ(f.m.status().wrongIncidentReports, 2u);
    CHECK_EQ(f.m.status().invalidReports, 2u);
    CHECK(f.m.status().state == EmergencyState::AwaitingPhoneAck);
    CHECK(f.m.status().stage == DeliveryStage::QueuedForTransport);
}

TEST(em_40_accepted_but_no_outcome_is_delivery_unknown)
{
    Fixture f;
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    run(f.m, 0u, 5100u);
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 6000u), 6000u);
    run(f.m, 6000u, 6000u + f.cfg.deliveryTimeoutMs + 100u, 100u);
    CHECK(f.m.status().state == EmergencyState::DeliveryUnknown);   // honest: never turned into success
    CHECK(f.m.status().stage == DeliveryStage::AcceptedByPhone);
    CHECK_EQ(f.events.count(SafetyEventType::SosDeliveryUnknown, SafetyTransition::Notice), 1u);
    // a later report may still resolve it
    f.m.handleReport(rep(TransportReportKind::DeliveryConfirmed, 1u, 1u, 200000u), 200000u);
    CHECK(f.m.status().state == EmergencyState::Delivered);
}

TEST(em_41_explicit_delivery_failure_and_manual_resend)
{
    Fixture f;
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    run(f.m, 0u, 5100u);
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 6000u), 6000u);
    f.m.handleReport(rep(TransportReportKind::PhoneSendAttempted, 1u, 1u, 6500u), 6500u);
    f.m.handleReport(rep(TransportReportKind::DeliveryFailed, 1u, 1u, 9000u), 9000u);
    CHECK(f.m.status().state == EmergencyState::DeliveryFailed);
    CHECK(f.m.status().failure == FailureReason::PhoneReportedFailure);
    CHECK_EQ(f.events.count(SafetyEventType::SosDeliveryFailed, SafetyTransition::Notice), 1u);
    // the pilot presses SOS again: same incident, next revision, sent again
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 2, 0u, 20000u), 20000u);
    f.m.tick(20000u);
    CHECK_EQ(f.tr.sent.size(), 2u);
    CHECK_EQ(f.tr.sent[1].req.incidentId, 1u);
    CHECK_EQ(f.tr.sent[1].req.revision, 2u);
    CHECK_EQ(f.m.status().incidents, 1u);
}

TEST(em_42_delivery_confirmation_provenance)
{
    Fixture f;
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    run(f.m, 0u, 5100u);
    // transport acceptance alone is only QueuedForTransport
    CHECK(f.m.status().stage == DeliveryStage::QueuedForTransport);
    CHECK((f.m.status().reportedStages & stagebit::AcceptedByPhone) == 0u);
    f.m.handleReport(rep(TransportReportKind::WriteCompleted, 1u, 1u, 5200u), 5200u);
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 5400u), 5400u);
    CHECK(f.m.status().state == EmergencyState::AwaitingDelivery);
    f.m.handleReport(rep(TransportReportKind::PhoneSendAttempted, 1u, 1u, 5600u), 5600u);
    CHECK(f.m.status().state == EmergencyState::AwaitingDelivery);   // a send ATTEMPT is not delivery
    f.m.handleReport(rep(TransportReportKind::DeliveryConfirmed, 1u, 1u, 9000u), 9000u);
    CHECK(f.m.status().state == EmergencyState::Delivered);
    const uint16_t all = stagebit::Created | stagebit::QueuedForTransport | stagebit::TransferredToPhone |
                         stagebit::AcceptedByPhone | stagebit::PhoneSendAttempted | stagebit::DeliveryConfirmed;
    CHECK_EQ(f.m.status().reportedStages, all);
    CHECK((f.m.status().flags & emflag::ProvenanceIncomplete) == 0u);
    // a contradicting failure after confirmation is counted, not applied
    f.m.handleReport(rep(TransportReportKind::DeliveryFailed, 1u, 1u, 9100u), 9100u);
    CHECK(f.m.status().state == EmergencyState::Delivered);

    // confirmation WITHOUT a reported acceptance: accepted, but flagged; earlier bit stays unset
    Fixture g;
    g.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    run(g.m, 0u, 5100u);
    g.m.handleReport(rep(TransportReportKind::DeliveryConfirmed, 1u, 1u, 7000u), 7000u);
    CHECK(g.m.status().state == EmergencyState::Delivered);
    CHECK((g.m.status().flags & emflag::ProvenanceIncomplete) != 0u);
    CHECK((g.m.status().reportedStages & stagebit::AcceptedByPhone) == 0u);
}

TEST(em_43_44_countdown_without_sensors_and_scheduler_gap)
{
    // 43: no crash report after the start (sensors / processing silent): ticks alone expire it.
    Fixture f;
    f.m.observeCrash(report(1000u, 1u, 1u), 1000u);
    run(f.m, 1000u, 1000u + f.cfg.autoCountdownMs + 40u);
    CHECK(f.m.status().requestIssued);
    // 44: no tick at all for 75 s; a cancel issued at 30 s (before the deadline) is processed late but honoured
    Fixture g;
    g.m.observeCrash(report(1000u, 1u, 1u), 1000u);
    g.m.tick(1000u);
    g.m.handleCommand(cmd(EmergencyCommandType::CancelSos, 1, 1u, 31000u), 76000u);
    CHECK(g.m.status().state == EmergencyState::Cancelled);
    CHECK(!g.m.status().requestIssued);
    // same gap without a cancel: expiry evaluated late, flagged, request created
    Fixture h;
    h.m.observeCrash(report(1000u, 1u, 1u), 1000u);
    h.m.tick(1000u);
    h.m.tick(76000u);
    CHECK(h.m.status().requestIssued);
    CHECK((h.m.status().flags & emflag::CountdownOverrun) != 0u);
    std::printf("    [metric] 75 s scheduler gap: expiry evaluated %u ms late (flagged CountdownOverrun)\n",
                static_cast<unsigned>(h.m.status().countdownOverrunMs));
    CHECK_EQ(h.m.status().countdownOverrunMs, 76000u - (1000u + h.cfg.autoCountdownMs));   // 15 s late
}

TEST(em_45_reboot_recovery_with_mock_persistence)
{
    mock::SosTransport tr;
    mock::EmergencyStore store;
    {
        EmergencyManager a(EmergencyConfig{}, tr, &store);
        a.restore(0u);
        CHECK((a.status().flags & emflag::PersistenceUnavailable) == 0u);
        a.observeCrash(report(1000u, 1u, 3u), 1000u);
        run(a, 1000u, 61100u);
        CHECK(a.status().state == EmergencyState::AwaitingPhoneAck);
        CHECK(store.writes >= 3u);
    }   // --- reset: RAM state gone, the store survives ---
    tr.sent.clear();
    EmergencyManager b(EmergencyConfig{}, tr, &store);
    b.restore(500u);
    CHECK(b.status().state == EmergencyState::SosPending);
    CHECK_EQ(b.status().incidentId, 1u);
    CHECK((b.status().flags & emflag::RestoredAfterReset) != 0u);
    b.tick(520u);
    CHECK_EQ(tr.sent.size(), 1u);
    CHECK_EQ(tr.sent[0].req.incidentId, 1u);
    CHECK_EQ(tr.sent[0].req.revision, 2u);
    CHECK((tr.sent[0].req.validity & sosvalid::Restored) != 0u);
    // ids continue after a reset (not reused)
    b.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 2u, 600u), 600u);
    b.handleReport(rep(TransportReportKind::DeliveryConfirmed, 1u, 2u, 700u), 700u);
    b.handleCommand(cmd(EmergencyCommandType::ResetIncident, 1, 1u, 800u), 800u);
    b.handleCommand(cmd(EmergencyCommandType::ManualSos, 2, 0u, 900u), 900u);
    CHECK_EQ(b.status().incidentId, 2u);

    // reset during a countdown: the FULL countdown restarts (the pilot can still cancel)
    mock::EmergencyStore s2;
    {
        EmergencyManager a(EmergencyConfig{}, tr, &s2);
        a.restore(0u);
        a.observeCrash(report(1000u, 1u, 3u), 1000u);
        run(a, 1000u, 40000u);
    }
    EmergencyManager c(EmergencyConfig{}, tr, &s2);
    c.restore(100u);
    CHECK(c.status().state == EmergencyState::Countdown);
    CHECK_EQ(c.tick(100u).countdownRemainingMs, EmergencyConfig{}.autoCountdownMs);

    // a corrupted record is ignored (no invented incident)
    s2.bytes[13] ^= 0xFFu;
    EmergencyManager d(EmergencyConfig{}, tr, &s2);
    d.restore(0u);
    CHECK(d.status().state == EmergencyState::Idle);

    // no persistence (firmware today): stated, not hidden
    EmergencyManager e(EmergencyConfig{}, tr, nullptr);
    e.restore(0u);
    CHECK((e.status().flags & emflag::PersistenceUnavailable) != 0u);
    CHECK((e.status().flags & emflag::IncidentIdVolatile) != 0u);
}

TEST(em_46_rearm_new_incident_and_repeated_observations)
{
    Fixture f;
    f.m.observeCrash(report(1000u, 1u, 3u), 1000u);
    f.m.observeCrash(report(2000u, 2u, 4u), 2000u);   // another suspicion during the countdown
    CHECK_EQ(f.m.status().incidents, 1u);
    CHECK_EQ(f.m.status().additionalImpacts, 1u);
    run(f.m, 2000u, 61100u);
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 61200u), 61200u);
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::ResetIncident, 1, 1u, 61300u), 61300u) ==
          CommandResult::NotAllowed);   // still sending: use CancelSos
    f.m.handleReport(rep(TransportReportKind::DeliveryConfirmed, 1u, 1u, 70000u), 70000u);
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::ResetIncident, 2, 2u, 70100u), 70100u) ==
          CommandResult::WrongIncident);
    CHECK(f.m.handleCommand(cmd(EmergencyCommandType::ResetIncident, 3, 1u, 70100u), 70100u) == CommandResult::Accepted);
    CHECK(f.m.status().state == EmergencyState::Idle);
    // a duplicate acknowledgment of the closed incident creates nothing
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 70200u), 70200u);
    CHECK_EQ(f.m.status().incidents, 1u);
    CHECK(f.m.status().state == EmergencyState::Idle);
    // a new suspicion later: a new incident with a new id
    f.m.observeCrash(report(200000u, 3u, 9u), 200000u);
    CHECK(f.m.status().state == EmergencyState::Countdown);
    CHECK_EQ(f.m.status().incidentId, 2u);
}

TEST(em_cancel_after_send_sends_a_cancellation_notice)
{
    Fixture f;
    f.m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u);
    run(f.m, 0u, 5100u);
    f.m.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 1u, 5200u), 5200u);
    f.m.handleCommand(cmd(EmergencyCommandType::CancelSos, 2, 1u, 8000u), 8000u);
    CHECK(f.m.status().state == EmergencyState::Cancelled);
    CHECK(f.m.status().cancelledAfterSend);   // the alert may already be out: stated explicitly
    f.m.tick(8000u);
    CHECK_EQ(f.tr.sent.size(), 2u);
    CHECK(f.tr.sent[1].req.kind == SosRequestKind::Cancel);
    CHECK_EQ(f.tr.sent[1].req.incidentId, 1u);
    CHECK(f.m.status().cancelNotice == CancelNotice::Sent);
    f.m.handleReport(rep(TransportReportKind::CancelAccepted, 1u, 2u, 8500u), 8500u);
    CHECK(f.m.status().cancelNotice == CancelNotice::Accepted);
    run(f.m, 8500u, 8500u + f.cfg.cancelRearmMs + 40u);
    CHECK(f.m.status().state == EmergencyState::Idle);
}

TEST(em_detector_silence_after_strong_impact_opens_incident)
{
    Fixture f;
    CrashReport r = report(10000u, 0u, 5u, CrashState::PostImpactObservation);
    f.m.observeCrash(r, 10000u);
    // the processing task stops: the same (old) report is read again and again
    for (uint32_t t = 10020u; t < 16000u; t += 20u) {
        f.m.observeCrash(r, t);
        f.m.tick(t);
    }
    CHECK(f.m.status().state == EmergencyState::Countdown);
    CHECK(f.m.status().trigger == IncidentTrigger::DetectorSilence);
    CHECK_EQ(f.m.status().detectorSilenceIncidents, 1u);
    // only once per crash episode
    f.m.handleCommand(cmd(EmergencyCommandType::CancelSos, 1, 1u, 16000u), 16000u);
    for (uint32_t t = 16020u; t < 40000u; t += 20u) {
        f.m.observeCrash(r, t);
        f.m.tick(t);
    }
    CHECK_EQ(f.m.status().incidents, 1u);
}

TEST(em_invalid_config_is_unavailable)
{
    EmergencyConfig c;
    c.maxAttempts = 0u;
    mock::SosTransport tr;
    EmergencyManager m(c, tr, nullptr);
    CHECK(!m.configValid());
    CHECK(m.tick(0u).state == EmergencyState::Unavailable);
    CHECK(m.handleCommand(cmd(EmergencyCommandType::ManualSos, 1, 0u, 0u), 0u) == CommandResult::NotAllowed);
}

// ============================== SOS request =================================

TEST(sos_request_encoding_roundtrip_and_integrity)
{
    Fixture f;
    f.m.observeCrash(report(1000u, 1u, 3u), 1000u);
    run(f.m, 1000u, 60900u);
    f.m.observeCrash(report(60900u, 1u, 3u), 60900u);   // the processing task refreshes the context every frame
    run(f.m, 60900u, 61100u);
    CHECK_EQ(f.tr.sent.size(), 1u);
    const SosRequest r = f.tr.sent[0].req;
    CHECK_EQ(r.version, kSosProtocolVersion);
    CHECK((r.validity & sosvalid::Position) != 0u);
    CHECK((r.validity & sosvalid::HeightMsl) != 0u);
    CHECK((r.validity & sosvalid::RelativeAltitude) != 0u);
    CHECK_EQ(r.heightMsl_dm, 12345);          // GNSS MSL ...
    CHECK_EQ(r.relativeAltitude_dm, 123);     // ... and barometric relative altitude: separate fields
    CHECK(r.fixAgeMs >= 400u);                // age of the LAST VALID fix, stated
    // navigation values older than 5 s are not sent (they carry no age of their own)
    {
        Fixture g;
        g.m.observeCrash(report(1000u, 1u, 3u), 1000u);
        run(g.m, 1000u, 61100u);
        CHECK((g.tr.sent[0].req.validity & sosvalid::RelativeAltitude) == 0u);
        CHECK((g.tr.sent[0].req.validity & sosvalid::Position) != 0u);   // position: sent WITH its age
    }
    CHECK_EQ(r.peakAccel_cg, 999u);           // 98 m/s^2 = 9.99 g
    uint8_t buf[kSosFrameBytes];
    CHECK_EQ(encodeSosRequest(r, buf, sizeof(buf)), kSosFrameBytes);
    SosRequest back{};
    CHECK(decodeSosRequest(buf, sizeof(buf), back));
    CHECK_EQ(back.incidentId, r.incidentId);
    CHECK_EQ(back.latitude_1e7deg, r.latitude_1e7deg);
    CHECK_EQ(back.longitude_1e7deg, r.longitude_1e7deg);
    CHECK_EQ(back.verticalSpeed_cmps, r.verticalSpeed_cmps);
    CHECK_EQ(back.crashEvidence, r.crashEvidence);
    for (size_t i = 0; i < kSosFrameBytes; ++i) {   // every single-bit error is detected
        uint8_t c[kSosFrameBytes];
        std::memcpy(c, buf, sizeof(c));
        c[i] ^= 0x01u;
        SosRequest x{};
        CHECK(!decodeSosRequest(c, sizeof(c), x));
    }
    CHECK_EQ(encodeSosRequest(r, buf, kSosFrameBytes - 1u), 0u);   // bounded
    uint8_t v2[kSosFrameBytes];
    std::memcpy(v2, buf, sizeof(v2));
    v2[0] = 2u;
    {   // re-seal with a valid CRC
        const uint32_t crc = crc32Ieee(v2, kSosFrameBytes - 4u);
        for (int k = 0; k < 4; ++k) v2[60 + k] = static_cast<uint8_t>(crc >> (8 * k));
    }
    SosRequest y{};
    CHECK(!decodeSosRequest(v2, sizeof(v2), y));   // unknown version rejected even with a valid CRC
    // legacy (unversioned) payload mapping: MSL only, never the barometric altitude
    const SosPayload p = toLegacySosPayload(r);
    CHECK(p.positionValid);
    CHECK_EQ(p.altitudeMsl_dm, 12345);
    CHECK(p.reason == SosReason::NoMotionAfterImpact);
    CHECK_EQ(p.batteryMillivolts, 0u);   // unknown, not invented
}

TEST(sos_request_without_gnss_states_no_position)
{
    mock::SosTransport tr;
    EmergencyManager m(EmergencyConfig{}, tr, nullptr);
    m.restore(0u);
    CrashReport r = report(1000u, 1u, 3u);
    r.context = IncidentContext{};   // never had a fix, no navigation
    m.observeCrash(r, 1000u);
    run(m, 1000u, 61100u);
    CHECK_EQ(tr.sent.size(), 1u);
    const SosRequest& q = tr.sent[0].req;
    CHECK((q.validity & sosvalid::Position) == 0u);
    CHECK((q.validity & sosvalid::HeightMsl) == 0u);
    CHECK((q.validity & sosvalid::RelativeAltitude) == 0u);
    CHECK_EQ(q.latitude_1e7deg, 0);
    CHECK_EQ(q.fixAgeMs, UINT32_MAX);
    CHECK((q.validity & sosvalid::IncidentIdVolatile) != 0u);
    // an old fix is flagged stale, not presented as current
    mock::SosTransport tr2;
    EmergencyManager m2(EmergencyConfig{}, tr2, nullptr);
    CrashReport s = report(100000u, 1u, 3u);
    s.context.fixTimestampMs = 1000u;   // 99 s before the incident
    m2.observeCrash(s, 100000u);
    run(m2, 100000u, 160100u);
    CHECK((tr2.sent[0].req.validity & sosvalid::PositionStale) != 0u);
    CHECK(tr2.sent[0].req.fixAgeMs > 150000u);
}

TEST(persist_record_roundtrip_and_rejects_corruption)
{
    EmergencyPersistRecord r{};
    r.nextIncidentId = 42u;
    r.incidentId = 41u;
    r.state = EmergencyState::AwaitingDelivery;
    r.trigger = IncidentTrigger::Manual;
    r.stage = DeliveryStage::AcceptedByPhone;
    r.requestRevision = 3u;
    r.reportedStages = 0x0Fu;
    r.requestIssued = r.requestLeftDevice = true;
    uint8_t b[kEmergencyRecordBytes];
    CHECK_EQ(encodeEmergencyRecord(r, b, sizeof(b)), kEmergencyRecordBytes);
    EmergencyPersistRecord o{};
    CHECK(decodeEmergencyRecord(b, sizeof(b), o));
    CHECK_EQ(o.incidentId, 41u);
    CHECK_EQ(o.requestRevision, 3u);
    CHECK(o.state == EmergencyState::AwaitingDelivery);
    b[5] = 0xEEu;   // invalid state value (CRC now wrong as well)
    CHECK(!decodeEmergencyRecord(b, sizeof(b), o));
}

TEST(emergency_command_inbox_is_bounded)
{
    mock::NoLock lock;
    BoundedInbox<EmergencyCommand, 4> inbox(lock);
    for (uint16_t i = 0; i < 6u; ++i) inbox.push(cmd(EmergencyCommandType::RequestStatus, i, 0u, i));
    CHECK_EQ(inbox.size(), 4u);
    CHECK_EQ(inbox.dropped(), 2u);
    EmergencyCommand c{};
    CHECK(inbox.pop(c));
    CHECK_EQ(c.sequence, 0u);   // FIFO; the newest were dropped, nothing replaced
}
