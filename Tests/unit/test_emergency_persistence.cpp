/**
 ******************************************************************************
 * @file    test_emergency_persistence.cpp
 * @brief   Milestone 8: reset recovery and incident identity with the REAL
 *          chain EmergencyManager -> JournaledEmergencyStore -> StorageService
 *          -> EmergencyJournal -> FakeNorFlash. A "reset" destroys every RAM
 *          object and keeps only the flash contents.
 ******************************************************************************
 */
#include <cstdio>
#include <memory>
#include <set>
#include <vector>

#include "TestFramework.hpp"
#include "mocks/FakeNorFlash.hpp"
#include "mocks/MockEmergencyIo.hpp"
#include "Algorithms/Emergency/EmergencyManager.hpp"
#include "Processing/TimeBase.hpp"
#include "Storage/JournaledEmergencyStore.hpp"
#include "Storage/StorageService.hpp"

using namespace pg;

namespace {

EmergencyCommand manual(uint16_t seq, uint32_t now)
{
    return makeEmergencyCommand(EmergencyCommandType::ManualSos, CommandSource::Test, seq, 0u, now);
}
TransportReport rep(TransportReportKind k, uint32_t incident, uint16_t revision, uint32_t now)
{
    return TransportReport{k, revision, incident, now};
}

/// Everything that lives in RAM on the MCU. Destroying it = a reset / power cut.
struct Device {
    mock::NoLock lock;
    JournaledEmergencyStore store{lock};
    LogQueue<8> critical{lock};
    LogQueue<8> telemetry{lock};
    StorageService svc;
    EmergencyManager mgr;
    bool storageRuns = true;   ///< false: the storage task is starved / stuck (nothing gets committed)

    Device(FakeNorFlash& f, mock::SosTransport& tr, const EmergencyConfig& cfg, uint32_t now)
        : svc(f, store, critical, telemetry, StorageServiceConfig{}, &f), mgr(cfg, tr, &store)
    {
        svc.start(now);          // StorageTask mounts first ...
        mgr.restore(now);        // ... the emergency task restores after mounted()
        settle(now);
    }
    void settle(uint32_t now)
    {
        if (!storageRuns) return;
        for (int i = 0; i < 16 && svc.step(now); ++i) {}
        svc.step(now);
    }
    /// Emergency task every 20 ms, storage task after it (lower priority).
    void run(uint32_t from, uint32_t to)
    {
        for (uint32_t t = from + 20u; timebase::diffMs(to, t) >= 0; t += 20u) {
            mgr.tick(t);
            settle(t);
        }
    }
};

EmergencyConfig quick()
{
    EmergencyConfig c{};
    c.manualCountdownMs = 1000u;
    c.autoCountdownMs = 30000u;
    return c;
}

}  // namespace

TEST(persist_first_emergency_blank_journal_id_durable_before_sos_leaves)
{
    FakeNorFlash f;
    mock::SosTransport tr;
    Device d(f, tr, quick(), 0u);
    CHECK(d.store.mountState() == StoreMount::Blank);
    CHECK(d.mgr.status().persist == PersistState::Pending);     // queued is reported as queued ...
    d.mgr.tick(0u);
    CHECK(d.mgr.status().persist == PersistState::Committed);   // ... until the storage task committed it
    CHECK_EQ(d.mgr.status().flags & (emflag::PersistenceUnavailable | emflag::IncidentIdVolatile), 0u);
    d.mgr.handleCommand(manual(1, 100u), 100u);
    CHECK_EQ(d.mgr.status().incidentId, 1u);
    d.run(100u, 1300u);
    CHECK_EQ(tr.sent.size(), 1u);
    CHECK(d.mgr.identityDurable(1u));
    CHECK_EQ(tr.sent[0].req.validity & sosvalid::IncidentIdVolatile, 0u);
    CHECK(d.mgr.status().state == EmergencyState::AwaitingPhoneAck);
}

TEST(persist_sos_waits_bounded_for_identity_then_goes_out_marked_volatile)
{
    FakeNorFlash f;
    mock::SosTransport tr;
    Device d(f, tr, quick(), 0u);
    d.storageRuns = false;   // storage task starved: records stay queued
    d.mgr.handleCommand(manual(1, 0u), 0u);
    d.run(0u, 1500u);        // countdown expired at 1000 ms, request created
    CHECK(d.mgr.status().requestIssued);
    CHECK(d.mgr.status().persist == PersistState::Pending);
    CHECK_EQ(tr.sent.size(), 0u);   // held: its id is not durable yet
    d.storageRuns = true;
    d.run(1500u, 1600u);
    CHECK_EQ(tr.sent.size(), 1u);   // committed -> sent with a durable id
    CHECK_EQ(tr.sent[0].req.validity & sosvalid::IncidentIdVolatile, 0u);

    // storage permanently failing: help is NOT withheld beyond identityHoldMs
    FakeNorFlash g;
    mock::SosTransport tr2;
    Device e(g, tr2, quick(), 0u);
    g.timeoutWrites = true;
    e.mgr.handleCommand(manual(1, 0u), 0u);
    e.run(0u, 1000u);
    CHECK_EQ(tr2.sent.size(), 0u);   // countdown
    e.run(1000u, 3100u);             // the store REPORTS the failure: no point waiting the full hold
    CHECK_EQ(tr2.sent.size(), 1u);
    CHECK((tr2.sent[0].req.validity & sosvalid::IncidentIdVolatile) != 0u);
    CHECK((e.mgr.status().flags & emflag::IdentityUnconfirmed) != 0u);
    CHECK((e.mgr.status().flags & emflag::PersistenceDegraded) != 0u);
    CHECK(e.mgr.status().persist == PersistState::Failed);
}

TEST(persist_reboot_during_countdown_same_id_and_reset_loop_escalates)
{
    FakeNorFlash f;
    mock::SosTransport tr;
    {
        Device d(f, tr, quick(), 0u);
        d.mgr.handleCommand(manual(1, 0u), 0u);
        d.run(0u, 500u);   // countdown running
        CHECK(d.mgr.status().state == EmergencyState::Countdown);
    }
    for (uint8_t k = 1; k <= 2u; ++k) {   // two resets: countdown restarts in full each time
        Device d(f, tr, quick(), 0u);
        CHECK(d.mgr.status().state == EmergencyState::Countdown);
        CHECK_EQ(d.mgr.status().incidentId, 1u);
        CHECK_EQ(d.mgr.status().countdownRestarts, k);
        CHECK_EQ(d.mgr.tick(0u).countdownRemainingMs, quick().manualCountdownMs);
        CHECK_EQ(tr.sent.size(), 0u);
    }
    Device d(f, tr, quick(), 0u);   // third reset during the same countdown: no more postponing
    CHECK((d.mgr.status().flags & emflag::RepeatedResetEscalated) != 0u);
    CHECK(d.mgr.status().requestIssued);
    d.run(0u, 100u);
    CHECK_EQ(tr.sent.size(), 1u);
    CHECK_EQ(tr.sent[0].req.incidentId, 1u);
}

TEST(persist_reboot_during_transmission_and_after_phone_acceptance_reuses_identity)
{
    FakeNorFlash f;
    mock::SosTransport tr;
    {
        Device d(f, tr, quick(), 0u);
        d.mgr.handleCommand(manual(1, 0u), 0u);
        d.run(0u, 1100u);
        CHECK(d.mgr.status().state == EmergencyState::AwaitingPhoneAck);
    }
    tr.sent.clear();
    {
        Device d(f, tr, quick(), 50u);
        CHECK(d.mgr.status().state == EmergencyState::SosPending);
        d.run(50u, 200u);
        CHECK_EQ(tr.sent.size(), 1u);
        CHECK_EQ(tr.sent[0].req.incidentId, 1u);   // same incident, NOT a new one
        CHECK_EQ(tr.sent[0].req.revision, 2u);     // newer revision of the same request
        CHECK((tr.sent[0].req.validity & sosvalid::Restored) != 0u);
        d.mgr.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 2u, 300u), 300u);
        d.run(300u, 400u);
        CHECK(d.mgr.status().state == EmergencyState::AwaitingDelivery);
    }
    tr.sent.clear();
    {
        Device d(f, tr, quick(), 0u);   // reboot after phone acceptance
        CHECK_EQ(d.mgr.status().incidentId, 1u);
        CHECK(d.mgr.status().stage == DeliveryStage::AcceptedByPhone);
        d.run(0u, 100u);
        CHECK_EQ(tr.sent.size(), 1u);
        CHECK_EQ(tr.sent[0].req.incidentId, 1u);   // the phone / backend de-duplicate on this id
        d.mgr.handleReport(rep(TransportReportKind::PhoneAccepted, 1u, 3u, 150u), 150u);
        d.mgr.handleReport(rep(TransportReportKind::DeliveryConfirmed, 1u, 3u, 160u), 160u);
        d.run(160u, 300u);
        CHECK(d.mgr.status().state == EmergencyState::Delivered);
    }
    tr.sent.clear();
    {
        Device d(f, tr, quick(), 0u);   // reboot after delivery: nothing is re-sent
        CHECK(d.mgr.status().state == EmergencyState::Delivered);
        d.run(0u, 3000u);
        CHECK_EQ(tr.sent.size(), 0u);
        d.mgr.handleCommand(makeEmergencyCommand(EmergencyCommandType::ResetIncident, CommandSource::Test, 2, 1u, 3000u),
                            3000u);
        d.mgr.handleCommand(manual(3, 3100u), 3100u);
        CHECK_EQ(d.mgr.status().incidentId, 2u);   // new incident, new id
    }
}

TEST(persist_reboot_during_cancellation_resends_cancel_notice)
{
    FakeNorFlash f;
    mock::SosTransport tr;
    {
        Device d(f, tr, quick(), 0u);
        d.mgr.handleCommand(manual(1, 0u), 0u);
        d.run(0u, 1100u);   // sent
        tr.link = false;
        d.mgr.handleCommand(makeEmergencyCommand(EmergencyCommandType::CancelSos, CommandSource::Test, 2, 1u, 1200u), 1200u);
        d.run(1200u, 1300u);
        CHECK(d.mgr.status().state == EmergencyState::Cancelled);
        CHECK(d.mgr.status().cancelNotice == CancelNotice::Pending);
    }
    tr.link = true;
    tr.sent.clear();
    Device d(f, tr, quick(), 0u);
    CHECK(d.mgr.status().state == EmergencyState::Cancelled);
    CHECK(d.mgr.status().cancelledAfterSend);
    d.run(0u, 100u);
    CHECK_EQ(tr.sent.size(), 1u);
    CHECK(tr.sent[0].req.kind == SosRequestKind::Cancel);
    CHECK_EQ(tr.sent[0].req.incidentId, 1u);
}

TEST(persist_incidents_across_many_resets_never_share_an_identity)
{
    FakeNorFlash f;
    mock::SosTransport tr;
    std::set<uint32_t> ids;
    uint32_t sends = 0;
    auto alerts = [&tr]() {
        uint32_t n = 0;
        for (const auto& x : tr.sent) n += x.req.kind == SosRequestKind::Alert ? 1u : 0u;
        return n;
    };
    auto lastAlertId = [&tr]() {
        uint32_t id = 0;
        for (const auto& x : tr.sent) if (x.req.kind == SosRequestKind::Alert) id = x.req.incidentId;
        return id;
    };
    for (int boot = 0; boot < 25; ++boot) {
        Device d(f, tr, quick(), 0u);
        CHECK(d.mgr.status().state == EmergencyState::Idle || d.mgr.status().state == EmergencyState::Cancelled);
        // one incident per boot, sometimes reset before the record is even committed
        d.storageRuns = (boot % 3) != 0;
        d.mgr.handleCommand(manual(static_cast<uint16_t>(boot + 1), 0u), 0u);
        d.run(0u, 1400u);
        if (!d.storageRuns) {
            CHECK_EQ(alerts(), sends);   // never transmitted with an undurable id within the hold
            continue;                    // reset: the queued record is lost with RAM
        }
        CHECK_EQ(alerts(), sends + 1u);
        const uint32_t id = lastAlertId();
        CHECK(ids.insert(id).second);          // never seen before
        ++sends;
        d.mgr.handleCommand(makeEmergencyCommand(EmergencyCommandType::CancelSos, CommandSource::Test,
                                                 static_cast<uint16_t>(100 + boot), id, 1500u), 1500u);
        d.run(1500u, 1600u);
    }
    std::printf("    [persist] 25 boots: %u SOS sent, %zu distinct incident ids\n", static_cast<unsigned>(sends), ids.size());
    CHECK_EQ(ids.size(), static_cast<size_t>(sends));
}

TEST(persist_torn_record_keeps_ids_monotonic_via_uncommitted_body)
{
    FakeNorFlash f;
    mock::SosTransport tr;
    {
        Device d(f, tr, quick(), 0u);
        d.mgr.handleCommand(manual(1, 0u), 0u);
        d.run(0u, 1100u);   // incident 1 sent
        d.mgr.handleCommand(makeEmergencyCommand(EmergencyCommandType::CancelSos, CommandSource::Test, 2, 1u, 1200u), 1200u);
        d.run(1200u, 1300u);
        d.mgr.handleReport(rep(TransportReportKind::CancelAccepted, 1u, d.mgr.status().requestRevision, 1300u), 1300u);
        d.run(1300u, 12000u);   // re-armed (Idle)
        CHECK(d.mgr.status().state == EmergencyState::Idle);
        d.storageRuns = false;
        d.mgr.handleCommand(manual(3, 12100u), 12100u);   // incident 2 opened, record queued
        f.cutAfterBytes = 61;   // body programmed, commit word torn
        d.storageRuns = true;
        d.settle(12120u);
        CHECK(f.powerLost);
    }
    f.reboot();
    Device d(f, tr, quick(), 0u);
    CHECK(d.mgr.status().state == EmergencyState::Idle);   // the torn state is not trusted ...
    CHECK_EQ(d.mgr.status().nextIncidentId, 3u);           // ... but its id is never reused
}

TEST(persist_corrupt_journal_marks_identity_uncertain_in_requests)
{
    FakeNorFlash f;
    for (uint32_t i = 0; i < 2u * 4096u; i += 97u) f.mem[partition::kEmergencyJournal.base + i] = 0x5Au;
    mock::SosTransport tr;
    Device d(f, tr, quick(), 0u);
    CHECK(d.store.mountState() == StoreMount::Corrupt);
    CHECK((d.mgr.status().flags & emflag::IdentityUncertain) != 0u);
    d.mgr.handleCommand(manual(1, 0u), 0u);
    d.run(0u, 1200u);
    CHECK_EQ(tr.sent.size(), 1u);
    CHECK((tr.sent[0].req.validity & sosvalid::IncidentIdVolatile) != 0u);   // the phone must not de-duplicate on it
    // the condition is sticky across resets (persisted), until the device is re-provisioned
    Device e(f, tr, quick(), 0u);
    CHECK((e.mgr.status().flags & emflag::IdentityUncertain) != 0u);
}

TEST(persist_identity_counter_exhaustion_is_reported_not_silent)
{
    mock::SosTransport tr;
    mock::EmergencyStore s;
    EmergencyPersistRecord r{};
    r.nextIncidentId = UINT32_MAX - 1u;
    r.state = EmergencyState::Idle;
    CHECK_EQ(encodeEmergencyRecord(r, s.bytes, sizeof(s.bytes)), kEmergencyRecordBytes);
    s.has = true;
    EmergencyManager m(quick(), tr, &s);
    m.restore(0u);
    m.handleCommand(manual(1, 0u), 0u);
    CHECK_EQ(m.status().incidentId, UINT32_MAX - 1u);
    CHECK((m.status().flags & emflag::IdentityExhausted) != 0u);
    CHECK_EQ(m.status().nextIncidentId, 1u);
    for (uint32_t t = 20; t <= 1200u; t += 20u) m.tick(t);
    CHECK((tr.sent.at(0).req.validity & sosvalid::IncidentIdVolatile) != 0u);
    EmergencyManager m2(quick(), tr, &s);   // persisted across resets
    m2.restore(0u);
    CHECK((m2.status().flags & emflag::IdentityExhausted) != 0u);
}

TEST(persist_flash_unavailable_sos_still_sent_immediately_and_flagged)
{
    FakeNorFlash f;
    f.present = false;
    mock::SosTransport tr;
    Device d(f, tr, quick(), 0u);
    CHECK((d.mgr.status().flags & emflag::PersistenceUnavailable) != 0u);
    CHECK(d.mgr.status().persist == PersistState::Disabled);
    d.mgr.handleCommand(manual(1, 0u), 0u);
    d.run(0u, 1100u);
    CHECK_EQ(tr.sent.size(), 1u);   // no identity hold without a store: never delay help for it
    CHECK((tr.sent[0].req.validity & sosvalid::IncidentIdVolatile) != 0u);
}

TEST(persist_battery_reported_in_sos_only_when_valid_and_recent)
{
    mock::SosTransport tr;
    EmergencyManager m(quick(), tr, nullptr);
    m.restore(0u);
    m.observeBattery(3650u, true, 0u);
    m.handleCommand(manual(1, 0u), 0u);
    for (uint32_t t = 20; t <= 1100u; t += 20u) m.tick(t);
    CHECK((tr.sent.at(0).req.validity & sosvalid::Battery) != 0u);
    CHECK_EQ(tr.sent[0].req.batteryMillivolts, 3650u);
    EmergencyManager n(quick(), tr, nullptr);
    n.restore(0u);
    n.observeBattery(0u, false, 0u);   // unknown: never invented
    n.handleCommand(manual(1, 0u), 0u);
    for (uint32_t t = 20; t <= 1100u; t += 20u) n.tick(t);
    CHECK_EQ(tr.sent.back().req.validity & sosvalid::Battery, 0u);
    CHECK_EQ(tr.sent.back().req.batteryMillivolts, 0u);
}

TEST(persist_uint32_clock_wrap_during_countdown_and_commit)
{
    FakeNorFlash f;
    mock::SosTransport tr;
    const uint32_t t0 = 0xFFFFFF00u;
    Device d(f, tr, quick(), t0);
    d.mgr.handleCommand(manual(1, t0), t0);
    d.run(t0, t0 + 1300u);   // wraps through 0
    CHECK_EQ(tr.sent.size(), 1u);
    CHECK(d.mgr.status().persist == PersistState::Committed);
    CHECK(d.mgr.status().persistLatencyMaxMs < 100u);
}
