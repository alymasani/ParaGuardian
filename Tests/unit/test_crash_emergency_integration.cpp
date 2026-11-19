/**
 ******************************************************************************
 * @file    test_crash_emergency_integration.cpp
 * @brief   End-to-end: simulated flight -> SafetyMonitor -> CrashDetector ->
 *          EmergencyManager -> SafetyEventDispatcher -> audio / LED mocks,
 *          with a mocked transport and a mocked phone app (scenarios 61-70);
 *          the REAL preprocessing path for impact peaks; threshold
 *          sensitivity sweeps; frozen held-out scenarios; documented
 *          ambiguities; allocation and host-timing checks (scenario 60).
 *
 * The phone mock follows the stage rules (accept -> send attempt ->
 * confirmation) with configurable delays. It demonstrates the firmware's
 * bookkeeping, not that any phone or contact was reached.
 ******************************************************************************
 */
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <vector>

#include "TestFramework.hpp"
#include "mocks/MockEmergencyIo.hpp"
#include "sim/CrashSimulator.hpp"
#include "sim/SensorSimulator.hpp"

#include "Algorithms/Crash/CrashInputAdapter.hpp"
#include "Algorithms/Emergency/EmergencyManager.hpp"
#include "Algorithms/Safety/SafetyEventDispatcher.hpp"
#include "Algorithms/Safety/SafetyEventQueue.hpp"
#include "Processing/PreprocessingPipeline.hpp"

// ---- allocation counter (scenario 60): counts every global operator new ----
namespace {
std::atomic<uint64_t> g_allocs{0};
}
void* operator new(std::size_t n)
{
    g_allocs.fetch_add(1u, std::memory_order_relaxed);
    if (void* p = std::malloc(n == 0 ? 1 : n)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

using namespace pg;
using csim::Profile;
using csim::Scenario;

namespace {

/// Mock phone app: answers the firmware's requests with the configured delays.
struct Phone {
    bool online = true;
    int acceptDelayMs = 800, sendDelayMs = 1500, confirmDelayMs = 8000;   ///< < 0 = never
    bool failDelivery = false;
    size_t handled = 0;
    struct Pending {
        uint32_t at;
        TransportReport r;
    };
    std::vector<Pending> q;
    void poll(const mock::SosTransport& tr, uint32_t now, EmergencyManager& m)
    {
        for (; handled < tr.sent.size(); ++handled) {
            const SosRequest& r = tr.sent[handled].req;
            if (!online) continue;
            const uint32_t t0 = tr.sent[handled].ms;
            if (r.kind == SosRequestKind::Cancel) {
                q.push_back({t0 + 300u, TransportReport{TransportReportKind::CancelAccepted, r.revision, r.incidentId, 0u}});
                continue;
            }
            q.push_back({t0 + 50u, TransportReport{TransportReportKind::WriteCompleted, r.revision, r.incidentId, 0u}});
            if (acceptDelayMs >= 0)
                q.push_back({t0 + static_cast<uint32_t>(acceptDelayMs),
                             TransportReport{TransportReportKind::PhoneAccepted, r.revision, r.incidentId, 0u}});
            if (sendDelayMs >= 0)
                q.push_back({t0 + static_cast<uint32_t>(sendDelayMs),
                             TransportReport{TransportReportKind::PhoneSendAttempted, r.revision, r.incidentId, 0u}});
            if (confirmDelayMs >= 0)
                q.push_back({t0 + static_cast<uint32_t>(confirmDelayMs),
                             TransportReport{failDelivery ? TransportReportKind::DeliveryFailed
                                                          : TransportReportKind::DeliveryConfirmed,
                                             r.revision, r.incidentId, 0u}});
        }
        for (size_t i = 0; i < q.size();) {
            if (timebase::diffMs(now, q[i].at) >= 0) {
                TransportReport r = q[i].r;
                r.receivedMs = now;
                m.handleReport(r, now);
                q.erase(q.begin() + static_cast<long>(i));
            } else {
                ++i;
            }
        }
    }
};

/// The whole chain around one csim::Runner (the emergency side runs every 20 ms, like the task).
struct Chain {
    mock::NoLock lock;
    mock::SosTransport tr;
    mock::EmergencyStore store;
    mock::AudioAlert audio;
    mock::StatusLed led;
    SafetyEventQueue<16> procEvents{lock};
    SafetyEventQueue<8> emEvents{lock};
    EmergencyManager mgr;
    SafetyEventDispatcher disp{DispatcherConfig{}, &audio, &led};
    Phone phone;
    IncidentContext ctx{};
    std::vector<std::pair<uint32_t, EmergencyCommand>> commands;   ///< (time ms since sim start, command)
    std::function<void(uint32_t)> atTick;
    // results (sim seconds)
    double incidentT = -1, requestT = -1, firstSubmitT = -1, acceptedT = -1, deliveredT = -1, cancelledT = -1;
    uint32_t maxProcBacklog = 0;

    explicit Chain(const EmergencyConfig& ec = EmergencyConfig{}) : mgr(ec, tr, &store)
    {
        mgr.addSink(&emEvents);
        mgr.restore(0u);
    }

    void attach(csim::Runner& r, uint32_t t0Ms)
    {
        r.flight().monitor().addSink(&procEvents);
        r.each = [this, t0Ms](double t, const CrashInput&, const CrashStatus& st, const SafetySnapshot& snap,
                              const fsim::Truth& tr0, const SafetyInput& in) {
            (void)tr0;
            // incident context as SensorProcessingTask keeps it (synthetic coordinates from the sim)
            if (in.gnss.valid) {
                ctx.gnssFixValid = true;
                ctx.latitude_1e7deg = 461000000 + static_cast<int32_t>(tr0.x * 90.0);
                ctx.longitude_1e7deg = -1141000000 + static_cast<int32_t>(tr0.y * 130.0);
                ctx.heightMsl_m = static_cast<float>(1000.0 + tr0.h);
                ctx.horizontalAccuracy_m = 2.5f;
                ctx.fixType = 3u;
                ctx.numSatellites = 12u;
                ctx.fixTimestampMs = in.gnss.timestampMs;
            }
            if (in.nav.altitudeValid) {
                ctx.navAltitudeValid = true;
                ctx.navAltitude_m = in.nav.altitude_m;
                ctx.navTimestampMs = in.nav.timestampMs;
            }
            if (in.nav.velocityValid) {
                ctx.navVelocityValid = true;
                ctx.navVerticalVelocity_mps = in.nav.verticalVelocity_mps;
            }
            const uint32_t now = in.nowMs;
            const uint32_t simMs = now - t0Ms;
            if (simMs % 20u != 1u) return;   // emergency task period
            const uint32_t backlog = static_cast<uint32_t>(procEvents.size());
            if (backlog > maxProcBacklog) maxProcBacklog = backlog;
            for (auto& c : commands) {
                if (c.first != 0u && simMs >= c.first) {
                    c.second.issuedMs = now;
                    c.second.check = emergencyCommandCheck(c.second);
                    if (mgr.handleCommand(c.second, now) == CommandResult::ForwardToAlerts) disp.acknowledge(now);
                    c.first = 0u;
                }
            }
            phone.poll(tr, now, mgr);
            CrashReport rep{st, ctx, now};
            mgr.observeCrash(rep, now);
            const EmergencyStatus& es = mgr.tick(now);
            if (es.incidents > 0u && incidentT < 0) incidentT = t;
            if (es.requestIssued && requestT < 0) requestT = t;
            if (!tr.sent.empty() && firstSubmitT < 0) firstSubmitT = t;
            if ((es.reportedStages & stagebit::AcceptedByPhone) != 0u && acceptedT < 0) acceptedT = t;
            if (es.state == EmergencyState::Delivered && deliveredT < 0) deliveredT = t;
            if (es.state == EmergencyState::Cancelled && cancelledT < 0) cancelledT = t;
            SafetyEvent e{};
            while (procEvents.pop(e)) disp.consume(e, now);
            while (emEvents.pop(e)) disp.consume(e, now);
            disp.update(now, DispatcherInputs{&snap, &st, &es});
            if (atTick) atTick(simMs);
        };
    }
};

Scenario crash11()
{
    Profile p;
    p.crashT = 40.0;
    p.crashVz = -9.0;
    p.postTiltDeg = 80.0;
    Scenario s = csim::baseFlight("crash", p, 160);
    csim::impactAtContact(s, 9.0, 40, 600);
    return s;
}

}  // namespace

// ============================== end-to-end ==================================

TEST(e2e_61_62_normal_flights_never_reach_the_emergency_manager)
{
    Profile p;
    p.crashT = 70.0;
    p.crashVz = -1.2;   // a normal approach and landing at 70 s + ...
    p.postAccel = 2.0;
    p.postRate = 0.4;
    p.postWalkSpeed = 1.2;
    p.postRiseS = 2.0;
    Scenario land = csim::baseFlight("61 flight + safe landing", p, 160);
    csim::impactAtContact(land, 0.9, 80, 60);
    Profile q;
    q.flightAccel = 1.2;
    Scenario turb = csim::baseFlight("62 turbulent flight", q, 150);
    sim::Rng rng(62);
    for (int k = 0; k < 40; ++k)
        turb.pulses.push_back(csim::Pulse{40.0 + 2.5 * k, false, 0.8 + 1.4 * rng.uniform(), 80, 180 * rng.uniform(), 150});
    for (Scenario* s : {&land, &turb}) {
        Chain c;
        csim::Runner r(*s, 61);
        c.attach(r, s->fs.t0Ms);
        r.run();
        std::printf("    [e2e] %-28s incidents %u requests %u audio alerts %zu last LED %d\n", s->name,
                    c.mgr.status().incidents, c.mgr.status().sosRequestsCreated, c.audio.requests.size(),
                    int(c.led.pattern));
        CHECK_EQ(c.mgr.status().incidents, 0u);
        CHECK(c.tr.sent.empty());
        CHECK(c.mgr.status().state == EmergencyState::Idle);
    }
}

TEST(e2e_63_suspected_crash_cancelled_by_the_pilot)
{
    Chain c;
    Scenario s = crash11();
    csim::Runner r(s, 63);
    c.attach(r, s.fs.t0Ms);
    // the pilot is fine and cancels 20 s into the countdown (incident 1)
    c.commands.push_back({0u, makeEmergencyCommand(EmergencyCommandType::CancelSos, CommandSource::HardwareInput, 1, 1u, 0u)});
    c.atTick = [&](uint32_t simMs) {
        if (c.incidentT > 0 && c.commands[0].first == 0u && c.mgr.status().state == EmergencyState::Countdown &&
            simMs >= static_cast<uint32_t>((c.incidentT + 20.0) * 1000.0)) {
            c.commands[0].first = simMs + 20u;   // processed at the next emergency tick
        }
    };
    r.run();
    std::printf("    [e2e] 63 contact %.2f s, suspicion %.2f s, incident %.2f s, cancelled %.2f s, requests %u\n",
                r.contactT, r.firstSuspectT, c.incidentT, c.cancelledT, c.mgr.status().sosRequestsCreated);
    CHECK(c.cancelledT > 0);
    CHECK_EQ(c.mgr.status().sosRequestsCreated, 0u);
    CHECK(c.tr.sent.empty());
    CHECK(c.mgr.status().state == EmergencyState::Idle);   // re-armed after cancelRearmMs
    CHECK(c.audio.count(AlertTone::Countdown) >= 15u);       // the pilot was told
}

TEST(e2e_64_suspected_crash_countdown_sos_phone_ack_delivery)
{
    Chain c;
    Scenario s = crash11();
    csim::Runner r(s, 64);
    c.attach(r, s.fs.t0Ms);
    r.run();
    const EmergencyStatus& es = c.mgr.status();
    std::printf("    [e2e] 64 contact %.2f | suspected +%.2f | countdown +%.2f | request +%.2f | submitted +%.2f |"
                " phone accepted +%.2f | delivery reported +%.2f s\n",
                r.contactT, r.firstSuspectT - r.contactT, c.incidentT - r.contactT, c.requestT - r.contactT,
                c.firstSubmitT - r.contactT, c.acceptedT - r.contactT, c.deliveredT - r.contactT);
    CHECK(es.state == EmergencyState::Delivered);
    CHECK(es.trigger == IncidentTrigger::AutomaticCrashSuspected);
    CHECK(c.requestT - c.incidentT >= 59.9 && c.requestT - c.incidentT <= 60.1);
    CHECK_EQ(c.tr.sent.size(), 1u);
    const SosRequest& q = c.tr.sent[0].req;
    CHECK((q.validity & sosvalid::Position) != 0u);
    CHECK((q.validity & sosvalid::Impact) != 0u);
    CHECK(q.peakAccel_cg >= 600u);
    CHECK(c.deliveredT > c.acceptedT && c.acceptedT > c.firstSubmitT);
    // alerts: crash alarm / countdown beeps / SOS confirmation tone were requested
    CHECK(c.audio.count(AlertTone::Countdown) >= 50u);
    CHECK(c.audio.count(AlertTone::Emergency) >= 1u);
}

TEST(e2e_65_manual_sos_during_ordinary_flight)
{
    Chain c;
    Profile p;
    Scenario s = csim::baseFlight("65 manual SOS in flight", p, 100);
    csim::Runner r(s, 65);
    c.attach(r, s.fs.t0Ms);
    c.commands.push_back({60000u, makeEmergencyCommand(EmergencyCommandType::ManualSos, CommandSource::PhoneApp, 1, 0u, 0u)});
    r.run();
    std::printf("    [e2e] 65 manual SOS at 60.0 s: request %.2f s, submitted %.2f s, delivered %.2f s\n", c.requestT,
                c.firstSubmitT, c.deliveredT);
    CHECK(c.mgr.status().trigger == IncidentTrigger::Manual);
    CHECK(c.requestT >= 65.0 && c.requestT <= 65.1);
    CHECK_EQ(r.suspicions, 0u);   // no crash evidence was needed or invented
    CHECK((c.tr.sent[0].req.validity & sosvalid::Impact) == 0u);
}

TEST(e2e_66_67_68_emergency_with_gnss_lidar_or_navigation_lost)
{
    Scenario g = crash11();
    g.fs.gnss = false;
    Scenario l = crash11();
    l.fs.lidar = false;
    Scenario n = crash11();
    n.fs.navState = [](double t) { return t > 30.0 ? NavState::Unavailable : NavState::Tracking; };
    const char* names[] = {"66 GNSS lost", "67 LiDAR lost", "68 navigation failed"};
    Scenario* ss[] = {&g, &l, &n};
    for (int i = 0; i < 3; ++i) {
        Chain c;
        csim::Runner r(*ss[i], 66 + i);
        c.attach(r, ss[i]->fs.t0Ms);
        r.run();
        const SosRequest& q = c.tr.sent.empty() ? SosRequest{} : c.tr.sent[0].req;
        std::printf("    [e2e] %-22s state %d, request validity 0x%03x (position %d, rel. altitude %d)\n", names[i],
                    int(c.mgr.status().state), q.validity, int((q.validity & sosvalid::Position) != 0u),
                    int((q.validity & sosvalid::RelativeAltitude) != 0u));
        CHECK(c.mgr.status().state == EmergencyState::Delivered);
        if (i == 0) {
            CHECK((q.validity & sosvalid::Position) == 0u);   // never invented
            CHECK_EQ(q.fixAgeMs, UINT32_MAX);
        }
        if (i == 2) CHECK((q.validity & sosvalid::RelativeAltitude) == 0u);   // stale navigation not sent
    }
}

TEST(e2e_69_transport_failure_and_recovery)
{
    Chain c;
    c.tr.link = false;
    Scenario s = crash11();
    s.fs.durationS = 260;
    csim::Runner r(s, 69);
    c.attach(r, s.fs.t0Ms);
    bool sawCommFailureAlert = false;
    c.atTick = [&](uint32_t simMs) {
        if (c.requestT > 0 && simMs > static_cast<uint32_t>((c.requestT + 90.0) * 1000.0) && !c.tr.link) {
            c.tr.link = true;   // the phone comes into range 90 s after the request
            c.mgr.handleReport(TransportReport{TransportReportKind::LinkUp, 0u, 0u, simMs}, s.fs.t0Ms + simMs);
        }
        if (c.disp.status().audioAlert == AlertKind::SosCommFailure ||
            (c.disp.status().activeMask & (1u << static_cast<uint8_t>(AlertKind::SosCommFailure))) != 0u) {
            sawCommFailureAlert = true;
        }
    };
    r.run();
    std::printf("    [e2e] 69 request %.2f s, link back +90 s, submitted %.2f s, delivered %.2f s, attempts %u\n",
                c.requestT, c.firstSubmitT, c.deliveredT, c.mgr.status().transportAttempts);
    CHECK(sawCommFailureAlert);   // the pilot is told help is NOT on its way yet
    CHECK(c.firstSubmitT >= c.requestT + 90.0);
    CHECK(c.mgr.status().state == EmergencyState::Delivered);
    CHECK_EQ(c.mgr.status().transportAttempts, 1u);   // waiting for a link consumed no attempts
}

TEST(e2e_70_extended_repeated_flights_and_incidents)
{
    // One manager + dispatcher for ten consecutive scenarios (time keeps running):
    // crash -> delivered -> reset, normal flight, crash -> cancelled, manual SOS ... x2
    Chain c;
    uint32_t t0 = 100000u;
    uint32_t expectedIncidents = 0;
    for (int round = 0; round < 10; ++round) {
        const int kind = round % 4;   // 0 crash, 1 normal, 2 crash+cancel, 3 manual
        Scenario s = kind == 1 ? csim::baseFlight("normal", Profile{}, 90) : crash11();
        if (kind == 3) s = csim::baseFlight("manual", Profile{}, 90);
        s.fs.t0Ms = t0;
        if (kind != 1) ++expectedIncidents;
        csim::Runner r(s, 700 + round);
        c.commands.clear();
        c.incidentT = c.requestT = c.deliveredT = c.cancelledT = c.firstSubmitT = c.acceptedT = -1;
        const uint32_t incidentsBefore = c.mgr.status().incidents;
        if (kind == 3)
            c.commands.push_back({50000u, makeEmergencyCommand(EmergencyCommandType::ManualSos, CommandSource::Diagnostic,
                                                               static_cast<uint16_t>(round * 10 + 1), 0u, 0u)});
        c.attach(r, t0);
        c.atTick = [&](uint32_t simMs) {
            if (kind == 2 && c.mgr.status().incidents > incidentsBefore && c.mgr.status().state == EmergencyState::Countdown) {
                const EmergencyCommand x = makeEmergencyCommand(EmergencyCommandType::CancelSos, CommandSource::HardwareInput,
                                                                static_cast<uint16_t>(round * 10 + 2),
                                                                c.mgr.status().incidentId, t0 + simMs);
                c.mgr.handleCommand(x, t0 + simMs);
            }
        };
        r.run();
        const EmergencyStatus& es = c.mgr.status();
        if (es.state == EmergencyState::Delivered) {   // the operator closes the incident
            const EmergencyCommand x = makeEmergencyCommand(EmergencyCommandType::ResetIncident, CommandSource::PhoneApp,
                                                            static_cast<uint16_t>(round * 10 + 3), es.incidentId,
                                                            t0 + 160000u);
            CHECK(c.mgr.handleCommand(x, t0 + 160000u) == CommandResult::Accepted);
        }
        CHECK(c.mgr.status().state == EmergencyState::Idle || c.mgr.status().state == EmergencyState::Cancelled);
        CHECK_EQ(c.mgr.status().incidents - incidentsBefore, kind == 1 ? 0u : 1u);
        t0 += 200000u;   // the crash detector's cooldown has long expired between rounds
    }
    const EmergencyStatus& es = c.mgr.status();
    std::printf("    [e2e] 70 ten rounds: incidents %u (expected %u), requests %u, cancellations %u, delivered %u,"
                " max processing backlog %u\n",
                es.incidents, expectedIncidents, es.sosRequestsCreated, es.cancellations, es.deliveriesConfirmed,
                c.maxProcBacklog);
    CHECK_EQ(es.incidents, expectedIncidents);
    CHECK_EQ(es.cancellations, 2u);
    CHECK_EQ(es.deliveriesConfirmed, expectedIncidents - 2u);
    CHECK_EQ(es.wrongIncidentReports, 0u);
    CHECK_EQ(c.disp.status().eventGaps, 0u);
}

// ====================== real preprocessing path (impact peaks) =================

TEST(real_pipeline_carries_impact_peaks_and_saturation_before_any_smoothing)
{
    sim::SensorSimulator s(91);
    PreprocessingPipeline p;
    p.start(0);
    ProcessedSensorFrame f{};
    uint32_t t = 0;
    for (; t <= 5000u; t += 10u) {
        for (const SensorMessage& m : s.step(t)) p.ingest(m, t + 1u);
        p.buildFrame(t + 1u, f);
    }
    // ordinary frame: one sample per IMU, ~1 g
    CHECK_EQ(f.imu[0].samplesSinceFrame, 1u);
    CHECK(std::fabs(f.imu[0].peakAccelNorm_mps2 - 9.81f) < 0.5f);
    CHECK(!f.imu[0].accelSaturatedSinceFrame);
    // a delayed batch: 3 cycles ingested before one frame, the impact in the MIDDLE one
    s.specificForce = [](uint32_t ms) {
        return Vec3f{0.0f, 0.0f, ms == 5020u ? -120.0f : -9.80665f};
    };
    for (uint32_t k = 0; k < 3u; ++k, t += 10u)
        for (const SensorMessage& m : s.step(t)) p.ingest(m, t + 1u);
    p.buildFrame(t, f);
    CHECK(f.imu[0].samplesSinceFrame >= 3u);
    CHECK(f.imu[0].peakAccelNorm_mps2 > 100.0f);   // the newest sample alone would show 1 g
    CHECK(f.imu[0].peakTimestampMs == 5020u);
    // saturation: flagged, value kept as a lower bound (the voter excludes the channel)
    for (auto& m : s.imu) m.accelSaturated = true;
    s.specificForce = [](uint32_t) { return Vec3f{0.0f, 0.0f, -156.0f}; };
    for (const SensorMessage& m : s.step(t)) p.ingest(m, t + 1u);
    p.buildFrame(t + 1u, f);
    CHECK(f.imu[0].accelSaturatedSinceFrame && f.imu[1].accelSaturatedSinceFrame);
    CHECK(f.imu[0].peakAccelNorm_mps2 > 150.0f);
    CHECK(!f.specificForce_mps2.valid);   // nothing voted, but the crash input still sees the peaks
    const CrashInput ci = crashInputFrom(f, AttitudeEstimate{}, SafetySnapshot{}, NavigationEstimate{});
    CHECK(ci.imu[0].accelSaturated && ci.imu[0].peakAccel_mps2 > 150.0f);
    CHECK(!ci.forceValid);
    CHECK(!ci.attitudeValid);
    // the next frame resets the "since frame" peaks
    for (auto& m : s.imu) m.accelSaturated = false;
    s.specificForce = [](uint32_t) { return Vec3f{0.0f, 0.0f, -9.80665f}; };
    t += 10u;
    for (const SensorMessage& m : s.step(t)) p.ingest(m, t + 1u);
    p.buildFrame(t + 1u, f);
    CHECK(!f.imu[0].accelSaturatedSinceFrame);
    CHECK(f.imu[0].peakAccelNorm_mps2 < 12.0f);
}

TEST(incident_context_keeps_last_valid_fix_with_its_time)
{
    IncidentContext ctx{};
    ProcessedSensorFrame f{};
    NavigationEstimate n{};
    f.gnss.valid = true;
    f.gnss.fixType = GnssFixType::Fix3D;
    f.gnss.latitude_1e7deg = 1;
    f.gnss.longitude_1e7deg = 2;
    f.gnss.timestampMs = 1000u;
    updateIncidentContext(ctx, f, n);
    CHECK(ctx.gnssFixValid && ctx.fixTimestampMs == 1000u);
    f.gnss.valid = false;   // fix lost: the last valid one stays, with its old time
    f.gnss.latitude_1e7deg = 0;
    updateIncidentContext(ctx, f, n);
    CHECK(ctx.latitude_1e7deg == 1 && ctx.fixTimestampMs == 1000u);
    f.gnss.valid = true;
    f.gnss.fixType = GnssFixType::TimeOnly;   // not a position fix
    f.gnss.timestampMs = 2000u;
    updateIncidentContext(ctx, f, n);
    CHECK(ctx.fixTimestampMs == 1000u);
}

// ============================ sweeps and held-out =============================

namespace {

struct SetResult {
    int detected = 0, incidents = 0, falseAlarms = 0, normals = 0;
    double meanLatency = 0;
};

std::vector<Scenario> tuningIncidents()
{
    std::vector<Scenario> v;
    auto mk = [&](const char* n, double vz, double g, double dur, double dps, double tilt, bool still) {
        Profile p;
        p.crashT = 40.0;
        p.crashVz = vz;
        p.postTiltDeg = tilt;
        if (!still) {
            p.postAccel = 4.0;
            p.postRate = 0.6;
            p.postRiseS = 0.5;
        }
        Scenario s = csim::baseFlight(n, p, 75);
        csim::impactAtContact(s, g, dur, dps);
        s.expectSuspicion = 1;
        v.push_back(s);
    };
    mk("T11", -9.0, 9.0, 40, 600, 80, true);
    mk("T12", -3.0, 5.0, 60, 900, 20, true);
    mk("T13", -5.0, 3.5, 60, 150, 70, true);
    mk("T14", -4.5, 6.0, 50, 200, 0, true);
    mk("T15", -2.0, 9.0, 40, 300, 0, false);
    return v;
}

std::vector<Scenario> tuningNormals()
{
    std::vector<Scenario> v;
    Profile g;
    v.push_back(csim::baseFlight("N glide", g, 120));
    auto landing = [&](const char* n, double vz, double extra, double dps) {
        Profile p;
        p.crashT = 45.0;
        p.crashVz = vz;
        p.postAccel = 2.0;
        p.postRate = 0.4;
        p.postWalkSpeed = 1.2;
        p.postRiseS = 1.5;
        Scenario s = csim::baseFlight(n, p, 100);
        csim::impactAtContact(s, extra, 60, dps);
        v.push_back(s);
    };
    landing("N landing", -1.2, 0.8, 60);
    landing("N hard landing", -2.5, 3.5, 150);
    Profile t;
    t.flightAccel = 1.2;
    Scenario turb = csim::baseFlight("N turbulence", t, 120);
    sim::Rng r(33);
    for (int k = 0; k < 30; ++k)
        turb.pulses.push_back(csim::Pulse{40.0 + 2.5 * k, false, 0.8 + 1.6 * r.uniform(), 70, 220 * r.uniform(), 150});
    v.push_back(turb);
    return v;
}

SetResult evaluate(const std::vector<Scenario>& inc, const std::vector<Scenario>& nor, const CrashConfig& cfg,
                   uint64_t seed)
{
    SetResult r;
    double lat = 0;
    for (const Scenario& s : inc) {
        csim::Runner x(s, seed, cfg);
        x.run();
        ++r.incidents;
        if (x.suspicions > 0u) {
            ++r.detected;
            lat += x.firstSuspectT - x.contactT;
        }
    }
    for (const Scenario& s : nor) {
        csim::Runner x(s, seed, cfg);
        x.run();
        ++r.normals;
        if (x.suspicions > 0u) ++r.falseAlarms;
    }
    r.meanLatency = r.detected ? lat / r.detected : -1;
    return r;
}

}  // namespace

TEST(sweep_threshold_sensitivity_on_the_tuning_set)
{
    const auto inc = tuningIncidents();
    const auto nor = tuningNormals();
    struct Row {
        const char* name;
        CrashConfig c;
    };
    std::vector<Row> rows;
    auto add = [&](const char* n, auto fn) {
        CrashConfig c;
        fn(c);
        rows.push_back({n, c});
    };
    add("default (4 g, 400 dps, inact 4 s, obs 10 s)", [](CrashConfig&) {});
    add("impactAccel 3.0 g", [](CrashConfig& c) { c.impactAccel_mps2 = 3.0f * CrashConfig::kG; });
    add("impactAccel 5.0 g", [](CrashConfig& c) { c.impactAccel_mps2 = 5.0f * CrashConfig::kG; });
    add("impactAccel 6.0 g", [](CrashConfig& c) { c.impactAccel_mps2 = 6.0f * CrashConfig::kG; });
    add("impactRate 300 dps", [](CrashConfig& c) { c.impactRate_radps = 300.0f * CrashConfig::kDeg; });
    add("impactRate 600 dps", [](CrashConfig& c) { c.impactRate_radps = 600.0f * CrashConfig::kDeg; });
    add("candidateWindow 200 ms", [](CrashConfig& c) { c.candidateWindowMs = 200u; });
    add("inactivity 2 s", [](CrashConfig& c) { c.inactivityMs = 2000u; });
    add("inactivity 8 s", [](CrashConfig& c) { c.inactivityMs = 8000u; });
    add("observation 6 s", [](CrashConfig& c) { c.observationMs = 6000u; c.inactivityMs = 4000u; });
    add("observation 14 s", [](CrashConfig& c) { c.observationMs = 14000u; });
    add("quietForce 0.4", [](CrashConfig& c) { c.quietForce_mps2 = 0.4f; });
    std::printf("    [sweep] %-44s %9s %13s %9s\n", "configuration", "detected", "false alarms", "latency");
    for (const Row& row : rows) {
        CHECK(row.c.isValid());
        const SetResult r = evaluate(inc, nor, row.c, 5u);
        std::printf("    [sweep] %-44s %4d / %-3d %6d / %-5d %7.2f s\n", row.name, r.detected, r.incidents,
                    r.falseAlarms, r.normals, r.meanLatency);
        if (&row == &rows[0]) {
            CHECK_EQ(r.detected, r.incidents);
            CHECK_EQ(r.falseAlarms, 0);
        }
    }
    // countdown duration and cooldown only shift the time to request / re-arm (EmergencyManager):
    for (uint32_t cd : {30000u, 60000u, 120000u}) {
        EmergencyConfig ec;
        ec.autoCountdownMs = cd;
        Chain c(ec);
        Scenario s = crash11();
        s.fs.durationS = 50.0 + cd * 1e-3 + 20.0;
        csim::Runner r(s, 9);
        c.attach(r, s.fs.t0Ms);
        r.run();
        std::printf("    [sweep] countdown %3u s: contact -> request %.1f s\n", cd / 1000u, c.requestT - r.contactT);
        CHECK(c.requestT > 0);
    }
}

TEST(heldout_frozen_parameters)
{
    // Defined BEFORE running with the frozen defaults; expected outcome stated per case.
    struct H {
        Scenario s;
        bool expect;
    };
    std::vector<H> hs;
    auto crashCase = [&](const char* n, double vz, double g, double dur, double dps, double tilt, double accel, bool expect) {
        Profile p;
        p.crashT = 50.0;
        p.crashVz = vz;
        p.postTiltDeg = tilt;
        p.postAccel = accel;
        p.postRate = accel > 1.0 ? 0.5 : 0.01;
        p.postWalkSpeed = accel > 1.0 ? 1.0 : 0.0;
        p.postRiseS = 1.0;
        Scenario s = csim::baseFlight(n, p, 90);
        csim::impactAtContact(s, g, dur, dps);
        hs.push_back({s, expect});
    };
    crashCase("H1 sink -7, 8 g + 450 dps, lying at 50 deg", -7.0, 7.0, 35, 450, 50, 0.05, true);
    crashCase("H2 hard landing -3.0, 5.5 g, walks away", -3.0, 4.5, 50, 250, 0, 2.0, false);
    crashCase("H6 sink -6, 12 g, unconscious upright", -6.0, 11.0, 30, 300, 10, 0.05, true);
    crashCase("H8 slow tree-like -2.0, 3 g, moving", -2.0, 2.0, 80, 150, 30, 2.0, false);
    {
        Profile p;
        p.crashT = 50.0;
        p.crashVz = -8.0;
        p.postTiltDeg = 85.0;
        Scenario s = csim::baseFlight("H7 crash, GNSS and LiDAR both missing", p, 90);
        s.fs.gnss = false;
        s.fs.lidar = false;
        csim::impactAtContact(s, 10.0, 40, 700);
        hs.push_back({s, true});
    }
    {
        Profile q;
        q.flightAccel = 1.5;
        Scenario s = csim::baseFlight("H3 strong turbulence 3.2 g, other seed", q, 140);
        sim::Rng r(4242);
        for (int k = 0; k < 40; ++k)
            s.pulses.push_back(csim::Pulse{40.0 + 2.3 * k, false, 1.0 + 1.2 * r.uniform(), 90, 250 * r.uniform(), 150});
        hs.push_back({s, false});
    }
    {   // landed, 20 s later the device is dropped (free fall 0.4 s, 9 g)
        Profile p;
        p.crashT = 45.0;
        p.crashVz = -1.2;
        Scenario s = csim::baseFlight("H5 dropped 20 s after landing", p, 110);
        std::shared_ptr<double> tc = s.tc;
        s.loadG = [tc](double t) { return (t >= *tc + 20.0 && t < *tc + 20.4) ? 0.02 : 1.0; };
        s.pulses.push_back(csim::Pulse{20.4, true, 8.0, 25, 600, 80});
        csim::impactAtContact(s, 0.8, 80, 60);
        hs.push_back({s, false});
    }
    {   // a sharp in-flight collapse: 4.3 g + 450 dps, flight continues
        Profile p;
        Scenario s = csim::baseFlight("H9 collapse in flight, recovers", p, 110);
        s.pulses.push_back(csim::Pulse{60.0, false, 3.3, 80, 450, 200});
        hs.push_back({s, false});
    }
    int agree = 0;
    for (auto& h : hs) {
        csim::Runner r(h.s, 2024);
        r.run();
        const bool got = r.suspicions > 0u;
        agree += got == h.expect ? 1 : 0;
        std::printf("    [heldout] %-46s expected %d got %d latency %.2f decision %d\n", h.s.name, int(h.expect),
                    int(got), got ? r.firstSuspectT - r.contactT : -1.0, int(r.firstDecision));
        CHECK(got == h.expect);
    }
    std::printf("    [heldout] %d / %zu as expected (frozen defaults)\n", agree, hs.size());
}

TEST(ambiguity_hard_landing_vs_crash_is_not_separable)
{
    // Documented limitation: a hard landing after which the pilot lies still on the back
    // is indistinguishable (to these sensors) from a crash with an unconscious pilot.
    Profile p;
    p.crashT = 45.0;
    p.crashVz = -2.5;
    p.postTiltDeg = 85.0;   // lying on the back, winded, not moving
    Scenario s = csim::baseFlight("hard landing, pilot lies still on the back", p, 80);
    csim::impactAtContact(s, 3.5, 60, 150);
    csim::Runner r(s, 77);
    r.run();
    Profile q;
    q.crashT = 45.0;
    q.crashVz = -2.5;
    q.postAccel = 2.0;
    q.postRate = 0.4;
    q.postWalkSpeed = 1.2;
    q.postRiseS = 1.5;
    Scenario v = csim::baseFlight("very hard landing (9 g), walks away", q, 80);
    csim::impactAtContact(v, 8.0, 40, 200);
    csim::Runner w(v, 78);
    w.run();
    std::printf("    [ambiguity] %-44s suspected %d (decision %d) -> the countdown asks the pilot\n", s.name,
                int(r.suspicions > 0u), int(r.firstDecision));
    std::printf("    [ambiguity] %-44s suspected %d (decision %d) -> the countdown asks the pilot\n", v.name,
                int(w.suspicions > 0u), int(w.firstDecision));
    CHECK(r.suspicions > 0u);   // a false alarm by design: cancellable
    CHECK(w.suspicions > 0u);
}

// ============================ resources (60) =================================

TEST(alloc_60_no_heap_in_steady_state_updates_and_host_timing)
{
    mock::NoLock lock;
    mock::SosTransport tr;
    mock::AudioAlert audio;
    mock::StatusLed led;
    audio.requests.reserve(200000);   // the MOCK records into vectors: give them room up front
    audio.cancels.reserve(200000);
    CrashDetector det;
    SafetyEventEmitter em;
    SafetyEventQueue<16> q(lock);
    em.addSink(&q);
    EmergencyManager mgr(EmergencyConfig{}, tr, nullptr);
    tr.sent.reserve(64);
    SafetyEventQueue<8> eq(lock);
    mgr.addSink(&eq);
    SafetyEventDispatcher disp(DispatcherConfig{}, &audio, &led);
    SafetySnapshot snap{};
    CrashReport rep{};
    CrashInput in{};
    for (auto& c : in.imu) {
        c.present = true;
        c.samples = 1u;
        c.peakAccel_mps2 = 9.8f;
        c.peakRate_radps = 0.05f;
    }
    in.forceValid = in.rateValid = in.newImuData = in.attitudeValid = true;
    in.forceNorm_mps2 = 9.8f;
    in.rateNorm_radps = 0.05f;
    in.phase = FlightPhase::Flying;
    const uint64_t before = g_allocs.load();
    using clk = std::chrono::steady_clock;
    double detNs = 0, mgrNs = 0, dispNs = 0;
    const int N = 30000;
    for (int i = 0; i < N; ++i) {
        const uint32_t now = 1000u + static_cast<uint32_t>(i) * 10u;
        in.nowMs = now;
        for (int k = 0; k < 3; ++k) {
            in.imu[k].peakTimestampMs = now - 1u;
            in.imu[k].peakAccel_mps2 = (i % 5000 == 2500) ? 100.0f : 9.8f;   // impacts now and then
        }
        auto a = clk::now();
        rep.status = det.update(in, em);
        auto b = clk::now();
        rep.publishedMs = now;
        mgr.observeCrash(rep, now);
        if (i == 15000) mgr.handleCommand(makeEmergencyCommand(EmergencyCommandType::ManualSos, CommandSource::Test, 1, 0u, now), now);
        const EmergencyStatus& es = mgr.tick(now);
        auto c2 = clk::now();
        SafetyEvent e{};
        while (q.pop(e)) disp.consume(e, now);
        while (eq.pop(e)) disp.consume(e, now);
        snap.timestampMs = now;
        disp.update(now, DispatcherInputs{&snap, &rep.status, &es});
        auto d = clk::now();
        detNs += std::chrono::duration<double, std::nano>(b - a).count();
        mgrNs += std::chrono::duration<double, std::nano>(c2 - b).count();
        dispNs += std::chrono::duration<double, std::nano>(d - c2).count();
    }
    const uint64_t allocs = g_allocs.load() - before;
    std::printf("    [resource] %d updates: %llu heap allocations; host time per update: detector %.0f ns,"
                " manager %.0f ns, dispatcher %.0f ns\n",
                N, static_cast<unsigned long long>(allocs), detNs / N, mgrNs / N, dispNs / N);
    std::printf("    [resource] sizeof: CrashDetector %zu, EmergencyManager %zu, SafetyEventDispatcher %zu,"
                " CrashReport %zu, EmergencyStatus %zu, SosRequest %zu (host ABI)\n",
                sizeof(CrashDetector), sizeof(EmergencyManager), sizeof(SafetyEventDispatcher), sizeof(CrashReport),
                sizeof(EmergencyStatus), sizeof(SosRequest));
    CHECK(det.latest().impactCandidates >= 2u);   // others fall into the suspected-hold / cooldown
    CHECK(mgr.status().incidents >= 1u);
    CHECK_EQ(allocs, 0u);
}
