/**
 * Milestone 5 - integration (D47-D56), fault tolerance (brief section 23) and
 * limited parameter sweeps (tuning seeds 1-3; the scenario tests use the
 * held-out seed 101+ with the frozen defaults).
 *
 * D55 runs the REAL firmware data path:
 *   SensorMessages -> PreprocessingPipeline -> ProcessedSensorFrame
 *   -> AttitudeEstimator -> VerticalNavigationEstimator
 *   -> safetyInputFrom(frame, attitude, navigation) -> SafetyMonitor
 *   -> SafetyEventQueue (bounded) + SafetySnapshot.
 */
#include <cmath>
#include <cstdio>

#include "TestFramework.hpp"
#include "sim/FlightSafetySimulator.hpp"
#include "sim/SensorSimulator.hpp"

#include "Algorithms/Attitude/AttitudeEstimator.hpp"
#include "Algorithms/Attitude/AttitudeFrameAdapter.hpp"
#include "Algorithms/Navigation/ConstantAccelerationModel.hpp"
#include "Algorithms/Navigation/NavigationInputAdapter.hpp"
#include "Algorithms/Navigation/VerticalNavigationEstimator.hpp"
#include "Algorithms/Safety/SafetyEventQueue.hpp"
#include "Processing/PreprocessingPipeline.hpp"

using namespace pg;
using fsim::ramp;
using fsim::Scenario;

namespace {

constexpr uint64_t kSeed = 101u;

double hill(double x, double) { return x < 5 ? 100.0 : std::fmax(0.0, 100.0 - 0.35 * (x - 5)); }

/// Complete synthetic flight: still on a hill launch, run, glide out, thermal (circling climb),
/// glide down into the valley, approach, flare, touchdown, stand still.
Scenario fullFlight(double dur = 420)
{
    Scenario s;
    s.durationS = dur;
    s.terrain = hill;
    s.h0 = 101.0;
    s.groundSpeed = [](double t) {
        if (t < 10) return 0.0;
        if (t < 13) return 4.0 * ramp(t, 10, 1);
        if (t < 322) return 4.0 + 5.0 * ramp(t, 13, 2);
        return 9.0 * (1 - ramp(t, 322, 3));
    };
    s.vz = [](double t) {
        if (t < 10) return 0.0;
        if (t < 60) return -1.2;
        if (t < 180) return 2.0 + 0.8 * std::sin(0.4 * t);      // thermal: +243 m (to ~284 m)
        if (t < 312) return -2.0;                                // glide down to the valley floor
        return -2.0 + 1.4 * ramp(t, 312, 4);                     // flare; touchdown ~ 321 s
    };
    s.heading = [](double t) { return (t > 60 && t < 180) ? 0.35 * (t - 60) : 0.0; };
    s.rollDeg = [](double t) { return (t > 62 && t < 178) ? 25.0 : 0.0; };
    s.motionAccel = [](double t) { return t < 10 ? 0.05 : (t < 14 ? 2.0 : (t < 326 ? 0.6 : 0.05)); };
    s.motionRate = [](double t) { return t < 10 ? 0.01 : (t < 326 ? 0.12 : 0.01); };
    return s;
}

struct Run {
    explicit Run(const Scenario& s, uint64_t seed = kSeed, const SafetyConfig& c = SafetyConfig{}) : r(s, seed, c) {}
    fsim::Runner r;
    void print(const char* id, const char* name)
    {
        std::printf("    [integ] %s %-46s phases:", id, name);
        for (const auto& p : r.phases) std::printf(" %.1f:%d", p.first, int(p.second));
        std::printf(" | events %zu, Caution+ %zu\n", r.rec.events.size(), r.rec.raisedAtLeast(SafetySeverity::Caution));
    }
};

bool sequenceContains(const std::vector<std::pair<double, FlightPhase>>& ph, std::initializer_list<FlightPhase> want)
{
    auto it = ph.begin();
    for (FlightPhase w : want) {
        while (it != ph.end() && it->second != w) ++it;
        if (it == ph.end()) return false;
        ++it;
    }
    return true;
}

}  // namespace

TEST(integ_47_48_full_synthetic_flight_takeoff_thermal_landing)
{
    Run x(fullFlight());
    x.r.run();
    x.print("D47/48", "launch, glide, thermal, glide, landing");
    CHECK(sequenceContains(x.r.phases, {FlightPhase::Ground, FlightPhase::TakeoffCandidate, FlightPhase::Flying,
                                        FlightPhase::LandingCandidate, FlightPhase::Landed}));
    CHECK_EQ(x.r.monitor().flightState().takeoffs, 1u);
    CHECK_EQ(x.r.monitor().flightState().landings, 1u);
    CHECK_EQ(x.r.rec.raisedAtLeast(SafetySeverity::Caution), 0u);   // a normal flight: advisories at most
    CHECK(x.r.last.phase == FlightPhase::Landed && x.r.last.activeWarningMask == 0u);
    CHECK(x.r.rec.events.size() < 20u);                               // transitions only, never per frame
    // No advisory while standing on the launch before the phase is decided (Unknown, clearance ~1 m).
    for (const auto& e : x.r.rec.events) {
        if (e.type == SafetyEventType::LowGroundClearance) CHECK(e.phase != FlightPhase::Unknown && e.phase != FlightPhase::Ground);
    }
    // every event carries the placeholder / unvalidated flags
    for (const auto& e : x.r.rec.events) CHECK((e.qualityFlags & safetyq::Unvalidated) != 0u);
}

TEST(integ_49_continuous_sensor_degradation)
{
    Scenario s = fullFlight();
    s.gnssPresent = [](double t) { return t < 60; };
    s.lidarWeak = [](double t) { return t > 90 && t < 300; };   // weak / saturated returns for a long time
    s.navState = [](double t) { return (t > 120 && t < 200) ? NavState::Degraded : NavState::Tracking; };
    s.attitudeState = [](double t) { return (t > 150 && t < 200) ? AttitudeState::Degraded : AttitudeState::Tracking6Axis; };
    Run x(s);
    x.r.run();
    x.print("D49", "GNSS lost, LiDAR weak, nav + attitude degraded");
    CHECK(x.r.rec.first(SafetyEventType::NavigationDegraded, SafetyTransition::Raised) != nullptr);
    CHECK(x.r.rec.count(SafetyEventType::NavigationDegraded, SafetyTransition::Raised) == 1u);   // no repeats
    CHECK(x.r.monitor().flightState().landings == 1u);   // still lands once the sensors are back
    CHECK(x.r.rec.raisedAtLeast(SafetySeverity::Warning) == 0u);
}

TEST(integ_50_lidar_loss_and_recovery_near_terrain)
{
    Scenario s;
    s.durationS = 120;
    s.h0 = 20;
    s.vz = [](double t) { return 0.2 * std::sin(0.2 * t); };   // soaring low
    s.groundSpeed = [](double) { return 4.0; };
    s.motionAccel = [](double) { return 0.5; };
    s.motionRate = [](double) { return 0.12; };
    s.lidarPresent = [](double t) { return !(t > 40 && t < 60); };
    Run x(s);
    x.r.run();
    x.print("D50", "LiDAR lost 40-60 s at 20 m");
    const SafetyEvent* raised = x.r.rec.first(SafetyEventType::TerrainClearanceUnavailable, SafetyTransition::Raised);
    const SafetyEvent* cleared = x.r.rec.first(SafetyEventType::TerrainClearanceUnavailable, SafetyTransition::Cleared);
    CHECK(raised && cleared);
    CHECK(raised && (raised->timestampMs - 200001u) * 1e-3 > 42.9 && (raised->timestampMs - 200001u) * 1e-3 < 44.0);
    CHECK(cleared && (cleared->timestampMs - 200001u) * 1e-3 > 60.0 && (cleared->timestampMs - 200001u) * 1e-3 < 63.0);
    CHECK(isAirbornePhase(x.r.last.phase));
}

TEST(integ_51_navigation_loss_and_recovery)
{
    Scenario s;
    s.durationS = 120;
    s.h0 = 600;
    s.vz = [](double) { return -1.2; };
    s.groundSpeed = [](double) { return 10.0; };
    s.motionAccel = [](double) { return 0.6; };
    s.motionRate = [](double) { return 0.12; };
    s.navState = [](double t) { return (t > 40 && t < 70) ? NavState::Unavailable : NavState::Tracking; };
    Run x(s);
    x.r.run();
    x.print("D51", "navigation unavailable 40-70 s");
    const SafetyEvent* n = x.r.rec.first(SafetyEventType::NavigationDegraded, SafetyTransition::Raised);
    CHECK(n && n->severity == SafetySeverity::Caution);
    CHECK(x.r.rec.first(SafetyEventType::NavigationDegraded, SafetyTransition::Cleared) != nullptr);
    CHECK(x.r.rec.count(SafetyEventType::ExcessiveDescent, SafetyTransition::Raised) == 0u);
    CHECK(x.r.firstPhase(FlightPhase::Landed) < 0);
}

TEST(integ_52_overlapping_warnings_and_prioritization)
{
    // Spiral dive (-11 m/s) towards rising terrain at low height: several warnings overlap.
    Scenario s;
    s.durationS = 12;
    s.h0 = 45;
    s.vz = [](double) { return -6.5; };
    s.groundSpeed = [](double) { return 12.0; };
    s.terrain = [](double x, double) { return x > 30 ? 0.3 * (x - 30) : 0.0; };
    s.motionAccel = [](double) { return 1.0; };
    s.motionRate = [](double) { return 0.3; };
    Run x(s);
    SafetySeverity worst = SafetySeverity::Info;
    uint8_t maxMask = 0;
    x.r.each = [&](double, const SafetyInput&, const SafetySnapshot& snap, const fsim::Truth&) {
        if (snap.highestSeverity > worst) worst = snap.highestSeverity;
        if (__builtin_popcount(snap.activeWarningMask) > __builtin_popcount(maxMask)) maxMask = snap.activeWarningMask;
    };
    x.r.run();
    x.print("D52", "spiral into rising terrain");
    std::printf("    [integ] D52 max simultaneous warnings mask 0x%02x, highest severity %d\n", maxMask, int(worst));
    CHECK(__builtin_popcount(maxMask) >= 3);
    CHECK(worst == SafetySeverity::Critical);
    CHECK(x.r.rec.count(SafetyEventType::ExcessiveDescent, SafetyTransition::Raised) == 1u);
    CHECK(x.r.rec.first(SafetyEventType::HazardousApproach, SafetyTransition::Raised) != nullptr);
}

TEST(integ_53_duplicate_frames_do_not_change_decisions)
{
    // Every safety input delivered twice (republished frame, nowMs + 1): same events as a clean run.
    Scenario s = fullFlight(360);
    fsim::Runner clean(s, kSeed);
    clean.run();
    SafetyMonitor mon;
    fsim::Recorder rec;
    mon.addSink(&rec);
    fsim::Runner feed(s, kSeed);
    feed.each = [&](double, const SafetyInput& in, const SafetySnapshot&, const fsim::Truth&) {
        mon.update(in);
        SafetyInput dup = in;
        dup.nowMs += 1u;
        mon.update(dup);
    };
    feed.run();
    std::printf("    [integ] D53 clean run %zu events, duplicated-input run %zu events; LiDAR duplicates skipped %u\n",
                clean.rec.events.size(), rec.events.size(), unsigned(mon.clearance().duplicates));
    CHECK_EQ(rec.events.size(), clean.rec.events.size());
    for (size_t i = 0; i < rec.events.size() && i < clean.rec.events.size(); ++i) {
        CHECK(rec.events[i].type == clean.rec.events[i].type && rec.events[i].transition == clean.rec.events[i].transition);
    }
    CHECK(mon.clearance().duplicates > 1000u);   // LiDAR valid only within 40 m of the terrain
    CHECK_EQ(mon.clearance().samples, clean.monitor().clearance().samples);
}

TEST(integ_54_monitor_with_bounded_queue_and_slow_consumer)
{
    struct NoLock final : ICriticalSection {
        void enter() override {}
        void exit() override {}
    } lock;
    SafetyEventQueue<4> q(lock);
    // Started at 600 m; spiral dive (-11 m/s, Critical) at 30-40 s, then normal glide.
    Scenario s;
    s.durationS = 120;
    s.h0 = 600;
    s.vz = [](double t) { return (t > 30 && t < 40) ? -11.0 * ramp(t, 30, 2) : -1.2; };
    s.groundSpeed = [](double) { return 10.0; };
    s.motionAccel = [](double) { return 0.6; };
    s.motionRate = [](double) { return 0.12; };
    s.navState = [](double t) { return (t > 60 && t < 80) ? NavState::Degraded : NavState::Tracking; };   // more traffic
    fsim::Runner x(s, kSeed);
    x.monitor().addSink(&q);   // the queue AND the recorder receive every event; nobody pops the queue
    x.run();
    const SafetyQueueStats st = q.stats();
    SafetyEvent e{};
    bool criticalKept = false;
    while (q.pop(e)) criticalKept = criticalKept || e.severity == SafetySeverity::Critical;
    std::printf("    [integ] D54 %zu events into a 4-slot queue with no consumer: dropped new %u, dropped queued %u, "
                "critical kept %d\n", x.rec.events.size(), unsigned(st.droppedNew), unsigned(st.droppedQueued),
                int(criticalKept));
    CHECK(criticalKept);
    CHECK_EQ(st.droppedCritical, 0u);
    CHECK(x.last.eventsRejectedBySink == st.droppedNew + st.droppedQueued);
}

TEST(integ_55_end_to_end_real_pipeline_publishes_events)
{
    // REAL preprocessing + Madgwick + UKF + safety. Device still for 8 s (calibration), then a
    // winch-like climb to ~60 m with forward speed, a glide down and a landing.
    sim::SensorSimulator s(55);
    for (auto& m : s.imu) {
        m.gyroNoise = 0.0012f;
        m.accelNoise = 0.02f;
    }
    s.baro[0].noisePa = 2.0;
    s.baro[1].noisePa = 2.0;
    // still 0-15 s; climb +2 m/s to 60 m (15-45 s); glide down -1.6 m/s to the ground (45-82.5 s); still.
    auto vz = [](double t) { return t < 15 ? 0.0 : (t < 45 ? 2.0 : (t < 82.5 ? -1.6 : 0.0)); };
    auto h = [&](double t) {   // integral of vz (analytic, piecewise)
        if (t < 15) return 0.0;
        if (t < 45) return 2.0 * (t - 15);
        if (t < 82.5) return 60.0 - 1.6 * (t - 45);
        return 0.0;
    };
    auto a = [](double t) { return 0.0 * t; };
    s.altitude = [&](uint32_t ms) { return h(ms * 1e-3); };
    s.specificForce = [&](uint32_t ms) {
        return Vec3f{0.0f, 0.0f, -static_cast<float>(units::kStandardGravity_mps2 + a(ms * 1e-3))};
    };
    PreprocessingPipeline p;
    p.start(0);
    AttitudeEstimator att{AttitudeConfig{}};
    ConstantAccelerationModel model = ConstantAccelerationModel::fromConfig(NavigationConfig{});
    VerticalNavigationEstimator nav(NavigationConfig{}, model);
    SafetyMonitor mon;
    struct NoLock final : ICriticalSection {
        void enter() override {}
        void exit() override {}
    } lock;
    SafetyEventQueue<16> q(lock);
    mon.addSink(&q);
    ProcessedSensorFrame f{};
    uint32_t lseq = 0, gseq = 0;
    std::vector<std::pair<double, FlightPhase>> phases;
    FlightPhase last = FlightPhase::Unknown;
    for (uint32_t t = 0; t <= 120000u; t += 10u) {
        const double ts = t * 1e-3;
        for (const SensorMessage& m : s.step(t)) p.ingest(m, t + 1u);
        // LiDAR (straight down, level): range = height above the flat ground + 1 m harness height
        LidarSample L{};
        L.header.id = SensorId::Lidar0;
        L.header.timestampMs = t;
        L.header.sequence = ++lseq;
        L.header.valid = true;
        L.distance_m = static_cast<float>(h(ts) + 1.0);
        L.signalStrength = 1000u;
        p.ingest(SensorMessage::fromLidar(L), t + 1u);
        if (t % 200u == 0u) {
            GnssSample g{};
            g.header.id = SensorId::Gnss0;
            g.header.timestampMs = t;
            g.header.sequence = ++gseq;
            g.header.valid = true;
            g.velocityNorth_mps = (ts > 15 && ts < 82) ? 9.0f : 0.0f;
            g.verticalVelocity_mps = static_cast<float>(vz(ts));
            g.speedAccuracy_mps = 0.3f;
            g.horizontalAccuracy_m = 2.0f;
            g.verticalAccuracy_m = 3.0f;
            g.gnssTimeOfWeek_ms = 1000u + t;
            g.numSatellites = 12u;
            g.fixType = GnssFixType::Fix3D;
            g.gnssFixOk = g.positionValid = g.heightValid = true;
            p.ingest(SensorMessage::fromGnss(g), t + 1u);
        }
        p.buildFrame(t + 1u, f);
        att.update(attitudeInputFromFrame(f));
        nav.update(navigationInputFrom(f, att.latest()));
        const SafetySnapshot& snap = mon.update(safetyInputFrom(f, att.latest(), nav.latest()));
        if (snap.phase != last) {
            phases.emplace_back(ts, snap.phase);
            last = snap.phase;
        }
    }
    std::printf("    [integ] D55 REAL pipeline phases:");
    for (const auto& ph : phases) std::printf(" %.1f:%d", ph.first, int(ph.second));
    SafetyEvent e{};
    size_t n = 0, phaseEvents = 0;
    while (q.pop(e)) {
        ++n;
        phaseEvents += (e.type == SafetyEventType::FlightPhaseChanged) ? 1u : 0u;
    }
    std::printf(" | %zu events popped (%zu phase notices), clearance %s\n", n, phaseEvents,
                mon.latest().clearanceValid ? "valid" : "invalid");
    CHECK(sequenceContains(phases, {FlightPhase::Ground, FlightPhase::TakeoffCandidate, FlightPhase::Flying,
                                    FlightPhase::LandingCandidate, FlightPhase::Landed}));
    CHECK(phaseEvents == phases.size());
    CHECK(mon.latest().clearanceValid && std::fabs(mon.latest().clearance_m - 1.0f) < 0.3f);   // standing: harness height
    CHECK(mon.latest().qualityFlags & safetyq::ModelPlaceholder);   // propagated from the real UKF
    CHECK(mon.latest().qualityFlags & safetyq::ImuMountingUnverified);
    CHECK(mon.latest().qualityFlags & safetyq::LidarMountingUnverified);
    CHECK_EQ(mon.flightState().landings, 1u);
}

TEST(integ_56_long_duration_two_hours_bounded)
{
    // 2 h of soaring at 100-400 m with turbulence: no drift into a wrong phase, no event growth.
    Scenario s;
    s.durationS = 7200;
    s.h0 = 250;
    s.vz = [](double t) { return 1.5 * std::sin(0.01 * t) + 0.5 * std::sin(0.7 * t); };
    s.groundSpeed = [](double t) { return 8.0 + 3.0 * std::sin(0.05 * t); };
    s.rollDeg = [](double t) { return 20.0 * std::sin(0.02 * t); };
    s.heading = [](double t) { return 0.2 * t; };
    s.motionAccel = [](double) { return 0.8; };
    s.motionRate = [](double) { return 0.15; };
    Run x(s);
    x.r.run();
    x.print("D56", "2 h soaring");
    CHECK(x.r.phaseChanges() == 1u);
    CHECK(x.r.last.phase == FlightPhase::Flying);
    CHECK(x.r.rec.events.size() <= 2u);
    CHECK(x.r.last.sequence == 720001u);
}

// ---------------------------------------------------------------------------
// Fault tolerance (brief section 23)
// ---------------------------------------------------------------------------

TEST(integ_faults_imu_baro_gnss_lidar_attitude_ukf)
{
    struct F { const char* name; std::function<void(Scenario&)> set; };
    const F faults[] = {
        {"all IMUs unavailable (attitude invalid)", [](Scenario& s) { s.attitudeValid = [](double t) { return t < 60; }; }},
        {"both barometers lost (UKF Unavailable)", [](Scenario& s) { s.navState = [](double t) { return t < 60 ? NavState::Tracking : NavState::Unavailable; }; }},
        {"UKF degraded", [](Scenario& s) { s.navState = [](double t) { return t < 60 ? NavState::Tracking : NavState::Degraded; }; }},
        {"GNSS unavailable", [](Scenario& s) { s.gnss = false; }},
        {"LiDAR unavailable", [](Scenario& s) { s.lidar = false; }},
        {"altitude discontinuity +30 m", [](Scenario& s) { s.navAltOffset = [](double t) { return t > 60 ? 30.0 : 0.0; }; }},
    };
    for (const F& f : faults) {
        Scenario s;
        s.durationS = 150;
        s.h0 = 200;
        s.vz = [](double) { return -1.2; };
        s.groundSpeed = [](double) { return 10.0; };
        s.motionAccel = [](double) { return 0.6; };
        s.motionRate = [](double) { return 0.12; };
        f.set(s);
        Run x(s);
        x.r.run();
        x.print("F", f.name);
        CHECK(x.r.firstPhase(FlightPhase::Landed) < 0);            // a fault never "lands" the device
        CHECK(x.r.firstPhase(FlightPhase::LandingCandidate) < 0);
        CHECK(isAirbornePhase(x.r.last.phase));
        CHECK(x.r.rec.raisedAtLeast(SafetySeverity::Warning) == 0u);   // no invented hazard
        // missing data is never shown as numbers
        if (!x.r.last.clearanceValid) CHECK(std::isnan(x.r.last.clearance_m));
        if (!x.r.last.velocityValid) CHECK(std::isnan(x.r.last.verticalVelocity_mps));
    }
}

TEST(integ_faults_contradictory_navigation_and_lidar_and_stale_timestamps)
{
    // Navigation claims -6 m/s (e.g. a barometer fault) while the LiDAR shows constant 20 m clearance:
    // ExcessiveDescent (navigation-based) is raised, but no LiDAR hazard is confirmed.
    Scenario s;
    s.durationS = 40;
    s.h0 = 20;
    s.vz = [](double) { return 0.0; };
    s.groundSpeed = [](double) { return 6.0; };
    s.navVzBias = [](double t) { return t > 10 ? -6.0 : 0.0; };
    s.motionAccel = [](double) { return 0.6; };
    s.motionRate = [](double) { return 0.12; };
    Run x(s);
    x.r.run();
    x.print("F", "nav -6 m/s vs LiDAR steady 20 m");
    CHECK(x.r.rec.first(SafetyEventType::ExcessiveDescent, SafetyTransition::Raised) != nullptr);
    CHECK(x.r.rec.count(SafetyEventType::HazardousApproach, SafetyTransition::Raised) == 0u);   // LiDAR closure rate wins
    CHECK(x.r.rec.count(SafetyEventType::RapidGroundClosure, SafetyTransition::Raised) == 0u);

    // Stale navigation timestamps (estimate frozen 2 s old) are not used as evidence.
    SafetyMonitor mon;
    SafetyInput in{};
    in.nowMs = 10000u;
    in.nav.velocityValid = in.nav.altitudeValid = true;
    in.nav.verticalVelocity_mps = -9.0f;
    in.nav.velocitySigma_mps = 0.1f;
    in.nav.timestampMs = 8000u;
    in.nav.state = NavState::Tracking;
    for (int k = 0; k < 500; ++k) {
        in.nowMs += 10u;
        mon.update(in);
    }
    CHECK(!mon.latest().warnings[uint8_t(LandingWarning::ExcessiveDescent)].active);
    CHECK(mon.flightState().flags & flightflag::NavigationUnavailable);
}

// ---------------------------------------------------------------------------
// Parameter sweeps (tuning seeds 1-3). Defaults were frozen before the held-out
// scenario tests (seed 101+) above and in test_flight_state / test_landing_safety.
// ---------------------------------------------------------------------------

namespace {

struct SweepOut {
    double takeoffDelay = 0, landingDelay = 0;
    int falseFlying = 0, falseLanded = 0, missedTakeoff = 0, missedLanding = 0, extraTransitions = 0;
    int caution = 0, descentToggles = 0;
    double descentWarnDelay = 0;
};

SweepOut sweep(const SafetyConfig& cfg)
{
    SweepOut o;
    int n = 0;
    for (uint64_t seed = 1; seed <= 3; ++seed) {
        // takeoff + landing (full flight)
        fsim::Runner f(fullFlight(), seed, cfg);
        double liftoff = -1, touchdown = -1;
        bool wasAir = false;
        f.each = [&](double t, const SafetyInput&, const SafetySnapshot&, const fsim::Truth& tr) {
            if (tr.airborne && liftoff < 0) liftoff = t;
            if (!tr.airborne && wasAir && t > 100 && touchdown < 0) touchdown = t;
            wasAir = tr.airborne;
        };
        f.run();
        const double fly = f.firstPhase(FlightPhase::Flying), landed = f.firstPhase(FlightPhase::Landed);
        if (fly < 0) ++o.missedTakeoff; else o.takeoffDelay += fly - liftoff;
        if (landed < 0) ++o.missedLanding; else o.landingDelay += landed - touchdown;
        o.extraTransitions += static_cast<int>(f.phaseChanges()) - 5;
        o.caution += static_cast<int>(f.rec.raisedAtLeast(SafetySeverity::Caution));
        // carried device (must stay on the ground)
        Scenario carry;
        carry.durationS = 120;
        carry.terrain = [](double x, double) { return 0.25 * x; };
        carry.groundSpeed = [](double t) { return (t > 20 && t < 100) ? 1.3 : 0.0; };
        carry.motionAccel = [](double t) { return (t > 20 && t < 100) ? 1.2 : 0.05; };
        carry.motionRate = [](double t) { return (t > 20 && t < 100) ? 0.4 : 0.01; };
        fsim::Runner c(carry, seed, cfg);
        c.run();
        if (c.firstPhase(FlightPhase::Flying) >= 0) ++o.falseFlying;
        // low soaring (must never land)
        Scenario soar;
        soar.durationS = 240;
        soar.h0 = 12;
        soar.vz = [](double t) { return 0.4 * std::sin(0.3 * t); };
        soar.groundSpeed = [](double t) { return 2.5 + 1.5 * std::sin(0.05 * t); };
        soar.motionAccel = [](double) { return 0.5; };
        soar.motionRate = [](double) { return 0.15; };
        fsim::Runner so(soar, seed, cfg);
        so.run();
        if (so.firstPhase(FlightPhase::Landed) >= 0 || so.firstPhase(FlightPhase::LandingCandidate) >= 0) ++o.falseLanded;
        // sink oscillating around the caution threshold, then -7 m/s
        Scenario sink;
        sink.durationS = 90;
        sink.h0 = 900;
        sink.vz = [](double t) { return t < 10 ? -1.5 : (t < 60 ? -3.5 - 0.3 * std::sin(1.5 * t) : -7.0); };
        sink.groundSpeed = [](double) { return 10.0; };
        fsim::Runner sk(sink, seed, cfg);
        sk.run();
        o.descentToggles += static_cast<int>(sk.rec.count(SafetyEventType::ExcessiveDescent, SafetyTransition::Raised) +
                                             sk.rec.count(SafetyEventType::ExcessiveDescent, SafetyTransition::Cleared));
        for (const auto& e : sk.rec.events)
            if (e.type == SafetyEventType::ExcessiveDescent && e.severity >= SafetySeverity::Warning &&
                e.transition != SafetyTransition::Cleared) {
                o.descentWarnDelay += (e.timestampMs - 200001u) * 1e-3 - 60.0;
                break;
            }
        ++n;
    }
    o.takeoffDelay /= n;
    o.landingDelay /= n;
    o.descentWarnDelay /= n;
    return o;
}

void row(const char* p, double v, const SweepOut& o)
{
    std::printf("    [sweep] %-28s %7.0f : takeoff delay %5.2f s, landing delay %5.2f s, missed T/L %d/%d, false Flying %d, "
                "false Landed %d, Approach in/out transitions %d, Caution+ in normal flight %d, descent raise+clear events %d, "
                "Warning delay %.2f s\n",
                p, v, o.takeoffDelay, o.landingDelay, o.missedTakeoff, o.missedLanding, o.falseFlying, o.falseLanded,
                o.extraTransitions, o.caution, o.descentToggles, o.descentWarnDelay);
}

}  // namespace

TEST(integ_sweep_flight_and_warning_parameters)
{
    for (uint32_t v : {1000u, 3000u, 6000u}) {
        SafetyConfig c;
        c.flight.takeoffConfirmMs = v;
        row("takeoffConfirmMs", v, sweep(c));
    }
    for (uint32_t v : {2000u, 5000u, 10000u}) {
        SafetyConfig c;
        c.flight.landingConfirmMs = v;
        row("landingConfirmMs", v, sweep(c));
    }
    for (uint32_t v : {200u, 1000u, 2000u}) {
        SafetyConfig c;
        c.landing.descentPolicy.raiseMs = v;
        row("descent raiseMs", v, sweep(c));
    }
    for (float v : {0.0f, 0.5f, 1.0f}) {
        SafetyConfig c;
        c.landing.descentHysteresis_mps = v;
        row("descent hysteresis x1000", v * 1000.0f, sweep(c));
    }
    const SweepOut d = sweep(SafetyConfig{});
    row("DEFAULT", 0, d);
    CHECK(d.missedTakeoff == 0 && d.missedLanding == 0 && d.falseFlying == 0 && d.falseLanded == 0);
    CHECK(d.caution == 0);
}
