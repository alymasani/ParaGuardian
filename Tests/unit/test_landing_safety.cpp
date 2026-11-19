/**
 * Milestone 5 - LandingSafetyAnalyzer + SafetyEvent semantics + SafetyEventQueue
 * (scenarios C31-C46 and the queue / deduplication / priority rules).
 * Expected outcomes are judged from the simulator truth (true vertical speed,
 * true clearance beneath / illuminated, true time to contact), not from the
 * analyzer's own thresholds alone.
 */
#include <cmath>
#include <cstdio>

#include "TestFramework.hpp"
#include "sim/FlightSafetySimulator.hpp"

#include "Algorithms/Safety/SafetyEventQueue.hpp"

using namespace pg;
using fsim::ramp;
using fsim::Scenario;

namespace {

constexpr uint64_t kSeed = 101u;

Scenario air(double dur, double agl, double vz = -1.2)
{
    Scenario s;
    s.durationS = dur;
    s.h0 = agl;
    s.vz = [vz](double) { return vz; };
    s.groundSpeed = [](double) { return 10.0; };
    s.motionAccel = [](double) { return 0.6; };
    s.motionRate = [](double) { return 0.12; };
    return s;
}

struct Run {
    explicit Run(const Scenario& s, uint64_t seed = kSeed, const SafetyConfig& c = SafetyConfig{}) : r(s, seed, c)
    {
        r.each = [this](double t, const SafetyInput&, const SafetySnapshot& snap, const fsim::Truth& tr) {
            if (!tr.airborne && wasAir && touchdown < 0) touchdown = t;
            wasAir = tr.airborne;
            if (extra) extra(t, snap, tr);
        };
    }
    void go() { r.run(); }
    fsim::Runner r;
    std::function<void(double, const SafetySnapshot&, const fsim::Truth&)> extra;
    double touchdown = -1;
    bool wasAir = true;
    const fsim::Recorder& ev() const { return r.rec; }
    void print(const char* id, const char* name) const
    {
        std::printf("    [landing] %s %-46s events:", id, name);
        for (const SafetyEvent& e : r.rec.events) {
            if (e.type == SafetyEventType::FlightPhaseChanged) continue;
            std::printf(" %.1f:%d/%d/%d", (e.timestampMs - 200001u) * 1e-3, int(e.type), int(e.transition), int(e.severity));
        }
        std::printf("\n");
    }
    size_t count(SafetyEventType t, SafetyTransition tr) const { return r.rec.count(t, tr); }
    double firstTime(SafetyEventType t, SafetyTransition tr, SafetySeverity minSev = SafetySeverity::Info) const
    {
        for (const auto& e : r.rec.events)
            if (e.type == t && e.transition == tr && e.severity >= minSev) return (e.timestampMs - 200001u) * 1e-3;
        return -1;
    }
    double firstSeverity(SafetyEventType t, SafetySeverity s) const
    {
        for (const auto& e : r.rec.events)
            if (e.type == t && e.severity >= s &&
                (e.transition == SafetyTransition::Raised || e.transition == SafetyTransition::Escalated))
                return (e.timestampMs - 200001u) * 1e-3;
        return -1;
    }
};

/// Normal approach + flare + landing on a flat field (from 150 m).
Scenario normalLanding()
{
    Scenario s = air(160, 150, -1.5);
    s.vz = [](double t) { return t < 95 ? -1.5 : -1.5 + 1.0 * ramp(t, 95, 3); };
    s.groundSpeed = [](double t) { return t < 100 ? 9.0 : 9.0 * (1 - ramp(t, 100, 3)); };
    s.motionAccel = [](double t) { return t < 100 ? 0.6 : (t < 106 ? 1.5 : 0.05); };
    s.motionRate = [](double t) { return t < 100 ? 0.12 : (t < 106 ? 0.3 : 0.01); };
    return s;
}

}  // namespace

TEST(ls_31_controlled_descent_high_clearance_no_warning)
{
    Run x(air(120, 300, -1.5));
    x.go();
    x.print("C31", "controlled descent -1.5 m/s at 300-120 m");
    CHECK(x.ev().raisedAtLeast(SafetySeverity::Advisory) == 0u);
    CHECK(x.r.last.activeWarningMask == 0u);
}

TEST(ls_32_37_normal_approach_flare_landing_only_advisory)
{
    Run x(normalLanding());
    x.go();
    x.print("C32/37", "normal approach, flare, touchdown");
    CHECK(x.ev().raisedAtLeast(SafetySeverity::Caution) == 0u);   // no hazard warnings in a normal landing
    CHECK(x.count(SafetyEventType::LowGroundClearance, SafetyTransition::Raised) == 1u);   // informational advisory
    CHECK(x.count(SafetyEventType::HazardousApproach, SafetyTransition::Raised) == 0u);
    CHECK(x.count(SafetyEventType::RapidGroundClosure, SafetyTransition::Raised) == 0u);
    // post-landing: everything cleared, nothing new after Landed
    CHECK(x.r.last.phase == FlightPhase::Landed && x.r.last.activeWarningMask == 0u);
    CHECK(x.count(SafetyEventType::LowGroundClearance, SafetyTransition::Cleared) == 1u);
}

TEST(ls_33_excessive_sink_escalation_and_delay)
{
    // B-stall-like -7 m/s, then spiral -12 m/s, then recovery to -1.5 m/s.
    Scenario s = air(80, 900);
    s.vz = [](double t) { return t < 20 ? -1.5 : (t < 35 ? -7.0 : (t < 50 ? -12.0 : -1.5)); };
    Run x(s);
    x.go();
    x.print("C33", "excessive sink -7 then -12 m/s");
    const double warn = x.firstSeverity(SafetyEventType::ExcessiveDescent, SafetySeverity::Warning);
    const double crit = x.firstSeverity(SafetyEventType::ExcessiveDescent, SafetySeverity::Critical);
    const double clr = x.firstTime(SafetyEventType::ExcessiveDescent, SafetyTransition::Cleared);
    std::printf("    [landing] C33 delays: Warning %.2f s after -7 m/s began, Critical %.2f s after -12 m/s, Cleared %.2f s "
                "after recovery\n", warn - 20, crit - 35, clr - 50);
    CHECK(warn > 20 && warn - 20 < 2.0);
    CHECK(crit > 35 && crit - 35 < 1.0);
    CHECK(clr > 50 && clr - 50 < 4.0);
    // one episode: Raised once, escalated, cleared once
    CHECK(x.count(SafetyEventType::ExcessiveDescent, SafetyTransition::Raised) == 1u);
    const SafetyEvent* r = x.ev().first(SafetyEventType::ExcessiveDescent, SafetyTransition::Raised);
    const SafetyEvent* c = x.ev().first(SafetyEventType::ExcessiveDescent, SafetyTransition::Cleared);
    CHECK(r && c && r->episodeId == c->episodeId);
    CHECK(r->qualityFlags & safetyq::ModelPlaceholder);   // placeholder model propagated into every event
    CHECK(r->qualityFlags & safetyq::Unvalidated);
}

TEST(ls_34_high_descent_without_lidar)
{
    Scenario s = air(40, 160, -5.0);
    s.lidar = false;
    Run x(s);
    x.go();
    x.print("C34", "-5 m/s, LiDAR absent");
    CHECK(x.firstSeverity(SafetyEventType::ExcessiveDescent, SafetySeverity::Caution) > 0);
    const SafetyEvent* t = x.ev().first(SafetyEventType::TerrainClearanceUnavailable, SafetyTransition::Raised);
    CHECK(t != nullptr && t->severity == SafetySeverity::Caution);   // fast descent with no terrain information
    CHECK(x.count(SafetyEventType::HazardousApproach, SafetyTransition::Raised) == 0u);   // not CONFIRMED without LiDAR
    CHECK(x.count(SafetyEventType::TerrainClearanceUnavailable, SafetyTransition::Raised) == 1u);   // no repeats
}

TEST(ls_35_38_fast_terrain_closure_and_rising_terrain)
{
    // Level flight at 20 m towards terrain rising at 0.6 (31 deg): closure ~6 m/s from LiDAR only.
    Scenario s = air(20, 25, 0.0);
    s.groundSpeed = [](double) { return 10.0; };
    s.terrain = [](double x, double) { return x > 60 ? 0.6 * (x - 60) : 0.0; };
    Run x(s);
    double truthTtc2 = -1, truthTtc4 = -1, truthClr6 = -1;
    x.extra = [&](double t, const SafetySnapshot&, const fsim::Truth& tr) {
        // truth TTC beneath: clearance / (rise rate of terrain) with 6 m/s closure
        const double closure = 6.0;
        if (t > 6.1 && truthTtc4 < 0 && tr.clearanceBelow / closure < 4.0) truthTtc4 = t;
        if (t > 6.1 && truthTtc2 < 0 && tr.clearanceBelow / closure < 2.0) truthTtc2 = t;
        if (t > 6.1 && truthClr6 < 0 && tr.clearanceBelow < 6.0) truthClr6 = t;
    };
    x.go();
    x.print("C35/38", "level flight into terrain rising 0.6");
    const double rapidW = x.firstSeverity(SafetyEventType::RapidGroundClosure, SafetySeverity::Warning);
    const double rapidC = x.firstSeverity(SafetyEventType::RapidGroundClosure, SafetySeverity::Critical);
    const double hazW = x.firstSeverity(SafetyEventType::HazardousApproach, SafetySeverity::Warning);
    std::printf("    [landing] C35/38 RapidClosure Warning at %.2f s (truth TTC<4 at %.2f), Critical at %.2f s (truth TTC<2 at "
                "%.2f); HazardousApproach Warning at %.2f s; truth clearance < 6 m at %.2f s\n",
                rapidW, truthTtc4, rapidC, truthTtc2, hazW, truthClr6);
    CHECK(rapidW > 0 && rapidC > 0 && hazW > 0);
    CHECK(rapidC < truthClr6);     // critical before the ground is 6 m away
    CHECK(rapidW < rapidC);        // escalation order
    CHECK(rapidC - truthTtc2 < 0.8);
}

TEST(ls_36_low_clearance_while_climbing_is_advisory_only)
{
    Scenario s = air(60, 8, 1.0);   // ridge lift: climbing from 8 m
    s.vz = [](double t) { return t < 30 ? 0.0 : 1.0; };
    s.groundSpeed = [](double) { return 3.0; };
    Run x(s);
    x.go();
    x.print("C36", "low clearance (8 m) then climbing");
    CHECK(x.count(SafetyEventType::LowGroundClearance, SafetyTransition::Raised) == 1u);
    CHECK(x.ev().raisedAtLeast(SafetySeverity::Caution) == 0u);
    CHECK(x.count(SafetyEventType::LowGroundClearance, SafetyTransition::Cleared) == 1u);   // cleared once > 15 m
}

TEST(ls_39_lidar_failure_during_approach)
{
    Scenario s = normalLanding();
    s.lidarPresent = [](double t) { return t < 80; };   // LiDAR dies at ~30 m AGL
    Run x(s);
    x.go();
    x.print("C39", "LiDAR dies during the approach");
    const SafetyEvent* t = x.ev().first(SafetyEventType::TerrainClearanceUnavailable, SafetyTransition::Raised);
    CHECK(t != nullptr);
    CHECK(t && (t->timestampMs - 200001u) * 1e-3 > 82.9 && (t->timestampMs - 200001u) * 1e-3 < 84.5);
    CHECK(x.ev().raisedAtLeast(SafetySeverity::Warning) == 0u);   // no invented hazard
    CHECK(x.count(SafetyEventType::TerrainClearanceUnavailable, SafetyTransition::Raised) == 1u);
}

TEST(ls_40_rapid_bank_and_attitude_degradation_no_false_hazard)
{
    Scenario s = air(90, 25, -0.3);
    s.vz = [](double t) { return 0.3 * std::sin(0.5 * t); };
    s.rollDeg = [](double t) { return 50.0 * std::sin(0.8 * t); };   // aggressive wingovers
    s.heading = [](double t) { return 0.3 * t; };
    s.attitudeState = [](double t) { return (t > 40 && t < 60) ? AttitudeState::Degraded : AttitudeState::Tracking6Axis; };
    s.attitudeRollErrorDeg = 3.0;
    Run x(s);
    uint32_t geomFrames = 0, frames = 0;
    x.extra = [&](double, const SafetySnapshot& snap, const fsim::Truth&) {
        ++frames;
        if (snap.clearanceStatus == ClearanceStatus::BeamGeometry) ++geomFrames;
    };
    x.go();
    x.print("C40", "50-deg wingovers at 25 m, attitude degraded");
    std::printf("    [landing] C40 beam-geometry rejections %.1f%% of frames; clearance availability %.1f%%\n",
                100.0 * geomFrames / frames, 100.0 * x.r.clearanceValidFrames / frames);
    CHECK(geomFrames > 0u);
    CHECK(x.ev().raisedAtLeast(SafetySeverity::Warning) == 0u);
}

TEST(ls_41_uncertain_navigation_velocity_not_confirmed)
{
    Scenario s = air(40, 400, -7.0);
    s.navVzSigmaFn = [](double) { return 1.5; };   // UKF velocity sigma above maxVelocitySigma (1.0)
    Run x(s);
    x.go();
    x.print("C41", "-7 m/s but nav velocity sigma 1.5 m/s");
    CHECK(x.count(SafetyEventType::ExcessiveDescent, SafetyTransition::Raised) == 0u);   // not CONFIRMED
    const SafetyEvent* n = x.ev().first(SafetyEventType::NavigationDegraded, SafetyTransition::Raised);
    CHECK(n != nullptr && n->severity == SafetySeverity::Advisory);   // ... but the consumer is told why
    CHECK(x.r.last.qualityFlags & safetyq::ModelPlaceholder);
}

TEST(ls_42_false_warning_prevention_normal_flights_10_seeds)
{
    // Normal thermal flight + normal landing, 10 held-out seeds: no Caution or higher.
    size_t falseCaution = 0, advisories = 0;
    for (uint64_t seed = 101; seed <= 110; ++seed) {
        Scenario th = air(200, 250, 1.5);
        th.vz = [](double t) { return 1.5 + 1.0 * std::sin(0.4 * t); };
        th.rollDeg = [](double) { return 25.0; };
        th.heading = [](double t) { return 0.3 * t; };
        Run a(th, seed);
        a.go();
        Run b(normalLanding(), seed);
        b.go();
        falseCaution += a.ev().raisedAtLeast(SafetySeverity::Caution) + b.ev().raisedAtLeast(SafetySeverity::Caution);
        advisories += a.ev().raisedAtLeast(SafetySeverity::Advisory) + b.ev().raisedAtLeast(SafetySeverity::Advisory);
    }
    std::printf("    [landing] C42 20 normal flights (10 seeds): %zu Caution+ warnings, %zu advisories (low clearance on "
                "final approach)\n", falseCaution, advisories);
    CHECK_EQ(falseCaution, 0u);
    CHECK(advisories <= 10u);
}

TEST(ls_43_hysteresis_no_toggling)
{
    // Sink oscillating 3.2-3.8 m/s around the 3.5 m/s caution threshold.
    Scenario s = air(90, 900);
    s.vz = [](double t) { return t < 10 ? -1.5 : -3.5 - 0.3 * std::sin(1.5 * t); };
    Run x(s);
    x.go();
    x.print("C43", "sink oscillating around the caution threshold");
    CHECK(x.count(SafetyEventType::ExcessiveDescent, SafetyTransition::Raised) == 1u);
    CHECK(x.count(SafetyEventType::ExcessiveDescent, SafetyTransition::Cleared) == 0u);
}

TEST(ls_44_45_escalation_deescalation_and_clearing_same_episode)
{
    Scenario s = air(80, 900);
    s.vz = [](double t) {
        if (t < 10) return -1.5;
        if (t < 40) return -3.0 - 8.0 * ramp(t, 10, 20);   // -3 -> -11 m/s
        if (t < 50) return -7.0;                          // back to Warning level
        return -1.5;                                     // recovered
    };
    Run x(s);
    x.go();
    x.print("C44/45", "escalation, de-escalation, clearing");
    std::vector<SafetyTransition> seq;
    uint32_t episode = 0;
    bool sameEpisode = true;
    for (const auto& e : x.ev().events) {
        if (e.type != SafetyEventType::ExcessiveDescent) continue;
        seq.push_back(e.transition);
        if (episode == 0u) episode = e.episodeId;
        sameEpisode = sameEpisode && e.episodeId == episode;
    }
    CHECK(seq.size() == 5u);
    CHECK(seq[0] == SafetyTransition::Raised && seq[1] == SafetyTransition::Escalated &&
          seq[2] == SafetyTransition::Escalated && seq[3] == SafetyTransition::Deescalated &&
          seq[4] == SafetyTransition::Cleared);
    CHECK(sameEpisode);
}

TEST(ls_46_cooldown_suppresses_repeat_but_never_escalation)
{
    Scenario s = air(80, 900);
    s.vz = [](double t) {
        if (t > 10 && t < 15) return -4.0;   // caution episode
        if (t > 20 && t < 25) return -4.0;   // same again within the 20 s cooldown: not re-announced
        if (t > 30 && t < 35) return -11.0;  // critical within the cooldown: announced
        return -1.5;
    };
    Run x(s);
    x.go();
    x.print("C46", "repeat within cooldown, then critical");
    CHECK(x.count(SafetyEventType::ExcessiveDescent, SafetyTransition::Raised) == 2u);   // first caution + critical
    CHECK(x.r.last.suppressedReactivations >= 1u);
    CHECK(x.firstSeverity(SafetyEventType::ExcessiveDescent, SafetySeverity::Critical) > 30.0);
}

TEST(ls_events_only_on_transitions)
{
    // A warning held for 60 s produces exactly one Raised event (no 10 ms flood).
    Scenario s = air(80, 2000, -7.0);
    Run x(s);
    x.go();
    size_t n = 0;
    for (const auto& e : x.ev().events) n += (e.type == SafetyEventType::ExcessiveDescent) ? 1u : 0u;
    CHECK_EQ(n, 1u);
    CHECK(x.r.last.warnings[uint8_t(LandingWarning::ExcessiveDescent)].active);
    CHECK(x.r.last.highestSeverity == SafetySeverity::Warning);
}

// ---------------------------------------------------------------------------
// SafetyEventQueue: bounded, priority-aware overflow (scenario 54)
// ---------------------------------------------------------------------------

namespace {
struct NoLock final : ICriticalSection {
    void enter() override {}
    void exit() override {}
};
SafetyEvent ev(uint32_t id, SafetySeverity s)
{
    SafetyEvent e{};
    e.eventId = id;
    e.severity = s;
    return e;
}
}  // namespace

TEST(ls_54_queue_overflow_keeps_critical_events)
{
    NoLock lock;
    SafetyEventQueue<4> q(lock);
    for (uint32_t i = 1; i <= 4; ++i) CHECK(q.publish(ev(i, SafetySeverity::Advisory)));
    // Full of advisories: a Critical replaces the OLDEST advisory.
    CHECK(!q.publish(ev(10, SafetySeverity::Critical)));   // false: something was dropped
    CHECK_EQ(q.stats().droppedQueued, 1u);
    // Another advisory now: everything queued is >= Advisory, so the NEW one is dropped.
    CHECK(!q.publish(ev(11, SafetySeverity::Advisory)));
    CHECK_EQ(q.stats().droppedNew, 1u);
    SafetyEvent out{};
    uint32_t order[4];
    for (int i = 0; i < 4; ++i) {
        CHECK(q.pop(out));
        order[i] = out.eventId;
    }
    CHECK(order[0] == 2u && order[1] == 3u && order[2] == 4u && order[3] == 10u);   // FIFO kept
    CHECK(!q.pop(out));
    // Queue full of criticals: a new critical drops the oldest critical (counted).
    for (uint32_t i = 20; i < 24; ++i) q.publish(ev(i, SafetySeverity::Critical));
    q.publish(ev(30, SafetySeverity::Critical));
    CHECK_EQ(q.stats().droppedCritical, 1u);
    CHECK(q.pop(out) && out.eventId == 21u);
    CHECK_EQ(q.stats().highWater, 4u);
}

TEST(ls_54_slow_consumer_repeated_overflow_never_loses_the_latest_critical)
{
    NoLock lock;
    SafetyEventQueue<8> q(lock);
    sim::Rng r(9);
    uint32_t criticalSent = 0;
    for (uint32_t i = 1; i <= 2000; ++i) {
        const SafetySeverity s = static_cast<SafetySeverity>(r.next() % 4u);   // Info..Warning noise
        q.publish(ev(i, s));
        if (i % 97 == 0) {
            q.publish(ev(100000u + i, SafetySeverity::Critical));
            ++criticalSent;
        }
        if (i % 10 == 0) {   // consumer 10x slower than the producer
            SafetyEvent out{};
            q.pop(out);
        }
    }
    uint32_t criticalInQueue = 0;
    SafetyEvent out{};
    const SafetyQueueStats st = q.stats();
    while (q.pop(out)) criticalInQueue += (out.severity == SafetySeverity::Critical) ? 1u : 0u;
    std::printf("    [landing] queue 8, 2000 mixed events + %u criticals, consumer 10x slower: dropped new %u, dropped "
                "queued %u, dropped critical %u\n", criticalSent, unsigned(st.droppedNew), unsigned(st.droppedQueued),
                unsigned(st.droppedCritical));
    CHECK_EQ(st.droppedCritical, 0u);   // criticals never displaced by lower priorities
    CHECK(st.droppedNew + st.droppedQueued > 1000u);
    CHECK(criticalInQueue >= 1u);
}
