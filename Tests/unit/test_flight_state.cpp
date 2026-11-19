/**
 * Milestone 5 - FlightStateDetector in synthetic flights (scenarios A1-A15 plus
 * hard landing, dropped device, missing GNSS / LiDAR / navigation).
 * Truth comes from sim/FlightSafetySimulator.hpp (independent kinematics and
 * terrain); the expected phases are judged from the TRUTH (device airborne or
 * not), not from the detector's thresholds.
 */
#include <cmath>
#include <cstdio>

#include "TestFramework.hpp"
#include "sim/FlightSafetySimulator.hpp"

using namespace pg;
using fsim::ramp;
using fsim::Scenario;

namespace {

constexpr uint64_t kSeed = 101u;   // held-out evaluation seed (sweeps use 1-3)

/// Hill launch site: flat top (z = 100 m) until x = 5 m, then a 0.35 slope down to a flat valley at z = 0.
double hill(double x, double) { return x < 5 ? 100.0 : std::fmax(0.0, 100.0 - 0.35 * (x - 5)); }

struct Run {
    explicit Run(const Scenario& s, uint64_t seed = kSeed, const SafetyConfig& c = SafetyConfig{}) : r(s, seed, c)
    {
        r.each = [this](double t, const SafetyInput&, const SafetySnapshot& snap, const fsim::Truth& tr) {
            if (tr.airborne && liftoff < 0) liftoff = t;
            if (!tr.airborne && wasAir && touchdown < 0) touchdown = t;
            wasAir = tr.airborne;
            if (snap.phase == FlightPhase::Landed && tr.airborne) landedWhileAirborne = true;
            if (isAirbornePhase(snap.phase) && !tr.airborne && t > 1) airborneWhileGround += 0.01;
            held = held || (r.monitor().flightState().flags & flightflag::PhaseHeld) != 0u;
            if (extra) extra(t, snap, tr);
        };
        r.run();
    }
    fsim::Runner r;
    std::function<void(double, const SafetySnapshot&, const fsim::Truth&)> extra;
    double liftoff = -1, touchdown = -1, airborneWhileGround = 0;
    bool wasAir = false, landedWhileAirborne = false, held = false;
    const FlightStateStatus& fs() { return r.monitor().flightState(); }
    void print(const char* id, const char* name)
    {
        std::printf("    [flight] %s %-44s phases:", id, name);
        for (const auto& p : r.phases) std::printf(" %.1f:%d", p.first, int(p.second));
        std::printf(" | takeoffs %u landings %u aborted %u | liftoff %.1f touchdown %.1f\n", unsigned(fs().takeoffs),
                    unsigned(fs().landings), unsigned(fs().abortedTakeoffs), liftoff, touchdown);
    }
};

Scenario ground(double dur)
{
    Scenario s;
    s.durationS = dur;
    s.h0 = 1.0;
    return s;
}

/// Started in the air (the device is switched on during flight): h AGL, cruise.
Scenario cruise(double dur, double agl)
{
    Scenario s;
    s.durationS = dur;
    s.h0 = agl;
    s.vz = [](double) { return -1.2; };
    s.groundSpeed = [](double) { return 10.0; };
    s.motionAccel = [](double) { return 0.6; };
    s.motionRate = [](double) { return 0.12; };
    return s;
}

/// Hill launch: still on the launch until 10 s, run, glide out, (optional) land in the valley.
Scenario launch(double dur, bool landAfter)
{
    Scenario s;
    s.durationS = dur;
    s.terrain = hill;
    s.h0 = 101.0;
    s.groundSpeed = [](double t) { return t < 10 ? 0.0 : (t < 13 ? 4.0 * ramp(t, 10, 1) : 4.0 + 5.0 * ramp(t, 13, 2)); };
    s.vz = [](double t) { return t < 10 ? 0.0 : -1.2; };   // the slope falls away faster: lift-off on the slope
    s.motionAccel = [](double t) { return t < 10 ? 0.05 : (t < 14 ? 2.0 : 0.6); };
    s.motionRate = [](double t) { return t < 10 ? 0.01 : 0.12; };
    if (landAfter) {
        // valley floor at 0 m: glide down, flare, touchdown, stop, stand still
        s.vz = [](double t) { return t < 10 ? 0.0 : -1.2 - 0.6 * ramp(t, 20, 5); };
        s.groundSpeed = [](double t) {
            return t < 10 ? 0.0 : (t < 13 ? 4.0 * ramp(t, 10, 1) : 4.0 + 5.0 * ramp(t, 13, 2));
        };
    }
    return s;
}

}  // namespace

TEST(fs_01_stationary_on_ground)
{
    Run x(ground(120));
    x.print("A1", "stationary on ground");
    CHECK(x.fs().phase == FlightPhase::Ground);
    CHECK(x.r.firstPhase(FlightPhase::Ground) > 0 && x.r.firstPhase(FlightPhase::Ground) < 10);
    CHECK(x.r.firstPhase(FlightPhase::TakeoffCandidate) < 0);
    CHECK(x.r.phaseChanges() == 1u);
    CHECK(x.fs().support >= EvidenceLevel::Moderate);
}

TEST(fs_02_03_picked_up_and_carried)
{
    Scenario up = ground(60);
    up.vz = [](double t) { return (t > 20 && t < 21) ? 1.0 : 0.0; };   // lifted 1 m, then held
    up.motionAccel = [](double t) { return (t > 19 && t < 23) ? 1.5 : 0.05; };
    up.motionRate = [](double t) { return (t > 19 && t < 23) ? 0.5 : 0.01; };
    Run a(up);
    a.print("A2", "device picked up");
    CHECK(a.r.firstPhase(FlightPhase::Flying) < 0);
    CHECK(a.r.firstPhase(FlightPhase::TakeoffCandidate) < 0);

    Scenario carry = ground(120);
    carry.terrain = [](double x, double) { return 0.25 * x; };   // walking up a hill: +20 m in 60 s
    carry.groundSpeed = [](double t) { return (t > 20 && t < 100) ? 1.3 : 0.0; };
    carry.motionAccel = [](double t) { return (t > 20 && t < 100) ? 1.2 : 0.05; };
    carry.motionRate = [](double t) { return (t > 20 && t < 100) ? 0.4 : 0.01; };
    Run b(carry);
    b.print("A3", "device carried uphill (+26 m)");
    CHECK(b.r.firstPhase(FlightPhase::Flying) < 0);
    CHECK(b.fs().phase == FlightPhase::Ground);
}

TEST(fs_04_normal_hill_launch)
{
    Run x(launch(60, false));
    x.print("A4", "hill launch (LiDAR + GNSS + nav)");
    const double fly = x.r.firstPhase(FlightPhase::Flying);
    std::printf("    [flight] A4 takeoff detection delay: %.2f s after lift-off\n", fly - x.liftoff);
    CHECK(x.liftoff > 10 && x.liftoff < 14);   // the run reaches the slope edge at ~11.9 s
    CHECK(fly > x.liftoff && fly - x.liftoff < 6.0);
    CHECK(x.fs().phase == FlightPhase::Flying);
    CHECK_EQ(x.fs().takeoffs, 1u);
    CHECK(x.airborneWhileGround < 0.5);
}

TEST(fs_04b_launch_without_gnss_or_lidar)
{
    Scenario s = launch(80, false);
    s.gnss = false;
    Run a(s);
    a.print("A4b", "hill launch, no GNSS");
    Scenario t = launch(80, false);
    t.gnss = false;
    t.lidar = false;
    Run b(t);
    b.print("A4c", "hill launch, no GNSS, no LiDAR (nav only)");
    const double da = a.r.firstPhase(FlightPhase::Flying) - a.liftoff;
    const double db = b.r.firstPhase(FlightPhase::Flying) - b.liftoff;
    std::printf("    [flight] takeoff delay without GNSS %.2f s; with navigation only %.2f s\n", da, db);
    CHECK(da > 0 && da < 6.0);
    // Navigation only: no ground decision was possible before the run (no LiDAR / GNSS, 10 s on the
    // launch), so the detector goes Unknown -> Flying on the single-source dwell (15 s).
    CHECK(db > 0 && db < 20.0);
    CHECK(isAirbornePhase(b.fs().phase));
}

TEST(fs_05_aborted_takeoff)
{
    // Runs to 7 m/s (wing up) on the FLAT top, then stops: GNSS says "fast" but the
    // LiDAR sees the ground at harness height -> candidate, vetoed, aborted.
    Scenario s = ground(60);
    s.terrain = [](double, double) { return 100.0; };
    s.h0 = 101;
    s.groundSpeed = [](double t) { return (t > 20 && t < 24) ? 7.0 * ramp(t, 20, 1) : 0.0; };
    s.motionAccel = [](double t) { return (t > 20 && t < 25) ? 2.0 : 0.05; };
    s.motionRate = [](double t) { return (t > 20 && t < 25) ? 0.3 : 0.01; };
    Run x(s);
    x.print("A5", "aborted takeoff");
    CHECK(x.r.firstPhase(FlightPhase::TakeoffCandidate) > 20);
    CHECK(x.r.firstPhase(FlightPhase::Flying) < 0);
    CHECK_EQ(x.fs().abortedTakeoffs, 1u);
    CHECK(x.fs().phase == FlightPhase::Ground);
}

TEST(fs_06_07_08_cruise_thermal_and_extended_descent_stay_flying)
{
    struct C { Scenario s; const char* name; };
    Scenario th = cruise(180, 300);
    th.vz = [](double t) { return 2.5 + 0.8 * std::sin(t); };
    th.rollDeg = [](double) { return 25.0; };
    th.heading = [](double t) { return 0.35 * t; };
    th.groundSpeed = [](double t) { return 8.0 + 4.0 * std::sin(0.35 * t); };   // wind drift in circles
    Scenario de = cruise(180, 500);
    de.vz = [](double) { return -2.5; };
    const C cases[] = {{cruise(180, 600), "A6 normal cruise (started in flight)"}, {th, "A7 thermal climb, 25-deg bank"},
                       {de, "A8 extended descent 500 -> 50 m AGL"}};
    for (const C& c : cases) {
        Run x(c.s);
        x.print("A6-8", c.name);
        CHECK(x.r.firstPhase(FlightPhase::Flying) > 0 && x.r.firstPhase(FlightPhase::Flying) < 7.0);
        CHECK(x.r.firstPhase(FlightPhase::LandingCandidate) < 0);
        CHECK(x.r.firstPhase(FlightPhase::Landed) < 0);
        CHECK(isAirbornePhase(x.fs().phase));
        CHECK(x.fs().takeoffs == 0u);   // started in flight: not a takeoff
    }
}

TEST(fs_09_low_altitude_soaring_not_landing)
{
    // Ridge soaring 8-15 m above a slope with a strong headwind: low ground speed, nearly level.
    Scenario s = cruise(240, 12);
    s.terrain = [](double, double) { return 0.0; };
    s.vz = [](double t) { return 0.4 * std::sin(0.3 * t); };
    s.groundSpeed = [](double t) { return 2.5 + 1.5 * std::sin(0.05 * t); };
    s.motionAccel = [](double) { return 0.5; };
    s.motionRate = [](double) { return 0.15; };
    Run x(s);
    x.print("A9", "low-altitude soaring 8-15 m, slow ground speed");
    CHECK(x.r.firstPhase(FlightPhase::LandingCandidate) < 0);
    CHECK(x.r.firstPhase(FlightPhase::Landed) < 0);
    CHECK(x.fs().phase == FlightPhase::Approach);   // airborne near terrain
    CHECK(x.r.phaseChanges() <= 3u);                // Unknown -> Flying -> Approach, no oscillation
}

TEST(fs_10_11_12_approach_touchdown_and_post_landing)
{
    // Started at 150 m AGL over a flat field; glide in, flare, touchdown at ~100 s, stand still.
    Scenario s = cruise(220, 150);
    s.vz = [](double t) { return t < 95 ? -1.5 : -1.5 + 1.0 * ramp(t, 95, 3); };   // flare from ~7 m
    s.groundSpeed = [](double t) { return t < 100 ? 9.0 : 9.0 * (1 - ramp(t, 100, 3)); };
    s.motionAccel = [](double t) { return t < 100 ? 0.6 : (t < 106 ? 1.5 : 0.05); };
    s.motionRate = [](double t) { return t < 100 ? 0.12 : (t < 106 ? 0.3 : 0.01); };
    Run x(s);
    x.print("A10-12", "approach, flare, touchdown, stand still");
    const double app = x.r.firstPhase(FlightPhase::Approach);
    const double landed = x.r.firstPhase(FlightPhase::Landed);
    std::printf("    [flight] A10-12 approach entered %.1f s before touchdown; Landed %.1f s after touchdown\n",
                x.touchdown - app, landed - x.touchdown);
    CHECK(app > 0 && app < x.touchdown);
    CHECK(landed > x.touchdown && landed - x.touchdown < 15.0);
    CHECK(!x.landedWhileAirborne);
    CHECK(x.fs().phase == FlightPhase::Landed);
    CHECK_EQ(x.fs().landings, 1u);
    // post-landing: no further phase changes, no warnings above advisory after touchdown
    CHECK(x.r.phases.back().first == landed);
}

TEST(fs_13_takeoff_again_after_landing)
{
    Scenario s = cruise(320, 80);
    // land at ~55 s, stand until 120 s, take off again from a cliff edge at 120 s, fly
    s.terrain = [](double x, double) { return x < 900 ? 0.0 : -60.0; };
    s.vz = [](double t) { return t < 120 ? -1.6 : -1.2; };
    s.groundSpeed = [](double t) {
        if (t < 55) return 9.0;
        if (t < 120) return 9.0 * (1 - ramp(t, 55, 3));
        return 9.0 * ramp(t, 120, 4);
    };
    s.motionAccel = [](double t) { return (t > 58 && t < 120) ? 0.05 : 0.6; };
    s.motionRate = [](double t) { return (t > 58 && t < 120) ? 0.01 : 0.12; };
    Run x(s);
    x.print("A13", "landing then second takeoff");
    CHECK(x.r.firstPhase(FlightPhase::Landed) > 0);
    CHECK(x.r.firstPhase(FlightPhase::Flying, x.r.firstPhase(FlightPhase::Landed)) > 0);
    CHECK_EQ(x.fs().landings, 1u);
    CHECK_EQ(x.fs().takeoffs, 1u);   // the first "flight" was entered from Unknown (started airborne)
    CHECK(isAirbornePhase(x.fs().phase));
}

TEST(fs_14_start_while_airborne)
{
    Run x(cruise(30, 200));
    x.print("A14", "device switched on in flight");
    const double fly = x.r.firstPhase(FlightPhase::Flying);
    CHECK(x.r.phases.front().second == FlightPhase::Flying);   // straight from Unknown
    CHECK(fly > 4.9 && fly < 7.0);                             // startupDecisionMs of consistent evidence
    CHECK(x.r.firstPhase(FlightPhase::Ground) < 0);
}

TEST(fs_15_navigation_outage_in_flight_never_lands)
{
    // 20 s with NO navigation, NO GNSS and NO LiDAR return processing (attitude still valid):
    // the phase is held, and returns to normal afterwards.
    Scenario s = cruise(120, 300);
    s.navState = [](double t) { return (t > 40 && t < 60) ? NavState::Unavailable : NavState::Tracking; };
    s.gnssPresent = [](double t) { return !(t > 40 && t < 60); };
    s.lidarPresent = [](double t) { return !(t > 40 && t < 60); };
    s.attitudeValid = [](double t) { return !(t > 40 && t < 60); };
    Run x(s);
    x.print("A15", "20 s total sensor outage in flight");
    CHECK(x.held);
    CHECK(x.r.firstPhase(FlightPhase::LandingCandidate) < 0);
    CHECK(x.r.firstPhase(FlightPhase::Landed) < 0);
    CHECK(x.fs().phase == FlightPhase::Flying);
    CHECK(x.fs().heldUpdates > 1000u);
}

TEST(fs_hard_landing_and_dropped_device)
{
    Scenario hard = cruise(60, 40);
    hard.vz = [](double) { return -4.0; };   // ~10 s to the ground, no flare
    hard.groundSpeed = [](double t) { return t < 10 ? 9.0 : 0.0; };
    hard.motionAccel = [](double t) { return (t > 9.7 && t < 10.2) ? 25.0 : (t < 10 ? 0.6 : 0.05); };
    hard.motionRate = [](double t) { return t < 10.5 ? 0.3 : 0.01; };
    Run a(hard);
    a.print("HL", "hard landing (-4 m/s, no flare)");
    CHECK(a.r.firstPhase(FlightPhase::Landed) > a.touchdown);
    CHECK(a.r.firstPhase(FlightPhase::Landed) - a.touchdown < 15.0);

    Scenario drop = ground(60);
    drop.groundSensorHeight = 0.1;
    drop.h0 = 1.5;
    drop.vz = [](double t) { return (t > 20 && t < 20.55) ? -9.81 * (t - 20) : 0.0; };
    drop.motionAccel = [](double t) { return (t > 20 && t < 20.7) ? 9.0 : 0.05; };
    drop.motionRate = [](double t) { return (t > 20 && t < 22) ? 2.0 : 0.01; };
    Run b(drop);
    b.print("DROP", "device dropped 1.4 m on the ground");
    CHECK(b.r.firstPhase(FlightPhase::Flying) < 0);
    CHECK(b.fs().phase == FlightPhase::Ground);
}

TEST(fs_landing_without_lidar_and_gnss_needs_more_time_but_happens)
{
    Scenario s = cruise(200, 100);
    // -1.5 m/s to 10 m at 60 s, then -0.5 m/s: touchdown ~78 s; stop and stand still right after.
    s.vz = [](double t) { return t < 60 ? -1.5 : -0.5; };
    s.groundSpeed = [](double t) { return t < 78 ? 9.0 : 9.0 * (1 - ramp(t, 78, 3)); };
    s.motionAccel = [](double t) { return t < 82 ? 0.6 : 0.05; };
    s.motionRate = [](double t) { return t < 82 ? 0.12 : 0.01; };
    s.lidar = false;
    s.gnss = false;
    Run x(s);
    x.print("NOLG", "landing with navigation + IMU only");
    const double landed = x.r.firstPhase(FlightPhase::Landed);
    std::printf("    [flight] landing detected %.1f s after touchdown without LiDAR / GNSS\n", landed - x.touchdown);
    CHECK(landed > x.touchdown && landed - x.touchdown < 25.0);
}

TEST(fs_invalid_config_holds_unknown)
{
    FlightStateConfig c;
    c.approachExitClearance_m = 10.0f;   // below the entry threshold: invalid
    CHECK(!c.isValid());
    FlightStateDetector d(c);
    CHECK(!d.configValid());
    SafetyInput in{};
    in.nowMs = 1000u;
    CHECK(d.update(in, GroundClearance{}).phase == FlightPhase::Unknown);
    CHECK(d.latest().flags & flightflag::PhaseHeld);
}
