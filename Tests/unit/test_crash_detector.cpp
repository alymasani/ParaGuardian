/**
 ******************************************************************************
 * @file    test_crash_detector.cpp
 * @brief   CrashDetector: unit checks (numerics, timing, corroboration,
 *          state machine) and the synthetic incident scenarios 1-25
 *          (CRASH_EMERGENCY_VALIDATION_REPORT.md section 3).
 *
 * Scenario truth (contact time, expected outcome) comes from the independent
 * simulator; the expected outcome of each class is stated BEFORE running and
 * documented in the report, including the classes the detector cannot
 * separate (hard landing vs crash).
 ******************************************************************************
 */
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "TestFramework.hpp"
#include "sim/CrashSimulator.hpp"

#include "Algorithms/Crash/CrashDetector.hpp"

using namespace pg;
using csim::Profile;
using csim::Scenario;

namespace {

struct Sink final : ISafetyEventSink {
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

/// A hand-made frame: 3 IMUs with |f| = g * fG, rate w, airborne / ground context.
CrashInput frame(uint32_t now, double fG, double wDps, FlightPhase phase, float vz = -1.0f, bool tilt = false)
{
    CrashInput in{};
    in.nowMs = now;
    for (int i = 0; i < 3; ++i) {
        in.imu[i].present = true;
        in.imu[i].samples = 1u;
        in.imu[i].peakAccel_mps2 = static_cast<float>(fG * csim::kG);
        in.imu[i].peakRate_radps = static_cast<float>(wDps * fsim::kDeg);
        in.imu[i].peakTimestampMs = now - static_cast<uint32_t>(3 - i);
    }
    in.forceValid = true;
    in.forceNorm_mps2 = static_cast<float>(fG * csim::kG);
    in.rateValid = true;
    in.rateNorm_radps = static_cast<float>(wDps * fsim::kDeg);
    in.newImuData = true;
    in.attitudeValid = true;
    in.tilt_rad = tilt ? 1.4f : 0.1f;
    in.phase = phase;
    in.vzValid = true;
    in.vz_mps = vz;
    in.gnssValid = true;
    in.groundSpeed_mps = isAirbornePhase(phase) ? 10.0f : 0.0f;
    in.gnssTimestampMs = now;
    return in;
}

/// Runs a fixed hand-made sequence: flight, then an impact of peakG at t=impactMs, then still.
CrashStatus handRun(uint32_t t0, double peakG, double wDps, bool airborne, bool tilted, uint32_t endMs,
                    Sink& sink, const CrashConfig& cfg = CrashConfig{})
{
    CrashDetector d(cfg);
    SafetyEventEmitter em;
    em.addSink(&sink);
    CrashStatus st{};
    for (uint32_t t = 0; t <= endMs; t += 10u) {
        const bool impact = t >= 20000u && t < 20030u;
        const bool after = t >= 20030u;
        CrashInput in = frame(t0 + t, impact ? peakG : 1.0, impact ? wDps : (after ? 1.0 : 5.0),
                              airborne && !after ? FlightPhase::Flying : (airborne ? FlightPhase::Flying : FlightPhase::Ground),
                              airborne && !after ? -5.0f : 0.0f, after && tilted);
        if (after) in.groundSpeed_mps = 0.0f;
        st = d.update(in, em);
    }
    return st;
}

struct Outcome {
    bool suspected;
    double latency;   ///< suspicion - contact (s), -1 if none
    CrashDecision decision;
    uint32_t candidates;
    CrashStatus at;
};

Outcome runScenario(const Scenario& s, uint64_t seed = 1u, const CrashConfig& cfg = CrashConfig{})
{
    csim::Runner r(s, seed, cfg);
    r.run();
    Outcome o{};
    o.suspected = r.suspicions > 0u;
    o.latency = (o.suspected && r.contactT >= 0) ? r.firstSuspectT - r.contactT : -1.0;
    o.decision = r.firstDecision;
    o.candidates = r.candidates;
    o.at = r.atSuspicion;
    return o;
}

}  // namespace

// ============================== unit checks ==================================

TEST(crash_config_defaults_valid_and_invalid_rejected)
{
    CHECK(CrashConfig{}.isValid());
    CrashConfig c;
    c.severeAccel_mps2 = c.impactAccel_mps2;   // must be larger
    CHECK(!c.isValid());
    CrashConfig d;
    d.minObservationMs = d.observationMs;
    CHECK(!d.isValid());
    CrashDetector det(c);
    CHECK(!det.configValid());
}

TEST(crash_single_accelerometer_threshold_never_suspects)
{
    // A 12 g spike while ON THE GROUND, never airborne: impulse alone is not enough.
    Sink sink;
    const CrashStatus st = handRun(1000u, 12.0, 50.0, false, true, 40000u, sink);
    CHECK_EQ(st.suspicions, 0u);
    CHECK_EQ(st.impactCandidates, 1u);
    CHECK(st.decision == CrashDecision::NoFlightContext);
    CHECK_EQ(sink.count(SafetyEventType::ImpactCandidate, SafetyTransition::Raised), 1u);
    CHECK_EQ(sink.count(SafetyEventType::ImpactCandidate, SafetyTransition::Cleared), 1u);
    CHECK_EQ(sink.count(SafetyEventType::CrashSuspected, SafetyTransition::Raised), 0u);
}

TEST(crash_strong_impact_in_fast_descent_is_suspected_after_min_observation)
{
    Sink sink;
    CrashDetector d;
    SafetyEventEmitter em;
    em.addSink(&sink);
    uint32_t suspectAt = 0;
    for (uint32_t t = 0; t <= 30000u; t += 10u) {
        const bool impact = t >= 20000u && t < 20030u, after = t >= 20030u;
        CrashInput in = frame(5000u + t, impact ? 10.0 : 1.0, impact ? 600.0 : 2.0, FlightPhase::Flying,
                              after ? 0.0f : -6.0f, after);
        if (after) in.groundSpeed_mps = 0.0f;
        const CrashStatus& st = d.update(in, em);
        if (st.state == CrashState::CrashSuspected && suspectAt == 0) suspectAt = t;
    }
    CHECK(suspectAt >= 22000u && suspectAt <= 22100u);   // minObservationMs after the impact
    const CrashStatus& st = d.latest();
    CHECK(st.decision == CrashDecision::StrongImpactWithContext);
    CHECK((st.evidence & crashev::FastDescent) != 0u);
    CHECK((st.evidence & crashev::AirborneAtImpact) != 0u);
    CHECK(st.impact.cls == ImpactClass::Strong);
    CHECK_EQ(st.suspicionSerial, 1u);
    CHECK_EQ(sink.count(SafetyEventType::CrashSuspected, SafetyTransition::Raised), 1u);
    // events carry the crash episode id
    CHECK(sink.ev.back().episodeId == st.episodeId);
}

TEST(crash_nan_and_negative_inputs_never_become_valid_zero)
{
    CrashDetector d;
    SafetyEventEmitter em;
    CrashInput in = frame(1000u, 1.0, 1.0, FlightPhase::Flying);
    in.imu[0].peakAccel_mps2 = std::numeric_limits<float>::quiet_NaN();
    in.imu[1].peakAccel_mps2 = std::numeric_limits<float>::infinity();
    in.imu[2].peakRate_radps = -1.0f;
    in.forceNorm_mps2 = std::numeric_limits<float>::quiet_NaN();
    const CrashStatus& st = d.update(in, em);
    CHECK(st.state == CrashState::Monitoring);
    CHECK_EQ(st.impactCandidates, 0u);
    // A NaN huge "impact" is ignored, not counted.
    CrashInput bad = frame(1010u, 1.0, 1.0, FlightPhase::Flying);
    for (auto& c : bad.imu) c.peakAccel_mps2 = std::numeric_limits<float>::quiet_NaN();
    d.update(bad, em);
    CHECK_EQ(d.latest().impactCandidates, 0u);
}

TEST(crash_duplicate_and_out_of_order_frames_are_ignored)
{
    CrashDetector d;
    SafetyEventEmitter em;
    d.update(frame(1000u, 1.0, 1.0, FlightPhase::Flying), em);
    const uint32_t stale0 = d.latest().staleInputs;
    // the same frame again (same peak timestamps): each IMU sample evaluated once
    d.update(frame(1000u, 1.0, 1.0, FlightPhase::Flying), em);
    CHECK(d.latest().staleInputs >= stale0 + 3u);
    // an older frame: ignored entirely
    const uint32_t seq = d.latest().sequence;
    d.update(frame(990u, 15.0, 900.0, FlightPhase::Flying), em);
    CHECK_EQ(d.latest().sequence, seq);
    CHECK_EQ(d.latest().impactCandidates, 0u);
}

TEST(crash_one_faulty_imu_cannot_create_or_hide_an_impulse)
{
    CrashDetector d;
    SafetyEventEmitter em;
    for (uint32_t t = 0; t < 2000u; t += 10u) d.update(frame(t, 1.0, 2.0, FlightPhase::Flying), em);
    // IMU0 alone shows 14 g: corroborated value is the second-largest (1 g)
    CrashInput in = frame(2000u, 1.0, 2.0, FlightPhase::Flying);
    in.imu[0].peakAccel_mps2 = static_cast<float>(14.0 * csim::kG);
    d.update(in, em);
    CHECK(d.latest().state == CrashState::Monitoring);
    CHECK_EQ(d.latest().uncorroboratedPeaks, 1u);
    // IMU0 dead-low during a real 10 g impact: the other two still corroborate it
    CrashInput real = frame(2500u, 10.0, 600.0, FlightPhase::Flying);
    real.imu[0].peakAccel_mps2 = 0.0f;
    real.imu[0].peakRate_radps = 0.0f;
    d.update(real, em);
    CHECK(d.latest().state == CrashState::ImpactCandidate);
    CHECK(d.latest().impact.cls == ImpactClass::Strong);
}

TEST(crash_single_imu_is_capped_at_moderate)
{
    CrashDetector d;
    SafetyEventEmitter em;
    for (uint32_t t = 0; t < 2000u; t += 10u) {
        CrashInput in = frame(t, 1.0, 2.0, FlightPhase::Flying);
        in.imu[1].samples = in.imu[2].samples = 0u;
        d.update(in, em);
    }
    CrashInput in = frame(2000u, 12.0, 900.0, FlightPhase::Flying);
    in.imu[1].samples = in.imu[2].samples = 0u;
    d.update(in, em);
    CHECK(d.latest().state == CrashState::ImpactCandidate);
    CHECK(d.latest().impact.cls == ImpactClass::Moderate);
    CHECK((d.latest().flags & crashflag::SingleImuOnly) != 0u);
}

TEST(crash_sustained_spiral_load_is_not_an_impulse)
{
    // |f| ramps to 4.5 g over 3 s (spiral): above the 4 g threshold but never an impulse.
    CrashDetector d;
    SafetyEventEmitter em;
    for (uint32_t t = 0; t < 20000u; t += 10u) {
        const double load = t < 5000u ? 1.0 : (t < 8000u ? 1.0 + 3.5 * (t - 5000u) / 3000.0 : 4.5);
        d.update(frame(t, load, 150.0, FlightPhase::Flying, -15.0f), em);
    }
    CHECK_EQ(d.latest().impactCandidates, 0u);
}

TEST(crash_imu_unavailable_state_and_recovery)
{
    CrashDetector d;
    SafetyEventEmitter em;
    d.update(frame(0u, 1.0, 1.0, FlightPhase::Ground), em);
    for (uint32_t t = 10u; t <= 1500u; t += 10u) {
        CrashInput in = frame(t, 1.0, 1.0, FlightPhase::Ground);
        for (auto& c : in.imu) c.samples = 0u;
        in.newImuData = false;
        in.forceValid = in.rateValid = false;
        d.update(in, em);
    }
    CHECK(d.latest().state == CrashState::Unavailable);
    CHECK_EQ(d.latest().unavailableEpisodes, 1u);
    d.update(frame(1510u, 1.0, 1.0, FlightPhase::Ground), em);
    CHECK(d.latest().state == CrashState::Monitoring);
}

TEST(crash_cooldown_counts_impacts_then_rearms)
{
    CrashConfig cfg;
    Sink sink;
    CrashDetector d(cfg);
    SafetyEventEmitter em;
    em.addSink(&sink);
    uint32_t t = 0;
    for (; t <= 20000u; t += 10u) d.update(frame(t, 1.0, 3.0, FlightPhase::Flying, -6.0f), em);
    for (; t <= 20030u; t += 10u) d.update(frame(t, 10.0, 700.0, FlightPhase::Flying, -6.0f), em);
    for (; t <= 30000u; t += 10u) {
        CrashInput in = frame(t, 1.0, 1.0, FlightPhase::Flying, 0.0f, true);
        in.groundSpeed_mps = 0.0f;
        d.update(in, em);
    }
    CHECK(d.latest().state == CrashState::CrashSuspected);
    // hold, then cooldown: a further impact is counted but opens no new suspicion
    const uint32_t suspectedAt = d.latest().stateSinceMs;
    for (; t <= suspectedAt + cfg.suspectedHoldMs + 100u; t += 10u) d.update(frame(t, 1.0, 1.0, FlightPhase::Landed, 0.0f), em);
    CHECK(d.latest().state == CrashState::Cooldown);
    const uint8_t impacts = d.latest().impact.impacts;
    for (uint32_t k = 0; k < 3u; ++k, t += 10u) d.update(frame(t, 9.0, 700.0, FlightPhase::Landed, 0.0f), em);
    CHECK_EQ(d.latest().impact.impacts, static_cast<uint8_t>(impacts + 1u));
    CHECK_EQ(d.latest().suspicions, 1u);
    const uint32_t cdStart = d.latest().stateSinceMs;
    for (; t <= cdStart + cfg.cooldownMs + 100u; t += 10u) d.update(frame(t, 1.0, 1.0, FlightPhase::Landed, 0.0f), em);
    CHECK(d.latest().state == CrashState::Monitoring);   // re-armed
    std::printf("    [metric] rearm: suspected-hold %u ms + cooldown %u ms after a suspicion\n",
                static_cast<unsigned>(cfg.suspectedHoldMs), static_cast<unsigned>(cfg.cooldownMs));
}

TEST(crash_suspicion_cleared_when_flight_resumes)
{
    Sink sink;
    CrashDetector d;
    SafetyEventEmitter em;
    em.addSink(&sink);
    uint32_t t = 0;
    for (; t <= 20000u; t += 10u) d.update(frame(t, 1.0, 3.0, FlightPhase::Flying, -6.0f), em);
    for (; t <= 20030u; t += 10u) d.update(frame(t, 10.0, 700.0, FlightPhase::Flying, -6.0f), em);
    for (; t <= 23000u; t += 10u) d.update(frame(t, 1.0, 1.0, FlightPhase::Flying, 0.0f), em);   // gnss 10 m/s ...
    // pre = airborne + fast descent -> suspected at +2 s, before flight resumption (3 s) is confirmed
    CHECK(d.latest().suspicions == 1u);
    for (; t <= 30000u; t += 10u) d.update(frame(t, 1.0, 1.0, FlightPhase::Flying, 0.0f), em);
    CHECK_EQ(d.latest().suspicionsCleared, 1u);
    CHECK(d.latest().state == CrashState::Cooldown);
    CHECK_EQ(sink.count(SafetyEventType::CrashSuspected, SafetyTransition::Cleared), 1u);
}

TEST(crash_timestamps_across_uint32_wrap)
{
    Sink a, b;
    const CrashStatus s1 = handRun(1000u, 10.0, 700.0, true, true, 30000u, a);
    const CrashStatus s2 = handRun(0xFFFFFFFFu - 20005u, 10.0, 700.0, true, true, 30000u, b);   // wraps at the impact
    CHECK_EQ(s1.suspicions, 1u);
    CHECK_EQ(s2.suspicions, 1u);
    CHECK(s1.decision == s2.decision);
    CHECK_EQ(s1.evidence, s2.evidence);
}

TEST(crash_history_gap_is_flagged)
{
    CrashDetector d;
    SafetyEventEmitter em;
    d.update(frame(0u, 1.0, 1.0, FlightPhase::Flying), em);
    d.update(frame(10000u, 1.0, 1.0, FlightPhase::Flying), em);   // 10 s without frames
    d.update(frame(10010u, 10.0, 700.0, FlightPhase::Flying), em);
    CHECK((d.latest().flags & crashflag::HistoryGap) != 0u);
}

// ============================== scenarios 1-25 ==============================

namespace {

Scenario s01_glide()
{
    Profile p;
    return csim::baseFlight("01 steady glide", p, 150);
}
Scenario s02_thermal()
{
    Profile p;
    p.flightRate = 0.35;
    Scenario s = csim::baseFlight("02 thermal climb", p, 120);
    s.fs.rollDeg = [](double t) { return t > 40 && t < 100 ? 30.0 : 0.0; };
    s.loadG = [](double t) { return t > 40 && t < 100 ? 1.15 : 1.0; };
    return s;
}
Scenario s03_turbulence(uint64_t seed, double maxExtraG)
{
    Profile p;
    p.flightAccel = 1.2;
    p.flightRate = 0.2;
    Scenario s = csim::baseFlight("03 turbulence", p, 130);
    sim::Rng r(seed);
    for (int k = 0; k < 40; ++k) {
        const double t = 40.0 + k * 2.1 + r.uniform();
        s.pulses.push_back(csim::Pulse{t, false, 0.6 + maxExtraG * 0.7 * r.uniform(), 60 + 60 * r.uniform(),
                                       100 + 120 * r.uniform(), 150});
    }
    return s;
}
Scenario s04_turn()
{
    Profile p;
    p.flightRate = 0.45;
    Scenario s = csim::baseFlight("04 coordinated turn", p, 110);
    s.fs.rollDeg = [](double t) { return t > 40 && t < 70 ? 45.0 : 0.0; };
    s.loadG = [](double t) { return t > 40 && t < 70 ? 1.41 : 1.0; };
    return s;
}
Scenario s05_aggressive()
{
    Profile p;
    p.climbEnd = 75.0;   // ~155 m
    Scenario s = csim::baseFlight("05 spiral + wingovers + collapse", p, 140);
    std::shared_ptr<double> tc = s.tc;
    s.fs.vz = [tc](double t) {
        if (t >= *tc || t < 13.0) return 0.0;
        if (t < 75.0) return 2.5;
        if (t >= 85.0 && t < 97.0) return -14.0;   // spiral dive
        return -1.2;
    };
    s.fs.rollDeg = [](double t) { return t >= 85 && t < 97 ? 70.0 : (t > 105 && t < 115 ? 60.0 * std::sin(t) : 0.0); };
    s.fs.motionRate = [](double t) { return t >= 85 && t < 97 ? 2.6 : (t > 105 && t < 115 ? 1.0 : 0.1); };
    s.loadG = [](double t) {
        if (t >= 85 && t < 88) return 1.0 + 2.6 * (t - 85) / 3.0;
        if (t >= 88 && t < 97) return 3.6;
        if (t > 105 && t < 115) return 1.6 + 0.9 * std::sin(1.3 * t);   // wingovers 0.7-2.5 g
        return 1.0;
    };
    s.pulses.push_back(csim::Pulse{120.0, false, 2.0, 150, 250, 300});   // collapse + recovery: 3 g, 250 dps
    return s;
}
Scenario landing(const char* name, double vz, double extraG, double durMs, double dps, bool walk, double tiltDeg = 0.0)
{
    Profile p;
    p.crashT = 45.0;
    p.crashVz = vz;
    if (walk) {
        p.postAccel = 2.0;
        p.postRate = 0.4;
        p.postWalkSpeed = 1.2;
        p.postRiseS = 1.5;
    }
    p.postTiltDeg = tiltDeg;
    Scenario s = csim::baseFlight(name, p, 110);
    csim::impactAtContact(s, extraG, durMs, dps);
    return s;
}
Scenario s08_drop()
{
    Profile p;
    p.takeoff = 1e9;   // never flies
    Scenario s = csim::baseFlight("08 dropped on the ground", p, 60);
    s.loadG = [](double t) { return t >= 20.0 && t < 20.45 ? 0.02 : 1.0; };   // free fall 0.45 s (1 m)
    s.pulses.push_back(csim::Pulse{20.45, false, 11.0, 25, 700, 80});
    s.fs.rollDeg = [](double t) { return t > 20.5 ? 90.0 : 0.0; };
    return s;
}
Scenario s09_shaken()
{
    Profile p;
    p.takeoff = 1e9;
    Scenario s = csim::baseFlight("09 picked up and shaken", p, 60);
    sim::Rng r(9);
    for (int k = 0; k < 24; ++k)
        s.pulses.push_back(csim::Pulse{30.0 + 0.25 * k, false, 1.0 + 4.0 * r.uniform(), 40, 300 + 300 * r.uniform(), 80});
    s.fs.motionAccel = [](double t) { return t > 29 && t < 37 ? 3.0 : 0.05; };
    s.fs.motionRate = [](double t) { return t > 29 && t < 37 ? 2.0 : 0.01; };
    return s;
}
Scenario s10_failedTakeoff()
{
    Profile p;
    p.climbEnd = 15.0;   // lifts 1.5 m/s ... no: climbRate below
    p.climbRate = 1.5;
    p.crashT = 15.0;     // falls back from ~3 m
    p.crashVz = -4.0;
    p.postAccel = 2.0;
    p.postRate = 0.4;
    p.postWalkSpeed = 1.0;
    p.postRiseS = 2.0;
    Scenario s = csim::baseFlight("10 failed takeoff, pilot gets up", p, 60);
    csim::impactAtContact(s, 4.0, 50, 250);
    return s;
}
Scenario crash(const char* name, double vz, double extraG, double durMs, double dps, double tiltDeg, bool still = true)
{
    Profile p;
    p.crashT = 40.0;
    p.crashVz = vz;
    p.postTiltDeg = tiltDeg;
    if (!still) {   // vigorous movement (struggling / walking on rough ground): ~4 m/s^2 linear accel
        p.postAccel = 4.0;
        p.postRate = 0.6;
        p.postRiseS = 0.5;
    }
    Scenario s = csim::baseFlight(name, p, 75);
    csim::impactAtContact(s, extraG, durMs, dps);
    s.expectSuspicion = 1;
    return s;
}
Scenario s11() { return crash("11 abrupt descent then impact", -9.0, 9.0, 40, 600, 80); }

Scenario s23_startAirborne()
{
    Profile p;
    Scenario s = csim::baseFlight("23 started while airborne", p, 80);
    std::shared_ptr<double> tc = s.tc;
    s.fs.h0 = 120.0;
    s.fs.vz = [tc](double t) { return t >= *tc ? 0.0 : (t < 60.0 ? -1.2 : -9.0); };
    s.fs.groundSpeed = [tc](double t) { return t >= *tc ? 0.0 : 10.0; };
    s.fs.motionAccel = [tc](double t) { return t >= *tc ? 0.05 : 0.4; };
    s.fs.motionRate = [tc](double t) { return t >= *tc ? 0.01 : 0.08; };
    s.fs.rollDeg = [tc](double t) { return t >= *tc ? 80.0 : 0.0; };
    s.contactAfter = 60.0;
    csim::impactAtContact(s, 9.0, 40, 600);
    s.expectSuspicion = 1;
    return s;
}

}  // namespace

TEST(crash_sim_normal_flight_classes_raise_no_suspicion)
{
    struct Case {
        Scenario s;
        uint64_t seed;
    };
    std::vector<Case> cases = {{s01_glide(), 1}, {s02_thermal(), 2}, {s03_turbulence(3, 1.8), 3}, {s04_turn(), 4},
                               {s05_aggressive(), 5}};
    for (auto& c : cases) {
        const Outcome o = runScenario(c.s, c.seed);
        std::printf("    [scenario] %-34s candidates %u suspected %d\n", c.s.name, o.candidates, int(o.suspected));
        CHECK(!o.suspected);
    }
}

TEST(crash_sim_landings_drops_handling_raise_no_suspicion)
{
    std::vector<Scenario> cases = {landing("06 normal landing", -1.2, 0.8, 80, 60, true),
                                   landing("07 hard controlled landing (4.5 g)", -2.5, 3.5, 60, 150, true),
                                   s08_drop(), s09_shaken(), s10_failedTakeoff()};
    for (auto& s : cases) {
        const Outcome o = runScenario(s, 7);
        std::printf("    [scenario] %-34s candidates %u suspected %d decision %d\n", s.name, o.candidates,
                    int(o.suspected), int(o.decision));
        CHECK(!o.suspected);
    }
    // the drop is rejected for the right reason
    const Outcome d = runScenario(s08_drop(), 8);
    CHECK(d.candidates >= 1u);
    CHECK(d.decision == CrashDecision::DropSignature || d.decision == CrashDecision::NoFlightContext);
}

TEST(crash_sim_incident_classes_are_suspected_with_latency)
{
    std::vector<Scenario> cases = {
        s11(),
        crash("12 high-rotation impact (6 g, 900 dps)", -3.0, 5.0, 60, 900, 20),
        crash("13 low-speed impact, fast sink, lying", -5.0, 3.5, 60, 150, 70),
        crash("14 impact then inactivity (7 g)", -4.5, 6.0, 50, 200, 0),
        crash("15 strong impact, pilot keeps moving", -2.0, 9.0, 40, 300, 0, false),
    };
    for (auto& s : cases) {
        const Outcome o = runScenario(s, 11);
        std::printf("    [scenario] %-40s suspected %d latency %.2f s decision %d pre %u post %u evidence 0x%x\n",
                    s.name, int(o.suspected), o.latency, int(o.decision), o.at.preCount, o.at.postCount,
                    o.at.evidence);
        CHECK(o.suspected);
        CHECK(o.latency >= 0.0 && o.latency <= 10.6);
    }
    const Outcome a = runScenario(cases[0], 11);
    CHECK(a.latency <= 2.6);   // strong impact + 2 context indicators: decided at minObservation
    const Outcome m = runScenario(cases[4], 11);
    CHECK(m.decision == CrashDecision::StrongImpactFlightEnded);
    CHECK((m.at.evidence & crashev::PilotActivity) != 0u);   // reported: the pilot is moving
}

TEST(crash_sim_degraded_sensors_still_detect)
{
    Scenario noGnss = s11();
    noGnss.name = "16 impact, GNSS missing";
    noGnss.fs.gnss = false;
    Scenario noLidar = s11();
    noLidar.name = "17 impact, LiDAR missing";
    noLidar.fs.lidar = false;
    Scenario noBaro = s11();
    noBaro.name = "18 impact, barometers failed (nav out)";
    noBaro.fs.navState = [](double t) { return t > 30.0 ? NavState::Unavailable : NavState::Tracking; };
    Scenario sat = crash("19 25 g impact, IMUs saturate", -9.0, 24.0, 30, 600, 80);
    Scenario oneImu = s11();
    oneImu.name = "20 one IMU failed";
    oneImu.chanPresent[2] = [](double) { return false; };
    Scenario disagree = s11();
    disagree.name = "21b IMU0 reads +3 g off, real crash";
    disagree.chanFaultG[0] = [](double) { return 3.0; };
    // slow sink (only "airborne" as context): the decision must come from the explicit data-loss policy
    Scenario dropout = crash("22 all IMU data lost after impact", -2.0, 9.0, 40, 600, 0);
    {
        std::shared_ptr<double> tc = dropout.tc;
        dropout.imuData = [tc](double t) { return t < *tc + 0.1; };
        dropout.fs.attitudeValid = [tc](double t) { return t < *tc + 0.1; };
    }
    std::vector<Scenario> cases = {noGnss, noLidar, noBaro, sat, oneImu, disagree, dropout, s23_startAirborne()};
    for (auto& s : cases) {
        const Outcome o = runScenario(s, 21);
        std::printf("    [scenario] %-40s suspected %d latency %.2f s decision %d flags 0x%x sat %d\n", s.name,
                    int(o.suspected), o.latency, int(o.decision), o.at.flags, int(o.at.impact.saturated));
        CHECK(o.suspected);
    }
    const Outcome so = runScenario(sat, 21);
    CHECK(so.at.impact.saturated);
    CHECK((so.at.evidence & crashev::ImuSaturation) != 0u);
    const Outcome dl = runScenario(dropout, 21);
    CHECK(dl.decision == CrashDecision::StrongImpactDataLoss);
    CHECK((dl.at.evidence & crashev::PostImpactDataMissing) != 0u);
    const Outcome ng = runScenario(noGnss, 21);
    CHECK((ng.at.flags & crashflag::GnssMissing) != 0u);
}

TEST(crash_sim_single_imu_spike_in_flight_is_ignored)
{
    Profile p;
    Scenario s = csim::baseFlight("21 IMU0 alone spikes 12 g", p, 90);
    s.chanFaultG[0] = [](double t) { return t > 50.0 && t < 50.03 ? 11.0 : 0.0; };
    csim::Runner r(s, 21);
    r.run();
    std::printf("    [scenario] %-40s candidates %u uncorroborated %u\n", s.name, r.candidates, r.last.uncorroboratedPeaks);
    CHECK_EQ(r.candidates, 0u);
    CHECK(r.last.uncorroboratedPeaks >= 1u);
}

TEST(crash_sim_repeated_impacts_and_wraparound)
{
    Scenario rep = crash("24 tumbling: three impacts", -3.5, 4.0, 50, 200, 65);
    rep.pulses.push_back(csim::Pulse{0.8, true, 4.0, 50, 300, 100});
    rep.pulses.push_back(csim::Pulse{1.6, true, 3.5, 50, 300, 100});
    const Outcome o = runScenario(rep, 24);
    std::printf("    [scenario] %-40s suspected %d latency %.2f s impacts %u evidence 0x%x\n", rep.name,
                int(o.suspected), o.latency, o.at.impact.impacts, o.at.evidence);
    CHECK(o.suspected);
    CHECK((o.at.evidence & crashev::RepeatedImpacts) != 0u);

    Scenario w = s11();
    w.name = "25 timestamps wrap during the flight";
    w.fs.t0Ms = 0xFFFFFFFFu - 30000u;
    const Outcome a = runScenario(s11(), 25), b = runScenario(w, 25);
    std::printf("    [scenario] %-40s latency %.2f s (no wrap %.2f s)\n", w.name, b.latency, a.latency);
    CHECK(b.suspected);
    CHECK(std::fabs(a.latency - b.latency) < 0.05);
    CHECK(a.decision == b.decision);
}
