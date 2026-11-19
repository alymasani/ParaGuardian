/**
 * Vertical UKF milestone - independent simulation scenarios 1-30, the UKF vs
 * the barometer-only baseline on IDENTICAL inputs, uncertainty consistency
 * (NEES / NIS / coverage, several seeds), and limited parameter sweeps
 * (tuning seeds 1-5, held-out evaluation seeds 101+).
 *
 * The truth (sim/VerticalSimulator.hpp) is NOT generated with the filter's
 * constant-acceleration model except in the explicit "matched" consistency
 * test. Scenario motions are classes of vertical movement, not validated
 * paraglider aerodynamics.
 *
 * Every [nav] line is a metric reported in VERTICAL_UKF_VALIDATION_REPORT.md.
 */
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "TestFramework.hpp"
#include "sim/VerticalSimulator.hpp"

using namespace pg;
using vsim::AttQuality;
using vsim::Metrics;
using vsim::Runner;
using vsim::Scenario;

namespace {

constexpr uint64_t kEvalSeed = 101u;   // held-out seed for the scenario table

/// Realistic sensor errors (not matched to the filter's assumptions).
void realistic(Scenario& s)
{
    s.baroNoise = 0.4;          // per barometer
    s.baroCorrSigma = 0.25;     // correlated airflow / pressure error, common to both barometers
    s.baroCorrTau = 3.0;
    s.accelNoise = 0.15;
    s.accelCorrSigma = 0.05;    // slowly varying attitude-related error
    s.accelCorrTau = 5.0;
}

Scenario base(const char* name, double dur = 60.0)
{
    Scenario s;
    s.name = name;
    s.durationS = dur;
    realistic(s);
    return s;
}

/// Band-limited "turbulence": sum of sinusoids 0.3-3 Hz, total RMS ~amp/sqrt(2*...) (deterministic phases).
std::function<double(double)> turbulence(double amp, uint64_t seed)
{
    sim::Rng r(seed);
    std::vector<double> f, p, a;
    for (int i = 0; i < 12; ++i) {
        f.push_back(0.3 + 2.7 * r.uniform());
        p.push_back(2 * vsim::kPi * r.uniform());
        a.push_back(amp / std::sqrt(6.0) * (0.5 + r.uniform()));
    }
    return [f, p, a](double t) {
        double s = 0;
        for (size_t i = 0; i < f.size(); ++i) s += a[i] * std::sin(2 * vsim::kPi * f[i] * t + p[i]);
        return s;
    };
}

/// Thermal-like lift: every 25 s enter a thermal (+3.5 m/s over 2 s), core oscillation, exit to sink.
double thermal(double t)
{
    const double c = std::fmod(t, 25.0);
    double a = vsim::velocityChange(c, 2.0, 2.0, +3.5) + vsim::velocityChange(c, 14.0, 2.5, -4.5) +
               vsim::velocityChange(c, 20.0, 3.0, +1.0);
    if (c > 4.0 && c < 14.0) a += 0.6 * std::sin(2 * vsim::kPi * (c - 4.0) / 5.0);   // core oscillation
    return a;
}

/// One scenario by number (1..30). Scenario-specific checks are in the TESTs.
Scenario scenario(int id)
{
    switch (id) {
        case 1: return base("stationary");
        case 2: { Scenario s = base("constant climb +2 m/s"); s.v0 = 2.0; return s; }
        case 3: { Scenario s = base("constant descent -1.2 m/s"); s.v0 = -1.2; return s; }
        case 4: {
            Scenario s = base("increasing descent -1 -> -6 m/s");
            s.v0 = -1.0;
            s.accel = [](double t) { return vsim::velocityChange(t, 15.0, 10.0, -5.0); };
            return s;
        }
        case 5: {
            Scenario s = base("decreasing descent -6 -> -1.5 m/s");
            s.v0 = -6.0;
            s.accel = [](double t) { return vsim::velocityChange(t, 15.0, 6.0, +4.5); };
            return s;
        }
        case 6: {
            Scenario s = base("climb +2.5 -> descent -1.5 m/s in 3 s");
            s.v0 = 2.5;
            s.accel = [](double t) { return vsim::velocityChange(t, 20.0, 3.0, -4.0); };
            return s;
        }
        case 7: {
            Scenario s = base("smooth acceleration changes (0.8 m/s^2, 8 s)");
            s.accel = [](double t) { return 0.8 * std::sin(2 * vsim::kPi * t / 8.0); };
            return s;
        }
        case 8: {
            Scenario s = base("abrupt acceleration steps (+-1.5 m/s^2)");
            s.accel = [](double t) {
                const double c = std::fmod(t, 10.0);
                return (c >= 3.0 && c < 4.0) ? 1.5 : (c >= 4.0 && c < 5.0) ? -1.5 : 0.0;
            };
            return s;
        }
        case 9: { Scenario s = base("thermal-like lift cycles", 100.0); s.v0 = -1.2; s.accel = thermal; return s; }
        case 10: {
            Scenario s = base("turbulence-like disturbances (1.5 m/s^2)");
            s.v0 = -1.0;
            s.accel = turbulence(1.5, 7);
            return s;
        }
        case 11: {
            Scenario s = base("coordinated turn: Madgwick turn gate 20 s, attitude error", 60.0);
            s.v0 = -1.5;
            s.attitude = [](double t) { return (t > 20 && t < 40) ? AttQuality::TurnGate : AttQuality::Good; };
            s.accelError = [](double t) { return (t > 20 && t < 40) ? 0.15 + 0.1 * std::sin(0.5 * t) : 0.0; };
            return s;
        }
        case 12: { Scenario s = base("barometer noise 1.5 m per sensor"); s.baroNoise = 1.5; s.v0 = -1.0; return s; }
        case 13: { Scenario s = base("barometer weather drift 0.01 m/s", 300.0); s.baroDrift_mps = 0.01; return s; }
        case 14: {
            Scenario s = base("barometer step +15 m on BOTH sensors at 30 s");
            s.baroFault = [](double t, int) { return t > 30.0 ? 15.0 : 0.0; };
            return s;
        }
        case 15: {
            Scenario s = base("one barometer lost at 20 s");
            s.v0 = -1.0;
            s.baroPresent = [](double t, int j) { return !(j == 1 && t > 20.0); };
            return s;
        }
        case 16: {
            Scenario s = base("both barometers lost 20-35 s");
            s.v0 = -1.0;
            s.accel = [](double t) { return vsim::velocityChange(t, 25.0, 3.0, -2.0); };
            s.baroPresent = [](double t, int) { return !(t > 20.0 && t < 35.0); };
            return s;
        }
        case 17: { Scenario s = base("accelerometer bias +0.2 m/s^2"); s.accelBias = 0.2; s.v0 = -1.0; return s; }
        case 18: {
            Scenario s = base("acceleration saturation spikes (35 m/s^2, 0.3 s)");
            s.accel = [](double t) {
                const double c = std::fmod(t, 15.0);
                return (c > 10.0 && c < 10.3) ? 35.0 * std::sin(vsim::kPi * (c - 10.0) / 0.3) : 0.0;
            };
            s.accelSaturation_mps2 = 30.0;
            return s;
        }
        case 19: {
            Scenario s = base("Madgwick degraded 15 s (attitude error 0.5 m/s^2)");
            s.v0 = -1.0;
            s.attitude = [](double t) {
                return (t > 20 && t < 35) ? AttQuality::Degraded : (t >= 35 && t < 36) ? AttQuality::GapDetected : AttQuality::Good;
            };
            s.accelError = [](double t) { return (t > 20 && t < 36) ? 0.5 * std::sin(0.7 * t) : 0.0; };
            return s;
        }
        case 20: {
            Scenario s = base("GNSS dropout 20-40 s");
            s.gnss = true;
            s.accel = turbulence(1.0, 9);
            s.gnssPresent = [](double t) { return !(t > 20 && t < 40); };
            return s;
        }
        case 21: {
            Scenario s = base("GNSS 5% outliers (+-5 m/s)");
            s.gnss = true;
            s.v0 = -1.5;
            s.gnssOutlierProb = 0.05;
            return s;
        }
        case 22: {
            Scenario s = base("mixed rates: baro 50/25 Hz, GNSS 10 Hz + 100 ms latency");
            s.gnss = true;
            s.gnssPeriodMs = 100u;
            s.gnssLatencyMs = 100u;
            s.baroPeriodMs[1] = 40u;
            s.accel = [](double t) { return 0.8 * std::sin(2 * vsim::kPi * t / 8.0); };
            return s;
        }
        case 23: {
            Scenario s = base("irregular frames 5-15 ms + 3 ms accel jitter");
            s.frameJitterMs = 10u;
            s.accelJitterMs = 3u;
            s.accel = [](double t) { return 0.8 * std::sin(2 * vsim::kPi * t / 8.0); };
            return s;
        }
        case 24: {
            Scenario s = base("every frame republished twice (duplicates)");
            s.republish = 2u;
            s.accel = [](double t) { return 0.8 * std::sin(2 * vsim::kPi * t / 8.0); };
            return s;
        }
        case 25: {
            Scenario s = base("2% of frames replayed 30 ms late (out of order)");
            s.outOfOrderProb = 0.02;
            s.accel = [](double t) { return 0.8 * std::sin(2 * vsim::kPi * t / 8.0); };
            return s;
        }
        case 26: {
            Scenario s = base("uint32 ms wrap-around at 20 s");
            s.t0Ms = 0xFFFFFFFFu - 19999u;
            s.accel = [](double t) { return 0.8 * std::sin(2 * vsim::kPi * t / 8.0); };
            return s;
        }
        case 27: {
            Scenario s = base("start-up in flight (-3 m/s, accelerating)");
            s.v0 = -3.0;
            s.accel = [](double t) { return 0.6 * std::sin(2 * vsim::kPi * t / 6.0); };
            return s;
        }
        case 28: {
            Scenario s = base("complete sensor outage 20-25 s");
            s.v0 = -1.0;
            s.baroPresent = [](double t, int) { return !(t > 20.0 && t < 25.0); };
            s.accelPresent = [](double t) { return !(t > 20.0 && t < 25.0); };
            return s;
        }
        case 29: {
            Scenario s = base("barometers lost 20-28 s with +0.1 m/s^2 accel bias, recovery");
            s.v0 = -1.0;
            s.accelBias = 0.1;
            s.baroPresent = [](double t, int) { return !(t > 20.0 && t < 28.0); };
            return s;
        }
        case 30: {
            Scenario s = base("long duration 30 min (thermals + turbulence)", 1800.0);
            auto turb = turbulence(0.7, 11);
            s.v0 = -1.0;
            s.accel = [turb](double t) { return thermal(t) + turb(t); };
            return s;
        }
        default: return base("?");
    }
}

void print(int id, const Scenario& s, const Runner& r)
{
    const Metrics& u = r.ukf;
    const Metrics& b = r.base;
    std::printf("    [nav] S%02d %-58s | UKF h %.3f/%.3f/%.3f/%.3f v %.3f/%.3f/%.3f/%.3f a %.3f | BASE h %.3f/%.3f/%.3f/%.3f v "
                "%.3f/%.3f/%.3f/%.3f | usable %.1f%%/%.1f%% | bias h %+.3f v %+.3f | rej b%u a%u g%u rec %u | nf %u\n",
                id, s.name, u.h.mae(), u.h.rmse(), u.h.p95(), u.h.maxAbs, u.v.mae(), u.v.rmse(), u.v.p95(), u.v.maxAbs,
                u.a.rmse(), b.h.mae(), b.h.rmse(), b.h.p95(), b.h.maxAbs, b.v.mae(), b.v.rmse(), b.v.p95(), b.v.maxAbs,
                100 * u.usable(), 100 * b.usable(), u.h.bias(), u.v.bias(), unsigned(u.last.baro.rejectedGate),
                unsigned(u.last.accel.rejectedGate + u.last.accel.rejectedQuality), unsigned(u.last.gnss.rejectedGate),
                unsigned(u.last.baro.recoveries + u.last.accel.recoveries), unsigned(u.last.numericalFaults));
}

/// Runs scenario @p id with the held-out seed and prints its table row.
struct Run {
    explicit Run(int id, uint64_t seed = kEvalSeed, const NavigationConfig& c = NavigationConfig{})
        : sc(scenario(id)), r(sc, seed, c)
    {
        r.run();
        print(id, sc, r);
    }
    Scenario sc;
    Runner r;
    const Metrics& u() const { return r.ukf; }
    const Metrics& b() const { return r.base; }
};

/// Shared expectations of every scenario: no numerical fault, covariance symmetric and bounded.
bool healthy(const Metrics& m)
{
    return m.last.numericalFaults == 0u && m.maxAsymmetry < 1e-6 && m.minEigProxy > 0.0;
}

}  // namespace

// ---------------------------------------------------------------------------
// Scenarios 1-10: motion classes (process-model mismatch: none of these is a
// constant-acceleration trajectory driven by white jerk)
// ---------------------------------------------------------------------------

TEST(nav_sim_01_stationary)
{
    Run x(1);
    CHECK(healthy(x.u()));
    CHECK(x.u().usable() > 0.99);
    CHECK(x.u().v.rmse() < 0.15);
    CHECK(x.u().h.rmse() < 0.6);
    CHECK(x.u().last.state == NavState::Tracking);
}

TEST(nav_sim_02_03_constant_climb_and_descent)
{
    for (int id : {2, 3}) {
        Run x(id);
        CHECK(healthy(x.u()));
        CHECK(x.u().usable() > 0.99);
        CHECK(std::fabs(x.u().v.bias()) < 0.1);
        CHECK(x.u().v.rmse() < 0.2);
    }
}

TEST(nav_sim_04_05_increasing_and_decreasing_descent)
{
    for (int id : {4, 5}) {
        Run x(id);
        CHECK(healthy(x.u()));
        CHECK(x.u().v.rmse() < 0.25);
        CHECK(x.u().v.maxAbs < 1.0);
        CHECK(x.u().v.rmse() < x.b().v.rmse());   // acceleration lets the UKF follow the change
    }
}

TEST(nav_sim_06_climb_to_descent_response_delay)
{
    // Delay: time from the truth crossing the mid velocity (+0.5 m/s) to each estimate crossing it.
    Scenario s = scenario(6);
    Runner r(s, kEvalSeed);
    double tTruth = -1, tUkf = -1, tBase = -1;
    r.each = [&](double t, const NavigationInput&, const NavigationEstimate& u, const NavigationEstimate& b,
                 const vsim::TruthPoint& tp) {
        if (t < 15.0) return;
        if (tTruth < 0 && tp.v < 0.5) tTruth = t;
        if (tUkf < 0 && u.velocityValid && u.verticalVelocity_mps < 0.5f) tUkf = t;
        if (tBase < 0 && b.velocityValid && b.verticalVelocity_mps < 0.5f) tBase = t;
    };
    r.run();
    print(6, s, r);
    std::printf("    [nav] S06 response delay at the mid-velocity crossing: UKF %.0f ms, baseline %.0f ms\n",
                1000 * (tUkf - tTruth), 1000 * (tBase - tTruth));
    CHECK(healthy(r.ukf));
    CHECK(tUkf - tTruth < 0.15);
    CHECK(tBase - tTruth > tUkf - tTruth);
}

TEST(nav_sim_07_08_smooth_and_abrupt_acceleration)
{
    for (int id : {7, 8}) {
        Run x(id);
        CHECK(healthy(x.u()));
        CHECK(x.u().v.rmse() < 0.25);
        CHECK(x.u().a.rmse() < 0.35);
        CHECK(x.u().v.rmse() < x.b().v.rmse());
    }
}

TEST(nav_sim_09_10_thermal_and_turbulence)
{
    for (int id : {9, 10}) {
        Run x(id);
        CHECK(healthy(x.u()));
        CHECK(x.u().v.rmse() < 0.3);
        CHECK(x.u().v.rmse() < x.b().v.rmse());
    }
}

// ---------------------------------------------------------------------------
// 11-19: sensor quality, faults and losses
// ---------------------------------------------------------------------------

TEST(nav_sim_11_coordinated_turn_attitude_degradation)
{
    Run x(11);
    CHECK(healthy(x.u()));
    CHECK(x.u().v.rmse() < 0.3);
    CHECK(x.u().usable() > 0.99);
}

TEST(nav_sim_12_barometer_noise)
{
    Run x(12);
    CHECK(healthy(x.u()));
    CHECK(x.u().v.rmse() < x.b().v.rmse());
    CHECK(x.u().h.rmse() < 1.2);
}

TEST(nav_sim_13_barometer_drift_is_followed_not_detected)
{
    // A common weather drift is indistinguishable from altitude change for a relative
    // barometer: both estimators follow it. Reported, not hidden.
    Run x(13);
    CHECK(healthy(x.u()));
    const double driftAtEnd = 0.01 * 300.0;
    std::printf("    [nav] S13 altitude error at end %.2f m (drift %.1f m), velocity bias %+.4f m/s\n",
                double(x.u().last.altitude_m) - 500.0, driftAtEnd, x.u().v.bias());
    CHECK(std::fabs(x.u().v.bias() - 0.01) < 0.03);
}

TEST(nav_sim_14_barometer_step_fault)
{
    // Both barometers +15 m (a fault the 2-barometer vote cannot catch): rejected for
    // baroRejectRecoveryMs, then re-anchored. Single-barometer step: vote Ambiguous.
    Scenario s = scenario(14);
    Runner r(s, kEvalSeed);
    double firstFollow = -1;
    float maxVzErr = 0;
    r.each = [&](double t, const NavigationInput&, const NavigationEstimate& u, const NavigationEstimate&,
                 const vsim::TruthPoint& tp) {
        if (t > 30.0 && firstFollow < 0 && u.altitude_m - tp.h > 10.0f) firstFollow = t - 30.0;
        if (t > 30.0 && u.velocityValid) maxVzErr = std::fmax(maxVzErr, std::fabs(u.verticalVelocity_mps - float(tp.v)));
    };
    r.run();
    print(14, s, r);
    std::printf("    [nav] S14 common +15 m step: UKF follows after %.2f s (%u samples rejected first); max |vz err| after "
                "step %.2f m/s\n", firstFollow, unsigned(r.ukf.last.baro.rejectedGate), double(maxVzErr));
    CHECK(healthy(r.ukf));
    CHECK(firstFollow > 1.9 && firstFollow < 3.0);
    CHECK(r.ukf.last.baro.recoveries >= 1u);

    Scenario one = scenario(14);
    one.name = "barometer step +15 m on ONE sensor at 30 s";
    one.baroFault = [](double t, int j) { return (t > 30.0 && j == 0) ? 15.0 : 0.0; };
    Runner r1(one, kEvalSeed);
    bool degraded = false;
    double unavailableAt = -1;
    r1.each = [&](double t, const NavigationInput&, const NavigationEstimate& u, const NavigationEstimate&,
                  const vsim::TruthPoint&) {
        degraded = degraded || (t > 30.0 && u.state == NavState::Degraded);
        if (unavailableAt < 0 && u.state == NavState::Unavailable) unavailableAt = t;
    };
    r1.run();
    print(14, one, r1);
    std::printf("    [nav] S14 one-sensor step: vote Ambiguous -> accel-only Degraded, Unavailable at %.1f s "
                "(maxBaroOutageMs 20 s after the last accepted sample)\n", unavailableAt);
    CHECK(healthy(r1.ukf));
    CHECK(r1.ukf.last.baro.last == MeasurementOutcome::Ambiguous);   // vote refuses; UKF does not use it
    CHECK(degraded);
    CHECK(unavailableAt > 49.0 && unavailableAt < 51.5);
    CHECK(r1.ukf.h.maxAbs < 15.0);   // never jumped to the faulty value
}

TEST(nav_sim_15_one_barometer_lost)
{
    Run x(15);
    CHECK(healthy(x.u()));
    CHECK(x.u().usable() > 0.99);
    CHECK(x.u().last.has(navflag::BaroRedundancyDegraded));
    CHECK(x.u().v.rmse() < 0.25);
}

TEST(nav_sim_16_both_barometers_lost_15_s)
{
    Scenario s = scenario(16);
    Runner r(s, kEvalSeed);
    float maxHerrOutage = 0, sigmaAtEnd = 0;
    bool degraded = false;
    r.each = [&](double t, const NavigationInput&, const NavigationEstimate& u, const NavigationEstimate&,
                 const vsim::TruthPoint& tp) {
        if (t > 20.5 && t < 35.0) {
            degraded = degraded || u.state == NavState::Degraded;
            if (u.altitudeValid) maxHerrOutage = std::fmax(maxHerrOutage, std::fabs(u.altitude_m - float(tp.h)));
            sigmaAtEnd = u.altitudeSigma_m;
        }
    };
    r.run();
    print(16, s, r);
    std::printf("    [nav] S16 15 s barometer outage (accel only): max |h err| %.2f m, altitude sigma at end %.2f m, "
                "final state %d\n", double(maxHerrOutage), double(sigmaAtEnd), int(r.ukf.last.state));
    CHECK(healthy(r.ukf));
    CHECK(degraded);
    CHECK(r.ukf.last.state == NavState::Tracking);   // recovered
    CHECK(maxHerrOutage < 3.0f * sigmaAtEnd + 1.0f);
}

TEST(nav_sim_17_accelerometer_bias)
{
    Run x(17);
    CHECK(healthy(x.u()));
    // KNOWN LIMITATION (reported, not hidden): a constant accelerometer bias b is not a
    // state, so vz is biased by roughly b x (barometer correction time constant ~1.3 s).
    // Here the UKF is WORSE than the barometer-only baseline, which ignores the accelerometer.
    std::printf("    [nav] S17 +0.2 m/s^2 bias: UKF velocity bias %+.3f m/s, acceleration bias %+.3f m/s^2; velocity "
                "rmse UKF %.3f vs baseline %.3f m/s (UKF worse: bias not a state)\n",
                x.u().v.bias(), x.u().a.bias(), x.u().v.rmse(), x.b().v.rmse());
    CHECK(std::fabs(x.u().v.bias()) < 0.3);   // regression bound for the current tuning
    CHECK(x.u().last.baro.rejectedGate < 10u);
}

TEST(nav_sim_18_acceleration_saturation)
{
    Run x(18);
    CHECK(healthy(x.u()));
    CHECK(x.u().usable() > 0.97);
    CHECK(x.u().v.maxAbs < 3.0);
}

TEST(nav_sim_19_madgwick_degraded_mode)
{
    Run x(19);
    CHECK(healthy(x.u()));
    CHECK(x.u().last.accel.rejectedQuality >= 90u);   // gap-detected second rejected
    CHECK(x.u().v.rmse() < 0.35);
}

// ---------------------------------------------------------------------------
// 20-26: GNSS and timing
// ---------------------------------------------------------------------------

TEST(nav_sim_20_21_gnss_dropout_and_outliers)
{
    Run d(20);
    CHECK(healthy(d.u()));
    CHECK(d.u().last.gnss.accepted > 100u);
    CHECK(d.u().usable() > 0.99);
    Run o(21);
    CHECK(healthy(o.u()));
    CHECK(o.u().last.gnss.rejectedGate >= 10u);   // outliers gated
    CHECK(o.u().v.maxAbs < 1.0);
    // Same scenario without GNSS: the UKF must work with no GNSS at all.
    Scenario s = scenario(21);
    s.gnss = false;
    Runner r(s, kEvalSeed);
    r.run();
    std::printf("    [nav] S21 without GNSS: v rmse %.3f m/s (with GNSS %.3f)\n", r.ukf.v.rmse(), o.u().v.rmse());
    CHECK(r.ukf.usable() > 0.99);
}

TEST(nav_sim_22_23_mixed_rates_and_irregular_dt)
{
    Run m(22);
    CHECK(healthy(m.u()));
    CHECK(m.u().v.rmse() < 0.25);
    CHECK(m.u().last.gnss.accepted > 400u);
    Run i(23);
    CHECK(healthy(i.u()));
    CHECK(i.u().v.rmse() < 0.25);
}

TEST(nav_sim_24_duplicates_bit_identical)
{
    Run dup(24);
    Scenario clean = scenario(24);
    clean.republish = 0u;
    Runner r(clean, kEvalSeed);
    r.run();
    const NavigationEstimate& a = dup.u().last;
    const NavigationEstimate& b = r.ukf.last;
    CHECK(a.altitude_m == b.altitude_m && a.verticalVelocity_mps == b.verticalVelocity_mps);
    CHECK(a.baro.accepted == b.baro.accepted && a.accel.accepted == b.accel.accepted);
    CHECK(a.accel.notNew >= 2u * 5900u);
    std::printf("    [nav] S24 %u repeated acceleration samples skipped; final state bit-identical to the clean run\n",
                unsigned(a.accel.notNew));
}

TEST(nav_sim_25_out_of_order_frames)
{
    Run x(25);
    CHECK(healthy(x.u()));
    CHECK(x.r.replayed > 50u);
    CHECK(x.u().last.accel.outOfOrder >= x.r.replayed - 2u);
    CHECK(x.u().v.rmse() < 0.25);
}

TEST(nav_sim_26_timestamp_wraparound)
{
    Run w(26);
    Scenario n = scenario(26);
    n.t0Ms = 100000u;
    Runner r(n, kEvalSeed);
    r.run();
    CHECK(healthy(w.u()));
    CHECK(w.u().last.altitude_m == r.ukf.last.altitude_m);
    CHECK(w.u().last.verticalVelocity_mps == r.ukf.last.verticalVelocity_mps);
    CHECK(w.u().v.rmse() == r.ukf.v.rmse());
}

// ---------------------------------------------------------------------------
// 27-30: start-up, outages, recovery, long duration
// ---------------------------------------------------------------------------

TEST(nav_sim_27_startup_in_flight)
{
    Scenario s = scenario(27);
    Runner r(s, kEvalSeed);
    r.warmupS = 0.0;
    double firstValid = -1, vErrAtInit = 0, sigmaAtInit = 0;
    r.each = [&](double t, const NavigationInput&, const NavigationEstimate& u, const NavigationEstimate&,
                 const vsim::TruthPoint& tp) {
        if (firstValid < 0 && u.velocityValid) {
            firstValid = t;
            vErrAtInit = u.verticalVelocity_mps - tp.v;
            sigmaAtInit = u.velocitySigma_mps;
        }
    };
    r.run();
    print(27, s, r);
    std::printf("    [nav] S27 in-flight start: valid after %.2f s, initial vz error %+.2f m/s (sigma %.2f)\n", firstValid,
                vErrAtInit, sigmaAtInit);
    CHECK(healthy(r.ukf));
    CHECK(firstValid > 0 && firstValid < 1.0);
    CHECK(std::fabs(vErrAtInit) < 3.0 * sigmaAtInit);
}

TEST(nav_sim_28_complete_outage_unavailable_then_reinitialised)
{
    Scenario s = scenario(28);
    Runner r(s, kEvalSeed);
    bool unavailable = false;
    double backValid = -1;
    r.each = [&](double t, const NavigationInput&, const NavigationEstimate& u, const NavigationEstimate&,
                 const vsim::TruthPoint&) {
        unavailable = unavailable || u.state == NavState::Unavailable;
        if (t > 25.0 && backValid < 0 && u.altitudeValid && u.velocityValid) backValid = t - 25.0;
    };
    r.run();
    print(28, s, r);
    std::printf("    [nav] S28 5 s total outage: Unavailable reached, valid again %.2f s after data return\n", backValid);
    CHECK(healthy(r.ukf));
    CHECK(unavailable);
    CHECK(r.ukf.last.reinitializations == 1u);
    CHECK(backValid > 0 && backValid < 1.5);
    CHECK(r.ukf.last.state == NavState::Tracking);
}

TEST(nav_sim_29_recovery_after_barometer_loss)
{
    Scenario s = scenario(29);
    Runner r(s, kEvalSeed);
    double errAtReturn = 0, recovered = -1;
    bool sawRecovering = false;
    r.each = [&](double t, const NavigationInput&, const NavigationEstimate& u, const NavigationEstimate&,
                 const vsim::TruthPoint& tp) {
        if (t > 27.9 && t < 28.0) errAtReturn = u.altitude_m - tp.h;
        sawRecovering = sawRecovering || u.state == NavState::Recovering;
        if (t > 28.0 && recovered < 0 && std::fabs(u.altitude_m - tp.h) < 1.0 && u.state == NavState::Tracking)
            recovered = t - 28.0;
    };
    r.run();
    print(29, s, r);
    std::printf("    [nav] S29 8 s barometer loss with 0.1 m/s^2 bias: altitude error at return %.2f m, back < 1 m and "
                "Tracking after %.2f s\n", errAtReturn, recovered);
    CHECK(healthy(r.ukf));
    CHECK(sawRecovering);
    CHECK(recovered > 0 && recovered < 3.0);
}

TEST(nav_sim_30_long_duration_covariance_stability)
{
    Scenario s = scenario(30);
    Runner r(s, kEvalSeed);
    float sigH20 = 0, sigHEnd = 0, sigV20 = 0, sigVEnd = 0;
    r.each = [&](double t, const NavigationInput&, const NavigationEstimate& u, const NavigationEstimate&,
                 const vsim::TruthPoint&) {
        if (t > 19.99 && t < 20.0) { sigH20 = u.altitudeSigma_m; sigV20 = u.velocitySigma_mps; }
        sigHEnd = u.altitudeSigma_m;
        sigVEnd = u.velocitySigma_mps;
    };
    r.run();
    print(30, s, r);
    std::printf("    [nav] S30 30 min: sigma_h %.3f -> %.3f m, sigma_v %.3f -> %.3f m/s (t = 20 s -> end), max asymmetry "
                "%.1e, %u predictions\n", double(sigH20), double(sigHEnd), double(sigV20), double(sigVEnd),
                r.ukf.maxAsymmetry, unsigned(r.ukf.last.predictions));
    CHECK(healthy(r.ukf));
    CHECK(r.ukf.usable() > 0.999);
    CHECK(std::fabs(sigHEnd - sigH20) < 0.2f * sigH20);   // bounded, not growing or collapsing
    CHECK(std::fabs(sigVEnd - sigV20) < 0.3f * sigV20);
    CHECK(r.ukf.v.rmse() < 0.3);
}

// ---------------------------------------------------------------------------
// Uncertainty consistency: NEES / NIS / coverage over several seeds
// ---------------------------------------------------------------------------

namespace {

/// "Matched" truth: acceleration = integrated white jerk (the model's own assumption),
/// white sensor noise with exactly the configured sigmas, no bias, no correlated errors.
Scenario matched(uint64_t seed, double dur)
{
    const NavigationConfig c;
    Scenario s;
    s.name = "matched (white jerk truth, configured white noise)";
    s.durationS = dur;
    s.baroNoise = c.baroSigma_m * std::sqrt(2.0);   // two-barometer mean has sigma = baroSigma
    s.accelNoise = c.accelSigma_mps2;
    sim::Rng r(seed * 7919u + 13u);
    auto a = std::make_shared<std::vector<double>>();
    a->reserve(static_cast<size_t>(dur * 1000) + 2);
    double x = 0.0;
    for (size_t k = 0; k < static_cast<size_t>(dur * 1000) + 2; ++k) {
        a->push_back(x);
        x += std::sqrt(double(c.jerkPsd_m2ps5) * 0.001) * r.gaussian();
        x *= 0.9995;   // keeps the acceleration bounded over long runs (time constant 2 s)
    }
    s.accel = [a](double t) {
        const double k = t * 1000.0;
        const size_t i = std::min(static_cast<size_t>(k), a->size() - 2);
        const double f = k - double(i);
        return (*a)[i] * (1 - f) + (*a)[i + 1] * f;
    };
    return s;
}

struct Consistency {
    double nees = 0, in95 = 0, cov[3] = {0, 0, 0}, nis[3] = {0, 0, 0};
    int runs = 0;
    void add(const Metrics& m)
    {
        nees += m.nees();
        in95 += m.neesN ? double(m.neesIn95) / m.neesN : 0.0;
        for (int i = 0; i < 3; ++i) {
            cov[i] += std::isfinite(m.coverage(i)) ? m.coverage(i) : 0.0;
            nis[i] += std::isfinite(m.nis(i)) ? m.nis(i) : 0.0;
        }
        ++runs;
    }
    void print(const char* name, int dims) const
    {
        std::printf("    [nav] consistency %-52s: mean NEES %.2f (ideal %d), NEES<chi2_95 %.1f%%, 2-sigma coverage h %.1f%% "
                    "v %.1f%% a %.1f%%, mean NIS baro %.2f accel %.2f gnss %.2f (ideal 1), %d seeds\n",
                    name, nees / runs, dims, 100 * in95 / runs, 100 * cov[0] / runs, 100 * cov[1] / runs,
                    100 * cov[2] / runs, nis[0] / runs, nis[1] / runs, nis[2] / runs, runs);
    }
};

}  // namespace

TEST(nav_consistency_nees_nis_matched_and_realistic)
{
    // Held-out seeds 101-110 (the sweeps below use seeds 1-5).
    Consistency mt, mtBase, rl, rlBase, gn;
    for (uint64_t seed = 101; seed <= 110; ++seed) {
        Scenario m = matched(seed, 120.0);
        Runner rm(m, seed);
        rm.run();
        mt.add(rm.ukf);
        mtBase.add(rm.base);
        for (int id : {7, 9, 10}) {
            Runner rr(scenario(id), seed);
            rr.run();
            rl.add(rr.ukf);
            rlBase.add(rr.base);
        }
        Scenario g = scenario(21);
        g.gnssOutlierProb = 0.0;
        Runner rg(g, seed);
        rg.run();
        gn.add(rg.ukf);
    }
    mt.print("UKF, matched assumptions", 3);
    mtBase.print("baseline, matched assumptions (h, v)", 2);
    rl.print("UKF, realistic errors (S7, S9, S10)", 3);
    rlBase.print("baseline, realistic errors (S7, S9, S10)", 2);
    gn.print("UKF with GNSS velocity (S21, no outliers)", 3);
    // Matched: the filter must be approximately consistent (not over-confident).
    CHECK(mt.nees / mt.runs > 1.5 && mt.nees / mt.runs < 4.5);
    CHECK(mt.nis[0] / mt.runs > 0.6 && mt.nis[0] / mt.runs < 1.4);
    CHECK(mt.nis[1] / mt.runs > 0.6 && mt.nis[1] / mt.runs < 1.4);
}

// ---------------------------------------------------------------------------
// Limited, reproducible parameter sweeps on TUNING seeds 1-5
// ---------------------------------------------------------------------------

namespace {

struct SweepResult {
    double h = 0, v = 0, a = 0, nees = 0, usable = 0;
    unsigned rej = 0;
    int n = 0;
};

SweepResult sweep(const NavigationConfig& c, const BaroAlphaBetaConfig& bc = BaroAlphaBetaConfig{}, bool baseline = false)
{
    SweepResult s;
    for (uint64_t seed = 1; seed <= 5; ++seed) {
        for (int id : {4, 6, 7, 8, 9, 10, 17}) {
            Runner r(scenario(id), seed, c, bc);
            r.runBaseline = baseline;
            r.run();
            const Metrics& m = baseline ? r.base : r.ukf;
            s.h += m.h.rmse();
            s.v += m.v.rmse();
            s.a += baseline ? 0.0 : m.a.rmse();
            s.nees += m.nees();
            s.usable += m.usable();
            s.rej += m.last.baro.rejectedGate;
            ++s.n;
        }
    }
    s.h /= s.n;
    s.v /= s.n;
    s.a /= s.n;
    s.nees /= s.n;
    s.usable /= s.n;
    return s;
}

void printSweep(const char* label, double value, const SweepResult& s)
{
    std::printf("    [nav]   %-28s %8.3f : h rmse %.3f  v rmse %.3f  a rmse %.3f  mean NEES %.2f  usable %.1f%%  baro "
                "rejections %u\n", label, value, s.h, s.v, s.a, s.nees, 100 * s.usable, s.rej);
}

}  // namespace

TEST(nav_sweep_process_and_measurement_noise)
{
    std::printf("    [nav] sweep (tuning seeds 1-5; scenarios 4, 6, 7, 8, 9, 10, 17; mean of 35 runs)\n");
    for (float q : {0.1f, 0.3f, 1.0f, 3.0f, 10.0f}) {
        NavigationConfig c;
        c.jerkPsd_m2ps5 = q;
        printSweep("jerk PSD qj (m^2/s^5)", q, sweep(c));
    }
    for (float qa : {0.0f, 0.005f, 0.02f, 0.05f, 0.2f}) {
        NavigationConfig c;
        c.accelErrorPsd_m2ps3 = qa;
        printSweep("unmodelled accel qa (m^2/s^3)", qa, sweep(c));
    }
    for (float s : {0.15f, 0.35f, 0.7f}) {
        NavigationConfig c;
        c.accelSigma_mps2 = s;
        printSweep("accel sigma (m/s^2)", s, sweep(c));
    }
    for (float s : {0.25f, 0.5f, 0.7f, 1.0f}) {
        NavigationConfig c;
        c.baroSigma_m = s;
        printSweep("baro sigma (m)", s, sweep(c));
    }
    for (float g : {3.0f, 4.0f, 6.0f}) {
        NavigationConfig c;
        c.baroGateSigma = g;
        printSweep("baro gate (sigma)", g, sweep(c));
    }
    const SweepResult def = sweep(NavigationConfig{});
    printSweep("DEFAULT configuration", 0.0, def);
    CHECK(def.usable > 0.99);
}

TEST(nav_sweep_baseline_is_tuned_fairly)
{
    // The baseline gets the same treatment: its manoeuvre parameter is swept on the same
    // tuning runs and the best value is compared with the default.
    double best = 1e9, bestS = 0;
    for (float sa : {0.3f, 1.0f, 3.0f, 10.0f}) {
        BaroAlphaBetaConfig b;
        b.maneuverSigma_mps2 = sa;
        const SweepResult r = sweep(NavigationConfig{}, b, true);
        printSweep("BASELINE manoeuvre sigma_a", sa, r);
        if (r.v < best) {
            best = r.v;
            bestS = sa;
        }
    }
    std::printf("    [nav] best baseline sigma_a on tuning runs: %.1f m/s^2 (velocity rmse %.3f)\n", bestS, best);
    CHECK(bestS > 0.0);
}
