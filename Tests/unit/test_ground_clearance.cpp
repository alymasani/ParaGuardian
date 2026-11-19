/**
 * Milestone 5 - GroundClearanceEstimator: LiDAR geometry against INDEPENDENT
 * truth (analytic ray-plane intersection and ray marching in double, beam
 * rotated by the simulator's own Euler matrices), quality and failure
 * handling, closure rate, time to contact (LiDAR scenarios 16-30).
 */
#include <cmath>
#include <cstdio>

#include "TestFramework.hpp"
#include "sim/FlightSafetySimulator.hpp"

#include "Algorithms/Landing/GroundClearanceEstimator.hpp"

using namespace pg;
using fsim::V3;

namespace {

constexpr double kDeg = fsim::kDeg;

/// Analytic intersection of P + s D with the plane z = a x + b y + c (ENU). Returns s (or -1).
double rayPlane(const V3& P, const V3& D, double a, double b, double c)
{
    const double den = D.z - a * D.x - b * D.y;
    if (std::fabs(den) < 1e-12) return -1.0;
    const double s = (a * P.x + b * P.y + c - P.z) / den;
    return s > 0 ? s : -1.0;
}

/// One SafetyInput with a LiDAR sample and an attitude at time t (ms).
SafetyInput input(uint32_t t, float range, double rollDeg, double pitchDeg, double yawDeg = 0.0, bool lidarValid = true)
{
    SafetyInput in{};
    in.nowMs = t + 1u;
    in.lidar.valid = lidarValid;
    in.lidar.distance_m = range;
    in.lidar.timestampMs = t;
    in.lidar.signalStrength = 1000u;
    in.attitude.orientationValid = true;
    in.attitude.state = AttitudeState::Tracking6Axis;
    in.attitude.timestampMs = t;
    in.attitude.q = quat::fromEulerNedZyx(float(rollDeg * kDeg), float(pitchDeg * kDeg), float(yawDeg * kDeg));
    return in;
}

}  // namespace

TEST(gc_16_level_flat_terrain_equals_range)
{
    GroundClearanceEstimator g;
    const GroundClearance& c = g.update(input(1000u, 12.5f, 0, 0));
    CHECK(c.status == ClearanceStatus::Valid && c.clearanceValid);
    CHECK_NEAR(c.clearance_m, 12.5, 1e-5);
    CHECK_NEAR(c.projectionFactor, 1.0, 1e-6);
    CHECK_NEAR(c.tiltDeg, 0.0, 0.05);
    CHECK(c.flags & clearflag::MountingUnverified);   // default mounting is a placeholder
    CHECK(c.flags & clearflag::FlatTerrainAssumed);
    CHECK(c.clearanceSigma_m > 0.06f && c.clearanceSigma_m < 0.2f);
}

TEST(gc_17_21_slant_range_conversion_matches_independent_ray_plane_truth)
{
    // Random attitudes up to the tilt limit over FLAT terrain: the projected clearance must equal
    // the true height above the plane (independent analytic ray-plane solution, double precision).
    sim::Rng r(5);
    double worst = 0, worstRel = 0;
    int used = 0, rejected = 0;
    for (int k = 0; k < 2000; ++k) {
        const double roll = (r.uniform() * 2 - 1) * 55, pitch = (r.uniform() * 2 - 1) * 55, yaw = r.uniform() * 360;
        const double h = 2 + r.uniform() * 30;
        const V3 D = fsim::beamEnuTruth(V3{0, 0, 1}, roll * kDeg, pitch * kDeg, yaw * kDeg);
        const double s = rayPlane(V3{3, -2, h}, D, 0, 0, 0);
        GroundClearanceEstimator g;
        const GroundClearance& c = g.update(input(1000u, s > 0 ? float(s) : 1.0f, roll, pitch, yaw));
        const double trueTilt = std::acos(-D.z) / kDeg;
        if (s < 0 || s > 40 || trueTilt > 40.0) {
            CHECK(!c.clearanceValid && std::isnan(c.clearance_m));   // never a manufactured clearance
            if (trueTilt > 40.01 && s > 0.1 && s <= 40.0) CHECK(c.status == ClearanceStatus::BeamGeometry);
            ++rejected;
            continue;
        }
        if (trueTilt > 39.99) continue;   // boundary: float rounding decides
        CHECK(c.clearanceValid);
        worst = std::fmax(worst, std::fabs(c.clearance_m - h));
        worstRel = std::fmax(worstRel, std::fabs(c.clearance_m - h) / h);
        ++used;
    }
    std::printf("    [safety] LiDAR flat-terrain projection vs analytic ray-plane truth: %d attitudes used, %d rejected "
                "(tilt > 40 deg or no hit), max |error| %.2e m (%.1e relative)\n", used, rejected, worst, worstRel);
    CHECK(used > 500 && rejected > 500);
    CHECK(worstRel < 2e-5);
}

TEST(gc_17_18_19_bank_and_pitch_cases)
{
    struct Case { double roll, pitch; bool valid; const char* name; };
    const Case cases[] = {{30, 0, true, "30-deg bank"}, {-30, 10, true, "30-deg bank + 10 pitch"},
                          {60, 0, false, "60-deg bank"}, {0, 35, true, "35-deg pitch"}, {0, 45, false, "45-deg pitch"}};
    for (const Case& k : cases) {
        const double h = 10.0;
        const V3 D = fsim::beamEnuTruth(V3{0, 0, 1}, k.roll * kDeg, k.pitch * kDeg, 0);
        const double s = rayPlane(V3{0, 0, h}, D, 0, 0, 0);
        GroundClearanceEstimator g;
        const GroundClearance& c = g.update(input(1000u, float(s), k.roll, k.pitch));
        std::printf("    [safety] %-22s slant %.2f m -> clearance %s %.3f m (truth %.1f), tilt %.1f deg\n", k.name, s,
                    c.clearanceValid ? "VALID" : "REJECTED", c.clearanceValid ? double(c.clearance_m) : NAN, h, double(c.tiltDeg));
        CHECK(c.clearanceValid == k.valid);
        if (k.valid) {
            CHECK_NEAR(c.clearance_m, h, 1e-3);
            CHECK(c.slantRange_m > c.clearance_m);       // slant range is NOT the clearance
            if (std::fabs(k.roll) >= 30) CHECK(c.flags & clearflag::HighTilt);
        } else {
            CHECK(c.status == ClearanceStatus::BeamGeometry);
        }
    }
}

TEST(gc_20_beam_horizontal_or_upward_never_produces_a_clearance)
{
    for (double pitch : {80.0, 89.0, 95.0, 135.0, 180.0}) {
        GroundClearanceEstimator g;
        const GroundClearance& c = g.update(input(1000u, 25.0f, 0, pitch));
        CHECK(!c.clearanceValid && std::isnan(c.clearance_m) && c.status == ClearanceStatus::BeamGeometry);
    }
    GroundClearanceEstimator g;
    CHECK(!g.update(input(1000u, 25.0f, 180, 0)).clearanceValid);   // upside down: beam points up
}

TEST(gc_22_sloped_terrain_bias_is_quantified_not_hidden)
{
    // On a slope the projection gives the vertical distance to the ILLUMINATED point, not the
    // clearance beneath. Quantify both against independent truth.
    for (double slopeDeg : {10.0, 20.0, 30.0}) {
        for (double bankDeg : {0.0, 20.0, -20.0}) {
            const double a = std::tan(slopeDeg * kDeg);   // terrain rises towards +x (east)
            const double h = 15.0;
            const V3 D = fsim::beamEnuTruth(V3{0, 0, 1}, bankDeg * kDeg, 0, 0);   // bank -> beam moves east/west
            const double s = rayPlane(V3{0, 0, h}, D, a, 0, 0);
            GroundClearanceEstimator g;
            const GroundClearance& c = g.update(input(1000u, float(s), bankDeg, 0));
            const double hitZ = a * (s * D.x);
            const double illum = h - hitZ;
            CHECK(c.clearanceValid);
            CHECK_NEAR(c.clearance_m, illum, 1e-3);   // exactly the illuminated-point distance
            std::printf("    [safety] slope %4.1f deg, bank %+5.1f deg: estimate %.2f m, illuminated-point truth %.2f, "
                        "truth beneath %.2f (error vs beneath %+.2f m)\n",
                        slopeDeg, bankDeg, double(c.clearance_m), illum, h, double(c.clearance_m) - h);
            if (bankDeg == 0.0) CHECK_NEAR(c.clearance_m, h, 1e-3);   // vertical beam: no slope error
        }
    }
}

TEST(gc_23_terrain_rising_closure_rate_vs_truth)
{
    // Level flight (vz = 0) at 9 m/s over terrain rising 1:5 -> true closure = 1.8 m/s although UKF vz = 0.
    fsim::Scenario sc;
    sc.name = "terrain rising";
    sc.durationS = 8;
    sc.h0 = 30;
    sc.groundSpeed = [](double) { return 9.0; };
    sc.terrain = [](double x, double) { return x > 5 ? 0.2 * (x - 5) : 0.0; };
    sc.gnss = false;
    fsim::Runner r(sc, 3);
    double rateAt5 = NAN, ttcAt5 = NAN, clrAt5 = NAN, sigSum = 0;
    int sigN = 0;
    r.each = [&](double t, const SafetyInput&, const SafetySnapshot& s, const fsim::Truth&) {
        const GroundClearance& g = r.monitor().clearance();
        if (g.closureRateValid && t > 1.5) {
            sigSum += g.closureRateSigma_mps;
            ++sigN;
        }
        if (std::isnan(rateAt5) && t >= 5.0) {
            rateAt5 = s.closureRate_mps;
            ttcAt5 = s.timeToContact_s;
            clrAt5 = s.clearance_m;
        }
    };
    r.run();
    const double meanSig = sigSum / sigN;
    std::printf("    [safety] S23 rising terrain at 9 m/s (true closure 1.8 m/s, UKF vz 0): estimate %.2f m/s, clearance "
                "%.1f m, TTC %.1f s; rate error rms %.3f / p95 %.3f / max %.3f m/s (n %zu), mean reported sigma %.3f m/s\n",
                rateAt5, clrAt5, ttcAt5, r.rateErr.rms(), r.rateErr.p95(), r.rateErr.maxAbs(), r.rateErr.n(), meanSig);
    CHECK(std::fabs(rateAt5 - 1.8) < 3.0 * meanSig);
    CHECK(std::fabs(ttcAt5 - clrAt5 / rateAt5) < 1e-3);
    // Range noise is 1 % (0.2-0.3 m here): a 400 ms window gives ~0.3-0.4 m/s rate noise. The
    // reported sigma must describe it (not be over-confident).
    CHECK(r.rateErr.rms() < 1.5 * meanSig);
}

TEST(gc_24_25_sudden_terrain_drop_and_obstacle_are_kept_and_flagged)
{
    // A 12 m cliff drop and a 4 m vegetation patch: the new clearance is reported at once
    // (no over-smoothing), the step is flagged and the closure-rate history restarts.
    fsim::Scenario sc;
    sc.durationS = 6;
    sc.h0 = 20;
    sc.groundSpeed = [](double) { return 10.0; };
    sc.terrain = [](double x, double) {
        double z = x > 20 ? -12.0 : 0.0;          // cliff at x = 20 m (t = 2 s)
        if (x > 40 && x < 45) z += 4.0;           // bushes at x = 40-45 m (t = 4 s)
        return z;
    };
    sc.gnss = false;
    fsim::Runner r(sc, 4);
    double clrAfterDrop = NAN, clrOnBush = NAN;
    uint32_t steps = 0;
    double maxRate = 0;
    r.each = [&](double t, const SafetyInput&, const SafetySnapshot& s, const fsim::Truth&) {
        if (s.closureRateValid) maxRate = std::fmax(maxRate, std::fabs(s.closureRate_mps));
        if (std::isnan(clrAfterDrop) && t >= 2.05) clrAfterDrop = s.clearance_m;
        if (std::isnan(clrOnBush) && t >= 4.25) clrOnBush = s.clearance_m;
        steps = r.monitor().clearance().stepChanges;
    };
    r.run();
    std::printf("    [safety] S24/25 cliff -12 m: clearance 50 ms after %.2f m (truth 32); bushes +4 m: %.2f m (truth 28 at "
                "x=42.5); %u step flags; largest closure rate reported %.2f m/s (truth 0: level flight over flat segments)\n",
                clrAfterDrop, clrOnBush, steps, maxRate);
    CHECK(std::fabs(clrAfterDrop - 32.0) < 0.6);
    CHECK(std::fabs(clrOnBush - 28.0) < 0.8);
    CHECK(steps >= 2u);          // cliff and bush entry (the 4 m exit at 32 m sits at the 10 % threshold: noise decides)
    CHECK(maxRate < 1.5);        // no fake closure rate across any step (rate history reset / residual check)
    // 1 % range noise at ~32 m = 0.32 m; ~600 samples -> max about 3-4 sigma
    CHECK(r.clearanceErrIlluminated.maxAbs() < 1.4);
    CHECK(r.clearanceErrIlluminated.rms() < 0.4);
}

TEST(gc_26_27_invalid_return_out_of_range_and_weak_signal)
{
    GroundClearanceEstimator g;
    g.update(input(1000u, 20.0f, 0, 0));
    // Out of range (no return): preprocessing keeps the old accepted sample, which goes stale;
    // the rejection reason says why.
    SafetyInput in = input(1070u, 20.0f, 0, 0, 0, false);
    in.lidar.timestampMs = 1000u;
    in.lidar.lastRejectReasons = reject::LidarOutOfRange;
    in.lidar.lastRejectTimestampMs = 1070u;
    const GroundClearance& c = g.update(in);
    CHECK(c.status == ClearanceStatus::BeyondRange && !c.clearanceValid);
    CHECK_NEAR(c.lowerBound_m, 40.0, 1e-3);   // level: at least the 40 m range
    in.attitude.q = quat::fromEulerNedZyx(float(30 * kDeg), 0, 0);
    CHECK_NEAR(g.update(in).lowerBound_m, 40.0 * std::cos(30 * kDeg), 1e-3);
    // Weak / saturated signal: no number at all
    in.lidar.lastRejectReasons = reject::LidarWeakSignal;
    in.lidar.lastRejectTimestampMs = 1080u;
    in.nowMs = 1081u;
    CHECK(g.update(in).status == ClearanceStatus::WeakSignal);
    CHECK(std::isnan(g.latest().clearance_m) && std::isnan(g.latest().lowerBound_m));
    // Never a valid sample, never a rejection: NoLidar (never "0 m")
    GroundClearanceEstimator g2;
    SafetyInput none{};
    none.nowMs = 5000u;
    CHECK(g2.update(none).status == ClearanceStatus::NoLidar && std::isnan(g2.latest().clearance_m));
    // Out-of-range value inside an "accepted" sample is still refused
    GroundClearanceEstimator g3;
    CHECK(g3.update(input(1000u, 55.0f, 0, 0)).status == ClearanceStatus::InvalidMeasurement);
}

TEST(gc_28_dropout_duplicates_and_staleness)
{
    GroundClearanceEstimator g;
    g.update(input(1000u, 10.0f, 0, 0));
    // Same sample republished in the next frames: carried, not re-processed, ages, then stale.
    SafetyInput in = input(1000u, 10.0f, 0, 0);
    for (uint32_t k = 1; k <= 5; ++k) {
        in.nowMs = 1001u + 10u * k;
        const GroundClearance& c = g.update(in);
        CHECK(c.flags & clearflag::NotNew);
        CHECK(c.clearanceValid);
    }
    CHECK_EQ(g.latest().samples, 1u);
    CHECK_EQ(g.latest().duplicates, 5u);
    in.nowMs = 1070u;   // older than maxAgeMs (60)
    CHECK(g.update(in).status == ClearanceStatus::Stale && !g.latest().clearanceValid);
    // Preprocessing marks it invalid after 50 ms; with no rejection info: Stale
    in.lidar.valid = false;
    in.nowMs = 1200u;
    CHECK(g.update(in).status == ClearanceStatus::Stale);
}

TEST(gc_29_timestamp_wraparound)
{
    // A steady 2 m/s descent across the uint32 wrap: identical clearance / rate to normal times.
    auto run = [](uint32_t t0) {
        GroundClearanceEstimator g;
        for (uint32_t k = 0; k < 100u; ++k) g.update(input(t0 + 10u * k, 30.0f - 0.02f * float(k), 0, 0));
        return g.latest();
    };
    const GroundClearance a = run(5000u), b = run(0xFFFFFFFFu - 500u);
    CHECK(a.closureRateValid && b.closureRateValid);
    CHECK(a.clearance_m == b.clearance_m && a.closureRate_mps == b.closureRate_mps);
    CHECK_NEAR(a.closureRate_mps, 2.0, 1e-3);
    CHECK(a.ttcValid);
    CHECK_NEAR(a.timeToContact_s, a.clearance_m / 2.0, 1e-3);
}

TEST(gc_30_mounting_incorrect_or_unverified)
{
    // The TRUE beam is tilted 10 deg forward but the configuration says "straight down".
    // Error is quantified; outputs stay flagged MountingUnverified.
    double worst = 0;
    for (double pitch : {-20.0, 0.0, 20.0}) {
        const double h = 10;
        const V3 b{std::sin(10 * kDeg), 0, std::cos(10 * kDeg)};
        const V3 D = fsim::beamEnuTruth(b, 0, pitch * kDeg, 0);
        const double s = rayPlane(V3{0, 0, h}, D, 0, 0, 0);
        GroundClearanceEstimator g;
        const GroundClearance& c = g.update(input(1000u, float(s), 0, pitch));
        CHECK(c.flags & clearflag::MountingUnverified);
        if (c.clearanceValid) worst = std::fmax(worst, std::fabs(c.clearance_m - h));
    }
    std::printf("    [safety] beam actually 10 deg off the configured axis: clearance error up to %.2f m at 10 m\n", worst);
    CHECK(worst > 0.1);   // a wrong mounting IS visible as error: it must be verified on hardware
    // With the correct (configured) mounting the same geometry is exact and can be marked verified.
    GroundClearanceConfig cfg;
    cfg.beamBody = Vec3f{std::sin(float(10 * kDeg)), 0.0f, std::cos(float(10 * kDeg))};
    cfg.mountingVerified = true;
    GroundClearanceEstimator g(cfg);
    const V3 D = fsim::beamEnuTruth(V3{std::sin(10 * kDeg), 0, std::cos(10 * kDeg)}, 0, 0, 0);
    const GroundClearance& c = g.update(input(1000u, float(rayPlane(V3{0, 0, 10}, D, 0, 0, 0)), 0, 0));
    CHECK_NEAR(c.clearance_m, 10.0, 1e-3);
    CHECK((c.flags & clearflag::MountingUnverified) == 0u);
    CHECK(!GroundClearanceEstimator(GroundClearanceConfig{Vec3f{0, 0, 0}}).configValid());
}

TEST(gc_attitude_invalid_stale_or_degraded)
{
    GroundClearanceEstimator g;
    SafetyInput in = input(1000u, 10.0f, 0, 0);
    in.attitude.orientationValid = false;
    CHECK(g.update(in).status == ClearanceStatus::AttitudeInvalid);
    in = input(1010u, 10.0f, 0, 0);
    in.attitude.qualityFlags = attitude::GapDetected;
    CHECK(g.update(in).status == ClearanceStatus::AttitudeInvalid);
    in = input(1020u, 10.0f, 0, 0);
    in.attitude.timestampMs = 950u;   // orientation 70 ms older than the LiDAR sample
    CHECK(g.update(in).status == ClearanceStatus::AttitudeStale);
    CHECK_EQ(g.latest().rejectedAttitude, 3u);
    in = input(1030u, 10.0f, 15, 0);
    const float sigOk = g.update(in).clearanceSigma_m;
    in = input(1040u, 10.0f, 15, 0);
    in.attitude.state = AttitudeState::Degraded;
    const GroundClearance& c = g.update(in);
    CHECK(c.clearanceValid && (c.flags & clearflag::AttitudeDegraded));
    CHECK(c.clearanceSigma_m > sigOk);   // degraded attitude -> larger uncertainty
}

TEST(gc_closure_rate_needs_history_and_rejects_spikes_ttc_rules)
{
    GroundClearanceEstimator g;
    // One sample: no rate, no TTC.
    CHECK(!g.update(input(1000u, 20.0f, 0, 0)).closureRateValid);
    CHECK(!g.latest().ttcValid && std::isnan(g.latest().timeToContact_s));
    // Climbing away (clearance increasing): rate negative, no finite TTC.
    for (uint32_t k = 1; k < 40u; ++k) g.update(input(1000u + 10u * k, 20.0f + 0.03f * float(k), 0, 0));
    CHECK(g.latest().closureRateValid && g.latest().closureRate_mps < -2.5f);
    CHECK(!g.latest().ttcValid);
    // A single 6 m spike: flagged, rate restarted (no fake closure from the spike).
    GroundClearanceEstimator h;
    for (uint32_t k = 0; k < 40u; ++k) h.update(input(1000u + 10u * k, 20.0f, 0, 0));
    h.update(input(1400u, 14.0f, 0, 0));
    CHECK(h.latest().flags & clearflag::StepChange);
    CHECK(!h.latest().closureRateValid);
    h.update(input(1410u, 20.0f, 0, 0));
    CHECK(!h.latest().closureRateValid);
    CHECK(!h.latest().ttcValid);
}
