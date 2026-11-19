/**
 * Attitude milestone - AttitudeEstimator behaviour: initialisation state
 * machine, timestamp / dt handling, gaps, invalid input propagation, and the
 * magnetometer policy. Truth comes from sim/AttitudeSimulator.hpp.
 */
#include <cmath>
#include <functional>

#include "TestFramework.hpp"
#include "sim/AttitudeSimulator.hpp"

#include "Algorithms/Attitude/AttitudeEstimator.hpp"

using namespace pg;
using attsim::M3;
using attsim::V3;

namespace {

struct Runner {
    explicit Runner(const AttitudeConfig& cfg = AttitudeConfig{}, uint64_t seed = 3u) : est(cfg), h(seed) {}
    AttitudeEstimator est;
    attsim::AttitudeHarness h;

    /// Feeds samples at t = from, from+period, ... < to. @p mutate may edit or skip (return false) a sample.
    void run(uint32_t from, uint32_t to, uint32_t period = 10u,
             const std::function<bool(AttitudeInput&)>& mutate = nullptr,
             const std::function<void(uint32_t)>& each = nullptr)
    {
        for (uint32_t t = from; int32_t(to - t) > 0; t += period) {
            AttitudeInput in = h.sample(t);
            if (!mutate || mutate(in)) est.update(in);
            if (each) each(t);
        }
    }
    const AttitudeEstimate& e() const { return est.latest(); }
    double err() const { return attsim::quatErrorDeg(h.truth.R, e().q); }
    double tilt() const { return attsim::tiltErrorDeg(h.truth.R, e().q); }
};

bool isRejectionLike(CorrectionStatus s)
{
    return s == CorrectionStatus::RejectedMagnitude || s == CorrectionStatus::RejectedDirection ||
           s == CorrectionStatus::RejectedInclination;
}

/// Standard start: 1 s still (levels after 0.5 s).
void settle(Runner& r) { r.run(r.h.epochMs, r.h.epochMs + 1000u); }

}  // namespace

// ---------------------------------------------------------------------------
// Initialisation
// ---------------------------------------------------------------------------
TEST(att_est_uninitialized_without_calibrated_gyro_outputs_invalid)
{
    Runner r;
    r.run(1000u, 3000u, 10u, [](AttitudeInput& in) {
        in.gyroCalibrated = false;
        return true;
    });
    const AttitudeEstimate& e = r.e();
    CHECK(e.state == AttitudeState::Uninitialized);
    CHECK(!e.orientationValid);
    CHECK(e.has(attitude::NotInitialized) && e.has(attitude::GyroNotCalibrated));
    CHECK(std::isnan(e.q.w) && std::isnan(e.eulerDeg.roll));
    CHECK(!e.linearAccel.valid && std::isnan(e.linearAccel.vertical_mps2));
    CHECK_EQ(e.initializations, 0u);
}

TEST(att_est_levels_roll_pitch_from_gravity_yaw_unreferenced)
{
    Runner r;
    r.h.truth.R = attsim::fromNedEuler(30 * attsim::kDeg, -20 * attsim::kDeg, 0);
    r.h.err = attsim::realisticErrors();
    r.run(1000u, 1490u);
    CHECK(r.e().state == AttitudeState::Initializing);
    r.run(1490u, 1520u);
    CHECK(r.e().state == AttitudeState::Tracking6Axis);
    CHECK_EQ(r.e().initializations, 1u);
    CHECK(r.e().heading == HeadingReference::Unreferenced);
    CHECK(!r.e().yawReferenced && r.e().has(attitude::HeadingUnreferenced));
    CHECK(r.tilt() < 0.5);
    CHECK_NEAR(r.e().eulerDeg.roll, 30.0, 0.5);
    CHECK_NEAR(r.e().eulerDeg.pitch, -20.0, 0.5);
    CHECK_NEAR(r.e().eulerDeg.yaw, 0.0, 0.5);  // documented arbitrary start: pseudo-north
    std::printf("    [metric] levelling at roll 30 / pitch -20: tilt error %.3f deg\n", r.tilt());
}

TEST(att_est_motion_during_init_restarts_window)
{
    Runner r;
    r.h.truth.omegaBody = [](double t, const M3&) { return V3{t < 2.0 ? 0.5 : 0.0, 0, 0}; };
    uint32_t leveledAt = 0u;
    r.run(1000u, 4000u, 10u, nullptr, [&](uint32_t t) {
        if (leveledAt == 0u && r.e().orientationValid) leveledAt = t;
    });
    CHECK(leveledAt >= 1000u + 2000u + 500u);  // not before 0.5 s of stillness after the motion
    CHECK(r.tilt() < 0.5);
}

// ---------------------------------------------------------------------------
// Timestamps and dt
// ---------------------------------------------------------------------------
TEST(att_est_republished_input_not_integrated_twice)
{
    auto rate = [](double t, const M3&) { return V3{0, 0, attsim::smoothRate(t, 1.0, 2.0, attsim::kPi / 2)}; };
    Runner clean, dup;
    clean.h.truth.omegaBody = rate;
    dup.h.truth.omegaBody = rate;
    clean.run(1000u, 4000u);
    uint32_t repeats = 0u;
    for (uint32_t t = 1000u; t < 4000u; t += 10u) {
        const AttitudeInput in = dup.h.sample(t);
        dup.est.update(in);
        const Quaternion q1 = dup.e().q;
        AttitudeInput again = in;
        again.nowMs = t + 3u;  // the same frame republished 3 ms later (e.g. idle timeout)
        dup.est.update(again);
        if (dup.e().orientationValid) {
            CHECK(dup.e().gyroIntegration == GyroIntegration::NotNew);
            CHECK(dup.e().q.w == q1.w && dup.e().q.z == q1.z);
            ++repeats;
        }
    }
    CHECK(repeats > 200u);
    CHECK(dup.e().notNewSamples >= repeats);
    CHECK_NEAR(dup.e().q.w, clean.e().q.w, 1e-7);
    CHECK_NEAR(dup.e().q.z, clean.e().q.z, 1e-7);
    CHECK(dup.err() < 1.0);
}

TEST(att_est_out_of_order_sample_skipped)
{
    Runner r;
    settle(r);
    const AttitudeInput newer = r.h.sample(2000u);
    r.est.update(newer);
    const Quaternion q = r.e().q;
    AttitudeInput older = newer;
    older.gyroTimestampMs = 1995u;
    older.gyro_radps = Vec3f{3, 3, 3};
    r.est.update(older);
    CHECK(r.e().gyroIntegration == GyroIntegration::OutOfOrder);
    CHECK_EQ(r.e().outOfOrderSamples, 1u);
    CHECK(r.e().q.w == q.w && r.e().q.x == q.x);
}

TEST(att_est_skipped_samples_use_real_dt)
{
    // 90 deg yaw rotation; only every third gyro sample arrives (dt = 30 ms).
    Runner r;
    r.h.truth.omegaBody = [](double t, const M3&) { return V3{0, 0, attsim::smoothRate(t, 1.2, 2.0, attsim::kPi / 2)}; };
    settle(r);  // level at 100 Hz (levelling needs initMinSamples = 40 samples)
    uint32_t k = 0u;
    r.run(2000u, 4500u, 10u, [&](AttitudeInput&) { return (k++ % 3u) == 0u; });
    CHECK_EQ(r.e().dtMs, 30u);
    CHECK(r.e().state == AttitudeState::Tracking6Axis);
    CHECK_EQ(r.e().gaps, 0u);
    CHECK(r.err() < 2.0);
    std::printf("    [metric] 90 deg yaw with dt = 30 ms: final error %.3f deg\n", r.err());
}

TEST(att_est_moderate_gap_degrades_then_recovers)
{
    Runner r;
    settle(r);
    r.run(2000u, 2200u, 10u, [](AttitudeInput& in) {
        in.gyroValid = false;  // 200 ms gyro outage
        return true;
    });
    CHECK(r.e().state == AttitudeState::Degraded);
    CHECK(r.e().has(attitude::GapDetected));
    CHECK(r.e().orientationValid);  // held, not discarded
    r.run(2200u, 2210u);
    CHECK(r.e().gyroIntegration == GyroIntegration::GapSkipped);  // the 210 ms hole is not integrated
    CHECK(r.e().state == AttitudeState::Degraded);
    r.run(2210u, 2220u);
    CHECK(r.e().accelCorrection == CorrectionStatus::Accepted);
    CHECK(r.e().state == AttitudeState::Tracking6Axis);  // gravity agreed again -> confirmed
    CHECK(!r.e().has(attitude::GapDetected));
}

TEST(att_est_gap_skipped_is_not_integrated)
{
    Runner r;
    settle(r);
    // gyro samples stop for 120 ms while (truth) not rotating; the next one has a huge rate
    r.run(2000u, 2010u);
    AttitudeInput in = r.h.sample(2130u);
    in.gyro_radps = Vec3f{5, 0, 0};
    r.est.update(in);
    CHECK(r.e().gyroIntegration == GyroIntegration::GapSkipped);
    CHECK_EQ(r.e().dtMs, 130u);
    CHECK(r.tilt() < 0.5);  // 5 rad/s * 130 ms was NOT applied
    CHECK(r.e().state == AttitudeState::Degraded);
}

TEST(att_est_long_gap_invalidates_then_relevels_keeping_yaw)
{
    Runner r;
    r.h.truth.omegaBody = [](double t, const M3&) { return V3{0, 0, attsim::smoothRate(t, 1.0, 1.0, 60 * attsim::kDeg)}; };
    r.run(1000u, 3000u);
    CHECK_NEAR(r.e().eulerDeg.yaw, 60.0, 1.0);
    r.run(3000u, 3800u, 10u, [](AttitudeInput& in) {
        in.gyroValid = false;
        return true;
    });
    CHECK(r.e().state == AttitudeState::Invalid);
    CHECK(!r.e().orientationValid && r.e().has(attitude::NotInitialized));
    CHECK(!r.e().linearAccel.valid);
    CHECK_EQ(r.e().invalidations, 1u);
    r.run(3800u, 4400u);
    CHECK(r.e().orientationValid);
    CHECK_EQ(r.e().initializations, 2u);
    CHECK_NEAR(r.e().eulerDeg.yaw, 60.0, 1.0);  // previous yaw kept (still unreferenced)
    CHECK(r.e().heading == HeadingReference::Unreferenced);
}

TEST(att_est_timestamp_wraparound)
{
    Runner r;
    r.h.epochMs = 0xFFFFFFFFu - 2995u;  // wrap ~3 s after start, during the rotation
    r.h.truth.omegaBody = [](double t, const M3&) {
        return V3{attsim::smoothRate(t, 1.5, 3.0, attsim::kPi / 2), 0, attsim::smoothRate(t, 1.5, 3.0, 1.0)};
    };
    const uint32_t start = r.h.epochMs;
    r.run(start, start + 6000u);
    CHECK(r.e().timestampMs < 6000u);  // wrapped
    CHECK_EQ(r.e().gaps, 0u);
    CHECK_EQ(r.e().outOfOrderSamples, 0u);
    CHECK(r.err() < 1.0);
    std::printf("    [metric] rotation across uint32 ms wrap: final error %.3f deg\n", r.err());
}

TEST(att_est_invalid_config_disables_estimator)
{
    AttitudeConfig cfg;
    cfg.beta_radps = std::nanf("");
    Runner r(cfg);
    settle(r);
    CHECK(!r.est.configValid());
    CHECK(!r.e().orientationValid);
    CHECK(r.e().has(attitude::InvalidConfig) && r.e().has(attitude::NotInitialized));
    AttitudeConfig cfg2;
    cfg2.maxGapMs = 10u;  // smaller than maxIntegrationDtMs: inconsistent
    CHECK(!isValid(cfg2));
}

TEST(att_est_nonfinite_or_stale_gyro_is_unavailable_never_nan)
{
    Runner r;
    settle(r);
    AttitudeInput in = r.h.sample(2000u);
    in.gyro_radps.y = std::nanf("");
    r.est.update(in);
    CHECK(r.e().gyroIntegration == GyroIntegration::Unavailable);
    CHECK(attsim::finite(r.e().q));
    CHECK(r.e().has(attitude::GyroUnavailable));
    AttitudeInput st = r.h.sample(2010u);
    st.nowMs = 2010u + 40u;  // 40 ms old > gyroMaxAgeMs 30
    r.est.update(st);
    CHECK(r.e().gyroIntegration == GyroIntegration::Unavailable);
    CHECK(attsim::finite(r.e().q));
}

TEST(att_est_accel_invalid_propagates_and_degrades)
{
    Runner r;
    settle(r);
    r.run(2000u, 4500u, 10u, [](AttitudeInput& in) {
        in.accelValid = false;
        return true;
    });
    const AttitudeEstimate& e = r.e();
    CHECK(e.accelCorrection == CorrectionStatus::Unavailable);
    CHECK(!e.linearAccel.valid);
    CHECK(e.has(attitude::AccelUnavailable) && e.has(attitude::AccelRejectedProlonged));
    CHECK(e.state == AttitudeState::Degraded);
    CHECK(e.orientationValid);  // gyro-only propagation continues
    r.run(4500u, 4520u);
    CHECK(r.e().state == AttitudeState::Tracking6Axis);
    CHECK(r.e().linearAccel.valid && r.e().linearAccel.reliable);
}

TEST(att_est_inherited_quality_flags)
{
    Runner r;
    settle(r);
    AttitudeInput in = r.h.sample(2000u);
    in.mountingVerified = false;
    in.accelCalibrated = false;
    in.gyroStatus = RedundancyStatus::Degraded;
    r.est.update(in);
    CHECK(r.e().has(attitude::MountingUnverified));
    CHECK(r.e().has(attitude::AccelNotCalibrated));
    CHECK(r.e().has(attitude::ImuRedundancyDegraded));
    CHECK_EQ(r.e().gyroSourceMask, 0x7);
    CHECK(r.e().convention == RotationConvention::EnuFromFrdBody);
}

// ---------------------------------------------------------------------------
// Magnetometer policy
// ---------------------------------------------------------------------------
namespace {
AttitudeConfig magCfg(MagPolicy p = MagPolicy::CalibratedOnly)
{
    AttitudeConfig c;
    c.magPolicy = p;
    return c;
}
/// Horizontal rotation of the field by @p deg (same strength and dip): a "direction" disturbance.
V3 rotatedFieldDelta(const V3& m, double deg)
{
    const double c = std::cos(deg * attsim::kDeg), s = std::sin(deg * attsim::kDeg);
    return V3{c * m.x - s * m.y - m.x, s * m.x + c * m.y - m.y, 0.0};
}
}  // namespace

TEST(att_mag_unavailable_keeps_6axis_unreferenced)
{
    Runner r(magCfg());
    r.h.err = attsim::realisticErrors();
    r.run(1000u, 61000u);
    CHECK(r.e().state == AttitudeState::Tracking6Axis);
    CHECK(r.e().heading == HeadingReference::Unreferenced);
    CHECK(r.e().magCorrection == CorrectionStatus::Unavailable);
    CHECK(r.tilt() < 1.0);
}

TEST(att_mag_calibrated_field_acquires_absolute_heading)
{
    Runner r(magCfg());
    r.h.err = attsim::realisticErrors();
    r.h.truth.R = attsim::fromNedEuler(0, 0, 70 * attsim::kDeg);
    r.h.magPresent = true;
    r.run(1000u, 1600u);
    CHECK(r.e().heading == HeadingReference::Unreferenced);  // levelled, acquiring
    r.run(1600u, 3000u);
    CHECK_EQ(r.e().headingAcquisitions, 1u);
    CHECK(r.e().state == AttitudeState::Tracking9Axis);
    CHECK(r.e().heading == HeadingReference::MagneticReferenced);
    CHECK(r.e().yawReferenced && !r.e().has(attitude::HeadingUnreferenced));
    CHECK(std::fabs(attsim::headingErrorDeg(r.h.truth.R, r.e().q)) < 2.0);
    CHECK(r.err() < 2.0);
    std::printf("    [metric] heading acquisition (truth 70 deg): error %.3f deg\n",
                attsim::headingErrorDeg(r.h.truth.R, r.e().q));
}

TEST(att_mag_uncalibrated_policy_disabled_or_unvalidated)
{
    Runner def(magCfg());
    def.h.magPresent = true;
    def.h.magCalibrated = false;
    def.run(1000u, 3000u);
    CHECK(def.e().magCorrection == CorrectionStatus::Disabled);
    CHECK(def.e().state == AttitudeState::Tracking6Axis);
    CHECK(def.e().heading == HeadingReference::Unreferenced);

    Runner exp(magCfg(MagPolicy::AllowUncalibrated));
    exp.h.magPresent = true;
    exp.h.magCalibrated = false;
    exp.run(1000u, 3000u);
    CHECK(exp.e().heading == HeadingReference::MagneticUnvalidated);
    CHECK(!exp.e().yawReferenced);
    CHECK(exp.e().has(attitude::MagUncalibrated) && exp.e().has(attitude::HeadingUnreferenced));

    Runner off(magCfg(MagPolicy::Disabled));
    off.h.magPresent = true;
    off.run(1000u, 3000u);
    CHECK(off.e().magCorrection == CorrectionStatus::Disabled);
    CHECK(off.e().heading == HeadingReference::Unreferenced);
}

TEST(att_mag_wrong_magnitude_rejected)
{
    Runner r(magCfg());
    r.h.magPresent = true;
    r.run(1000u, 3000u);
    CHECK(r.e().heading == HeadingReference::MagneticReferenced);
    const V3 m = r.h.truth.magEnu;
    r.h.magDisturbanceEnu = [m](double) { return V3{0.5 * m.x, 0.5 * m.y, 0.5 * m.z}; };  // x1.5 strength
    r.run(3000u, 4000u);
    CHECK(r.e().magCorrection == CorrectionStatus::RejectedMagnitude);
    CHECK(r.e().has(attitude::MagRejected));
    CHECK(std::fabs(attsim::headingErrorDeg(r.h.truth.R, r.e().q)) < 1.0);
    r.h.magDisturbanceEnu = [](double) { return V3{0, 0, 0}; };
    r.h.magDisturbanceEnu = [m](double) { return V3{2.0 * m.x, 2.0 * m.y, 2.0 * m.z}; };  // 147 uT: implausible
    r.run(4000u, 4100u);
    CHECK(r.e().magCorrection == CorrectionStatus::RejectedMagnitude);
}

TEST(att_mag_inclination_change_rejected)
{
    Runner r(magCfg());
    r.h.magPresent = true;
    r.run(1000u, 3000u);
    // same strength, dip changed by ~20 deg (field tilted in the north-up plane)
    const V3 m = r.h.truth.magEnu;
    const double a = 20 * attsim::kDeg;
    const V3 tilted{0, std::cos(a) * m.y - std::sin(a) * m.z, std::sin(a) * m.y + std::cos(a) * m.z};
    r.h.magDisturbanceEnu = [m, tilted](double) { return V3{0, tilted.y - m.y, tilted.z - m.z}; };
    r.run(3000u, 3500u);
    CHECK(r.e().magCorrection == CorrectionStatus::RejectedInclination);
    CHECK(std::fabs(attsim::headingErrorDeg(r.h.truth.R, r.e().q)) < 1.0);
}

TEST(att_mag_direction_disturbance_rejected_then_persistent_one_accepted)
{
    Runner r(magCfg());
    r.h.err = attsim::realisticErrors();
    r.h.magPresent = true;
    r.run(1000u, 3000u);
    const V3 m = r.h.truth.magEnu;
    // 3 s disturbance: field direction rotated 60 deg, strength and dip unchanged
    r.h.magDisturbanceEnu = [m](double) { return rotatedFieldDelta(m, 60); };
    attsim::Stats hd;
    r.run(3000u, 6000u, 10u, nullptr, [&](uint32_t) { hd.add(attsim::headingErrorDeg(r.h.truth.R, r.e().q)); });
    CHECK(r.e().magCorrection == CorrectionStatus::RejectedDirection);
    CHECK(hd.max < 1.0);
    std::printf("    [metric] 3 s 60-deg magnetic disturbance: max heading error %.3f deg\n", hd.max);
    r.h.magDisturbanceEnu = [](double) { return V3{0, 0, 0}; };
    r.run(6000u, 6100u);
    CHECK(r.e().magCorrection == CorrectionStatus::Accepted);
    // A PERSISTENT self-consistent disturbance is eventually accepted (documented trade-off).
    r.h.magDisturbanceEnu = [m](double) { return rotatedFieldDelta(m, 60); };
    bool recovered = false;
    r.run(6100u, 13000u, 10u, nullptr, [&](uint32_t) {
        recovered = recovered || r.e().magCorrection == CorrectionStatus::AcceptedRecovery;
    });
    CHECK(recovered);
    std::printf("    [metric] persistent 60-deg disturbance: heading pulled to %.1f deg error after 7 s\n",
                attsim::headingErrorDeg(r.h.truth.R, r.e().q));
}

TEST(att_mag_single_corrupted_sample_causes_no_jump)
{
    Runner r(magCfg());
    r.h.magPresent = true;
    r.run(1000u, 3000u);
    const Quaternion before = r.e().q;
    AttitudeInput in = r.h.sample(3000u);
    in.mag_uT = Vec3f{-40, 30, 10};  // plausible strength, wildly wrong direction
    in.magTimestampMs = 3000u;
    r.est.update(in);
    CHECK(isRejectionLike(r.e().magCorrection));
    CHECK(quat::angleBetween(before, r.e().q) * quat::kDegPerRad < 0.2f);
}

TEST(att_mag_heading_lost_after_hold_time_without_magnetometer)
{
    Runner r(magCfg());
    r.h.magPresent = true;
    r.run(1000u, 3000u);
    CHECK(r.e().heading == HeadingReference::MagneticReferenced);
    r.run(3000u, 14000u, 10u, [](AttitudeInput& in) {
        in.magValid = false;
        return true;
    });
    CHECK(r.e().heading == HeadingReference::MagneticLost);
    CHECK(r.e().state == AttitudeState::Tracking6Axis);
    CHECK(!r.e().yawReferenced);
}
