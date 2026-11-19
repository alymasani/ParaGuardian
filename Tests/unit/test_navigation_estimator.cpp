/**
 * Vertical UKF milestone - estimator behaviour (VerticalNavigationEstimator):
 * configuration, initialisation, sample identity (each sample once), timing
 * (duplicates, out-of-order, late, wrap-around), measurement quality rules,
 * reference changes, outages, numerical faults, and the process-model
 * replacement boundary (second, nonlinear mock model; N = 4 engine).
 */
#include <cmath>
#include <cstdio>
#include <cstring>

#include "TestFramework.hpp"
#include "sim/VerticalSimulator.hpp"

#include "Algorithms/Navigation/NavigationInputAdapter.hpp"

using namespace pg;

namespace {

/// Hand-made inputs: two barometers (20 ms, phases 0 / 10 ms), acceleration every 10 ms.
struct Hand {
    uint32_t t0 = 5000u;
    float h = 100.0f;      ///< current true altitude (caller moves it)
    float a = 0.0f;
    bool accel = true;
    float ref = 101325.0f;
    uint32_t ch[2] = {0u, 0u};
    bool started = false;

    NavigationInput at(uint32_t t)   // t = MCU ms of this 10 ms cycle
    {
        if (!started) {   // channel 1's previous sample: 10 ms before the start
            ch[1] = t0 - 10u;
            started = true;
        }
        if ((t - t0) % 20u == 0u) ch[0] = t;
        if ((t - t0) % 20u == 10u) ch[1] = t;
        NavigationInput in{};
        in.nowMs = t + 1u;
        in.baroValid = true;
        in.baroAltitude_m = h;
        in.baroStatus = RedundancyStatus::Healthy;
        in.baroContributingMask = 3u;
        in.baroChannelTimestampMs[0] = ch[0];
        in.baroChannelTimestampMs[1] = ch[1];
        in.baroReferencePressure_pa[0] = in.baroReferencePressure_pa[1] = ref;
        const bool newer0 = static_cast<int32_t>(ch[0] - ch[1]) > 0;
        in.baroTimestampMs = newer0 ? ch[0] : ch[1];
        in.baroSpreadMs = newer0 ? ch[0] - ch[1] : ch[1] - ch[0];
        in.accelValid = accel;
        in.accelReliable = true;
        in.accelUp_mps2 = a;
        in.accelTimestampMs = t;
        in.attitudeState = AttitudeState::Tracking6Axis;
        in.attitudeAccelCorrection = CorrectionStatus::Accepted;
        in.imuAccelStatus = in.imuGyroStatus = RedundancyStatus::Healthy;
        return in;
    }
};

struct Est {
    explicit Est(const NavigationConfig& c = NavigationConfig{}) : model(ConstantAccelerationModel::fromConfig(c)), est(c, model) {}
    ConstantAccelerationModel model;
    VerticalNavigationEstimator est;
    Hand hand;
    /// Runs [t, t + ms) in 10 ms cycles from MCU time hand.t0 + startOffset.
    const NavigationEstimate& run(uint32_t fromOffset, uint32_t ms)
    {
        for (uint32_t k = 0; k < ms / 10u; ++k) est.update(hand.at(hand.t0 + fromOffset + 10u * k));
        return est.latest();
    }
};

/// Test-only, NONLINEAR process model: constant acceleration plus quadratic
/// drag on velocity (a deliberately different physics, id = Test).
class MockDragModel final : public IVerticalProcessModel {
public:
    explicit MockDragModel(float k) : k_(k) {}
    ProcessModelInfo info() const override { return ProcessModelInfo{ProcessModelId::Test, 7u}; }
    VerticalStateVec propagate(const VerticalStateVec& x, float dt, const ProcessModelInput&) const override
    {
        VerticalStateVec o = x;
        const float drag = -k_ * x[1] * std::fabs(x[1]);
        o[0] = x[0] + x[1] * dt + 0.5f * (x[2] + drag) * dt * dt;
        o[1] = x[1] + (x[2] + drag) * dt;
        o[2] = x[2];
        ++calls;
        return o;
    }
    void processNoise(float dt, const VerticalStateVec& x, const ProcessModelInput& u, VerticalCov& Q) const override
    {
        ConstantAccelerationModel::fromConfig(NavigationConfig{}).processNoise(dt, x, u, Q);
    }
    bool parametersValid() const override { return k_ >= 0.0f; }
    mutable uint32_t calls = 0;

private:
    float k_;
};

/// Model that starts returning NaN on request (numerical-fault injection).
class FaultyModel final : public IVerticalProcessModel {
public:
    ProcessModelInfo info() const override { return ProcessModelInfo{ProcessModelId::Test, 1u}; }
    VerticalStateVec propagate(const VerticalStateVec& x, float dt, const ProcessModelInput& u) const override
    {
        VerticalStateVec o = ca_.propagate(x, dt, u);
        if (broken) o[0] = std::nanf("");
        return o;
    }
    void processNoise(float dt, const VerticalStateVec& x, const ProcessModelInput& u, VerticalCov& Q) const override
    {
        ca_.processNoise(dt, x, u, Q);
    }
    bool parametersValid() const override { return true; }
    bool broken = false;

private:
    ConstantAccelerationModel ca_{1.0f};
};

/// A stand-in for a FUTURE safety consumer: uses only NavigationEstimate.
bool fastDescentWarning(const NavigationEstimate& e)
{
    return e.velocityValid && e.verticalVelocity_mps < -8.0f;
}

bool sameState(const NavigationEstimate& a, const NavigationEstimate& b)
{
    return std::memcmp(&a.altitude_m, &b.altitude_m, sizeof(float)) == 0 &&
           std::memcmp(&a.verticalVelocity_mps, &b.verticalVelocity_mps, sizeof(float)) == 0 &&
           std::memcmp(a.covariance, b.covariance, sizeof(a.covariance)) == 0 && a.timestampMs == b.timestampMs;
}

}  // namespace

TEST(nav_est_invalid_configuration_disables_estimator)
{
    NavigationConfig c;
    c.baroSigma_m = 0.0f;
    Est x(c);
    CHECK(!x.est.configValid());
    const NavigationEstimate& e = x.run(0, 1000);
    CHECK(e.state == NavState::Unavailable);
    CHECK(e.has(navflag::InvalidConfig));
    CHECK(!e.altitudeValid && std::isnan(e.altitude_m));

    NavigationConfig c2;
    c2.ukf.alpha = 0.0f;
    Est y(c2);
    CHECK(!y.est.configValid());
    ConstantAccelerationModel bad(-1.0f);
    VerticalNavigationEstimator z(NavigationConfig{}, bad);
    CHECK(!z.configValid());
}

TEST(nav_est_not_initialised_outputs_nan_never_zero)
{
    Est x;
    NavigationInput in = x.hand.at(x.hand.t0);
    in.baroValid = false;
    const NavigationEstimate& e = x.est.update(in);
    CHECK(e.state == NavState::Uninitialized);
    CHECK(std::isnan(e.altitude_m) && std::isnan(e.verticalVelocity_mps) && std::isnan(e.verticalAccel_mps2));
    CHECK(std::isnan(e.covariance[0][0]));
    CHECK(!e.altitudeValid && !e.velocityValid && !e.accelerationValid);
    CHECK(e.has(navflag::NotInitialized));
    CHECK(e.estimator == NavEstimatorId::VerticalUkf);
    CHECK(e.processModel.id == ProcessModelId::ConstantAcceleration);
}

TEST(nav_est_initialises_during_climb_with_honest_velocity_uncertainty)
{
    Est x;
    x.hand.accel = false;   // no attitude yet (as during the first seconds after power-up)
    for (uint32_t k = 0; k < 100u; ++k) {
        x.hand.h = 100.0f + 3.0f * 0.01f * float(k);   // climbing at 3 m/s
        x.est.update(x.hand.at(x.hand.t0 + 10u * k));
        if (k < 40u) CHECK(x.est.latest().state == NavState::Initializing || x.est.latest().state == NavState::Uninitialized);
    }
    const NavigationEstimate& e = x.est.latest();
    CHECK(e.initializations == 1u);
    CHECK(e.altitudeValid && e.velocityValid);
    CHECK(std::fabs(e.verticalVelocity_mps - 3.0f) < 3.0f * e.velocitySigma_mps);
    CHECK(std::fabs(e.verticalVelocity_mps - 3.0f) < 0.3f);
    CHECK(e.velocitySigma_mps > 0.05f);                    // not pretending to know it exactly
    CHECK(!e.accelerationValid);                           // no acceleration measured
    CHECK(e.state == NavState::Degraded && e.has(navflag::AccelUnavailable));
    CHECK(e.accel.accepted == 0u);                         // invalid acceleration never assimilated (not as zero)
}

TEST(nav_est_barometer_vote_assimilated_once_per_fresh_pair)
{
    Est x;
    x.run(0, 3000);
    const uint32_t before = x.est.latest().baro.accepted;
    const uint32_t accelBefore = x.est.latest().accel.accepted;
    x.run(3000, 2000);   // 2 s: 200 barometer samples (2 x 50 Hz) = 100 votes of two fresh samples
    const NavigationEstimate& e = x.est.latest();
    CHECK_EQ(e.baro.accepted - before, 100u);   // one observation per fresh pair (every 20 ms), not per frame
    std::printf("    [metric] barometer observations assimilated in 2 s (2 sensors x 50 Hz, frames at 100 Hz): %u\n",
                unsigned(e.baro.accepted - before));
    CHECK(e.baro.notNew >= 100u);                // the other half of the frames repeat a sample
    CHECK_EQ(e.accel.accepted - accelBefore, 200u);   // acceleration: every new 10 ms sample, once

    // A sample stamped AFTER the processing time is refused (clock / producer error).
    NavigationInput fut = x.hand.at(x.hand.t0 + 5000u);
    fut.accelTimestampMs = fut.nowMs + 5u;
    CHECK(x.est.update(fut).accel.last == MeasurementOutcome::FutureTimestamp);
}

TEST(nav_est_duplicate_frames_change_nothing_and_old_samples_are_skipped)
{
    Est a, b;
    for (uint32_t k = 0; k < 300u; ++k) {
        const NavigationInput in = a.hand.at(a.hand.t0 + 10u * k);
        a.est.update(in);
        const NavigationInput inB = b.hand.at(b.hand.t0 + 10u * k);
        b.est.update(inB);
        NavigationInput dup = inB;
        dup.nowMs += 2u;
        b.est.update(dup);   // republished frame (no new data)
    }
    CHECK(sameState(a.est.latest(), b.est.latest()));
    CHECK(b.est.latest().accel.notNew >= 299u);

    // An OLD acceleration sample (timestamp before the last one) is skipped.
    NavigationInput old = b.hand.at(b.hand.t0 + 3000u);
    b.est.update(old);
    NavigationInput older = old;
    older.accelTimestampMs -= 20u;
    older.nowMs += 1u;
    older.accelUp_mps2 = 50.0f;
    const NavigationEstimate before = b.est.latest();
    const NavigationEstimate& e = b.est.update(older);
    CHECK(e.accel.last == MeasurementOutcome::OutOfOrder);
    CHECK(e.accel.outOfOrder == before.accel.outOfOrder + 1u);
    CHECK(sameState(before, e));
}

TEST(nav_est_late_sample_inflated_too_late_rejected)
{
    NavigationConfig c;
    Est x(c);
    x.run(0, 2000);
    // GNSS arrives 150 ms "late" relative to the filter time: allowed (gnssMaxLateMs 300) but inflated.
    NavigationInput in = x.hand.at(x.hand.t0 + 2000u);
    in.gnssValid = true;
    in.gnssVerticalVelocity_mps = 0.0f;
    in.gnssSpeedAccuracy_mps = 0.2f;
    in.gnssTimestampMs = in.nowMs - 151u;
    in.gnssTimeOfWeekMs = 1000u;
    in.gnssFixType = GnssFixType::Fix3D;
    in.gnssNumSatellites = 10u;
    const NavigationEstimate& e = x.est.update(in);
    CHECK(e.gnss.last == MeasurementOutcome::AcceptedInflated);
    CHECK(e.gnss.measurementSigma > 0.3f);   // 0.2 * 1.5 = 0.3 before inflation
    CHECK(e.has(navflag::LateMeasurement));
    // A later solution (newer than the previous one, so not out of order) that is
    // ~410 ms behind the filter time: rejected as too late (gnssMaxLateMs 300).
    x.run(2010u, 390u);
    NavigationInput in3 = x.hand.at(x.hand.t0 + 2400u);
    in3.gnssValid = true;
    in3.gnssVerticalVelocity_mps = 0.0f;
    in3.gnssSpeedAccuracy_mps = 0.2f;
    in3.gnssTimestampMs = in3.nowMs - 350u;   // fresh enough (age 350 <= 400) but ~340 ms behind the filter
    in3.gnssTimeOfWeekMs = 3000u;
    in3.gnssFixType = GnssFixType::Fix3D;
    in3.gnssNumSatellites = 10u;
    const NavigationEstimate& e3 = x.est.update(in3);
    CHECK(e3.gnss.last == MeasurementOutcome::TooLate);
    CHECK(e3.gnss.tooLate == 1u);
}

TEST(nav_est_wraparound_identical_to_normal_time)
{
    Est a, b;
    a.hand.t0 = 1000u;
    b.hand.t0 = 0xFFFFFFFFu - 4995u;   // wraps 5 s into the run
    for (uint32_t k = 0; k < 1200u; ++k) {
        const float h = 100.0f + 2.0f * std::sin(0.002f * float(k));
        const float acc = -2.0f * 0.002f * 0.002f * 1e4f * std::sin(0.002f * float(k));
        a.hand.h = b.hand.h = h;
        a.hand.a = b.hand.a = acc;
        a.est.update(a.hand.at(a.hand.t0 + 10u * k));
        b.est.update(b.hand.at(b.hand.t0 + 10u * k));
    }
    const NavigationEstimate& ea = a.est.latest();
    const NavigationEstimate& eb = b.est.latest();
    CHECK(ea.state == NavState::Tracking && eb.state == NavState::Tracking);
    CHECK(std::memcmp(&ea.altitude_m, &eb.altitude_m, sizeof(float)) == 0);
    CHECK(std::memcmp(&ea.verticalVelocity_mps, &eb.verticalVelocity_mps, sizeof(float)) == 0);
    CHECK(std::memcmp(ea.covariance, eb.covariance, sizeof(ea.covariance)) == 0);
    CHECK_EQ(ea.baro.accepted, eb.baro.accepted);
    CHECK(eb.timestampMs < 100000u);   // really wrapped
}

TEST(nav_est_reference_change_reanchors_altitude)
{
    Est x;
    x.run(0, 3000);
    // New barometer calibration: reference pressure changes and the relative altitude jumps to ~0.
    x.hand.ref = 100000.0f;
    x.hand.h = 0.0f;
    x.run(3000, 500);
    const NavigationEstimate& e = x.est.latest();
    CHECK_EQ(e.referenceResets, 1u);
    CHECK(std::fabs(e.altitude_m) < 0.5f);
    CHECK_EQ(e.baro.rejectedGate, 0u);
    CHECK(std::fabs(e.verticalVelocity_mps) < 0.5f);   // no fake velocity spike from the jump
}

TEST(nav_est_ambiguous_barometers_skipped)
{
    Est x;
    x.run(0, 2000);
    NavigationInput in = x.hand.at(x.hand.t0 + 2000u);
    in.baroValid = false;
    in.baroStatus = RedundancyStatus::Ambiguous;
    const NavigationEstimate& e = x.est.update(in);
    CHECK(e.baro.last == MeasurementOutcome::Ambiguous);
    CHECK(e.altitudeValid);   // one skipped sample does not invalidate the estimate
}

TEST(nav_est_gnss_quality_rules)
{
    struct Case { GnssFixType fix; uint8_t sats; float sAcc; MeasurementOutcome expect; };
    const Case cases[] = {{GnssFixType::Fix3D, 10u, 0.3f, MeasurementOutcome::Accepted},
                          {GnssFixType::Fix2D, 10u, 0.3f, MeasurementOutcome::RejectedQuality},
                          {GnssFixType::Fix3D, 4u, 0.3f, MeasurementOutcome::RejectedQuality},
                          {GnssFixType::Fix3D, 10u, 3.0f, MeasurementOutcome::RejectedQuality},
                          {GnssFixType::Fix3D, 10u, std::nanf(""), MeasurementOutcome::RejectedQuality},
                          {GnssFixType::GnssPlusDeadReckoning, 10u, 0.3f, MeasurementOutcome::Accepted}};
    for (const Case& k : cases) {
        Est x;
        x.run(0, 2000);
        NavigationInput in = x.hand.at(x.hand.t0 + 2000u);
        in.gnssValid = true;
        in.gnssVerticalVelocity_mps = 0.1f;
        in.gnssSpeedAccuracy_mps = k.sAcc;
        in.gnssTimestampMs = in.nowMs - 1u;
        in.gnssTimeOfWeekMs = 777u;
        in.gnssFixType = k.fix;
        in.gnssNumSatellites = k.sats;
        CHECK(x.est.update(in).gnss.last == k.expect);
        // the same solution again (same iTOW) is never assimilated twice
        in.nowMs += 10u;
        in.gnssTimestampMs += 10u;
        CHECK(x.est.update(in).gnss.last == MeasurementOutcome::NotNew);
    }
    NavigationConfig off;
    off.useGnssVelocity = false;
    Est y(off);
    y.run(0, 2000);
    NavigationInput in = y.hand.at(y.hand.t0 + 2000u);
    in.gnssValid = true;
    in.gnssSpeedAccuracy_mps = 0.3f;
    in.gnssFixType = GnssFixType::Fix3D;
    in.gnssNumSatellites = 10u;
    CHECK(y.est.update(in).gnss.last == MeasurementOutcome::Disabled);
}

TEST(nav_adapter_maps_frame_and_attitude_signs_and_validity)
{
    ProcessedSensorFrame f{};
    f.timestampMs = 1000u;
    f.baroAltitude_m.valid = true;
    f.baroAltitude_m.value = 12.5f;
    f.baroAltitude_m.timestampMs = 995u;
    f.baroAltitude_m.sampleSpreadMs = 10u;
    f.baroAltitude_m.status = RedundancyStatus::Healthy;
    f.baroAltitude_m.contributingMask = 3u;
    f.baro[0].timestampMs = 985u;
    f.baro[1].timestampMs = 995u;
    f.baro[0].referencePressure_pa = 101000.0f;
    f.baro[1].referencePressure_pa = 101010.0f;
    f.gnss.valid = true;
    f.gnss.verticalVelocity_mps = -1.7f;   // positive UP: the driver already negated NAV-PVT velD (+1.7 down)
    f.gnss.speedAccuracy_mps = 0.4f;
    f.gnss.timeOfWeek_ms = 123456u;
    f.gnss.fixType = GnssFixType::Fix3D;
    f.gnss.numSatellites = 9u;
    f.qualityFlags = quality::BaroSourceChanged;
    AttitudeEstimate a{};
    a.orientationValid = true;
    a.linearAccel.valid = true;
    a.linearAccel.reliable = false;
    a.linearAccel.vertical_mps2 = 0.8f;
    a.linearAccel.timestampMs = 998u;
    a.state = AttitudeState::Degraded;
    a.accelCorrection = CorrectionStatus::RejectedRotation;
    a.qualityFlags = attitude::GapDetected;
    a.eulerDeg.roll = 30.0f;
    const NavigationInput in = navigationInputFrom(f, a);
    CHECK_EQ(in.nowMs, 1000u);
    CHECK(in.baroValid && in.baroAltitude_m == 12.5f && in.baroSpreadMs == 10u);
    CHECK_EQ(in.baroChannelTimestampMs[0], 985u);
    CHECK(in.baroReferencePressure_pa[1] == 101010.0f);
    CHECK(in.baroSourceChanged);
    CHECK(in.accelValid && !in.accelReliable && in.accelUp_mps2 == 0.8f && in.accelTimestampMs == 998u);
    CHECK(in.attitudeAccelCorrection == CorrectionStatus::RejectedRotation);
    CHECK(in.gnssValid && in.gnssVerticalVelocity_mps == -1.7f && in.gnssSpeedAccuracy_mps == 0.4f);
    CHECK_NEAR(in.model.bankAngle_rad, 0.5235988, 1e-6);
    a.orientationValid = false;
    CHECK(!navigationInputFrom(f, a).accelValid);   // no orientation -> no vertical acceleration
    CHECK(std::isnan(navigationInputFrom(f, a).model.bankAngle_rad));
}

TEST(nav_est_acceleration_quality_inflation_and_rejection)
{
    struct Case { int kind; MeasurementOutcome expect; float sigma; };
    const float s0 = NavigationConfig{}.accelSigma_mps2;
    const Case cases[] = {{0, MeasurementOutcome::Accepted, s0},
                          {1, MeasurementOutcome::Accepted, s0 * 1.5f},           // Madgwick turn gate
                          {2, MeasurementOutcome::Accepted, s0 * 3.0f},           // attitude not reliable
                          {3, MeasurementOutcome::RejectedQuality, NAN},          // gyro gap not yet confirmed
                          {4, MeasurementOutcome::Accepted, s0 * 1.5f},           // IMU vote degraded
                          {5, MeasurementOutcome::RejectedQuality, NAN}};         // 3 x 1.5 x 3 = 13.5 > 10
    for (const Case& k : cases) {
        Est x;
        x.run(0, 2000);
        NavigationInput in = x.hand.at(x.hand.t0 + 2000u);
        if (k.kind == 1) in.attitudeAccelCorrection = CorrectionStatus::RejectedRotation;
        if (k.kind == 2) in.accelReliable = false;
        if (k.kind == 3) in.attitudeFlags |= attitude::GapDetected;
        if (k.kind == 4) in.imuAccelStatus = RedundancyStatus::Degraded;
        if (k.kind == 5) {
            in.accelReliable = false;
            in.attitudeAccelCorrection = CorrectionStatus::RejectedRotation;
            in.attitudeFlags |= attitude::AccelRejectedProlonged;
        }
        const NavigationEstimate& e = x.est.update(in);
        CHECK(e.accel.last == k.expect);
        if (std::isfinite(k.sigma)) CHECK_NEAR(e.accel.measurementSigma, k.sigma, 1e-5);
    }
}

TEST(nav_est_outage_degraded_unavailable_then_reinitialised)
{
    Est x;
    x.run(0, 3000);
    CHECK(x.est.latest().state == NavState::Tracking);
    // Everything silent (no barometer, no acceleration).
    bool sawDegraded = false, sawUnavailable = false;
    float maxSigma = 0.0f;
    for (uint32_t k = 0; k < 400u; ++k) {
        NavigationInput in = x.hand.at(x.hand.t0 + 3000u + 10u * k);
        in.baroValid = false;
        in.baroStatus = RedundancyStatus::Unavailable;
        in.accelValid = false;
        const NavigationEstimate& e = x.est.update(in);
        sawDegraded = sawDegraded || e.state == NavState::Degraded;
        sawUnavailable = sawUnavailable || e.state == NavState::Unavailable;
        if (e.state == NavState::Degraded) maxSigma = std::fmax(maxSigma, e.velocitySigma_mps);
        if (e.state == NavState::Degraded) CHECK(e.has(navflag::PredictionOnly));
    }
    CHECK(sawDegraded && sawUnavailable);
    CHECK(maxSigma > 1.0f);                       // covariance grew during the outage
    CHECK(x.est.latest().state == NavState::Unavailable);
    CHECK(!x.est.latest().altitudeValid && std::isnan(x.est.latest().altitude_m));
    CHECK_EQ(x.est.latest().unavailableEpisodes, 1u);
    x.run(7000u, 2000u);                           // data return
    CHECK(x.est.latest().state == NavState::Tracking);
    CHECK_EQ(x.est.latest().initializations, 2u);
    CHECK_EQ(x.est.latest().reinitializations, 1u);
}

TEST(nav_est_numerical_fault_reported_and_recovered)
{
    FaultyModel m;
    VerticalNavigationEstimator est(NavigationConfig{}, m);
    Hand h;
    for (uint32_t k = 0; k < 200u; ++k) est.update(h.at(h.t0 + 10u * k));
    CHECK(est.latest().state == NavState::Tracking);
    m.broken = true;
    const NavigationEstimate& e = est.update(h.at(h.t0 + 2000u));
    CHECK(e.state == NavState::NumericalFault);
    CHECK(e.has(navflag::NumericalFault));
    CHECK_EQ(e.numericalFaults, 1u);
    CHECK_EQ(e.lastFaultCode, uint32_t(nav::UkfStatus::NonFiniteSigmaPoint));
    CHECK(!e.altitudeValid && std::isnan(e.altitude_m));   // the bad numbers are not published
    m.broken = false;
    for (uint32_t k = 201; k < 400u; ++k) est.update(h.at(h.t0 + 10u * k));
    CHECK(est.latest().state == NavState::Tracking);
    CHECK_EQ(est.latest().reinitializations, 1u);
    CHECK_EQ(est.latest().numericalFaults, 1u);            // diagnostics preserved
}

TEST(nav_process_model_swap_with_nonlinear_mock_model)
{
    // Same estimator class, same inputs, same consumer: only the model object differs.
    vsim::Scenario sc;
    sc.name = "steep descent";
    sc.durationS = 40.0;
    sc.v0 = -2.0;
    sc.accel = [](double t) { return vsim::velocityChange(t, 10.0, 4.0, -8.0); };   // to -10 m/s
    NavigationConfig cfg;
    MockDragModel drag(0.005f);   // 0.5 m/s^2 of (fictitious) drag at 10 m/s: deliberately NOT the truth's physics
    ConstantAccelerationModel ca = ConstantAccelerationModel::fromConfig(cfg);
    VerticalNavigationEstimator withCa(cfg, ca), withMock(cfg, drag);
    vsim::Runner feed(sc, 5);
    bool warnCa = false, warnMock = false;
    float maxDiff = 0.0f;
    feed.each = [&](double, const NavigationInput& in, const NavigationEstimate&, const NavigationEstimate&,
                    const vsim::TruthPoint&) {
        const NavigationEstimate& a = withCa.update(in);
        const NavigationEstimate& b = withMock.update(in);
        warnCa = warnCa || fastDescentWarning(a);
        warnMock = warnMock || fastDescentWarning(b);
        if (a.velocityValid && b.velocityValid)
            maxDiff = std::fmax(maxDiff, std::fabs(a.verticalVelocity_mps - b.verticalVelocity_mps));
    };
    feed.run();
    const NavigationEstimate& m = withMock.latest();
    CHECK(withMock.configValid());
    CHECK(m.processModel.id == ProcessModelId::Test && m.processModel.version == 7u);
    CHECK(!m.has(navflag::ModelPlaceholder));
    CHECK(withCa.latest().has(navflag::ModelPlaceholder));
    CHECK(m.state == NavState::Tracking && m.velocityValid);
    CHECK(drag.calls > 1000u);                 // the engine really used the injected model
    CHECK(warnCa && warnMock);                 // the consumer works unchanged with either model
    CHECK(maxDiff > 0.01f);                    // and the model really changes the estimate
    CHECK(std::fabs(m.verticalVelocity_mps + 10.0f) < 1.0f);
    std::printf("    [metric] model swap (CA vs nonlinear drag mock): max |vz difference| %.3f m/s, mock final vz %.2f m/s\n",
                double(maxDiff), double(m.verticalVelocity_mps));
}

TEST(nav_engine_reused_with_four_states)
{
    // The engine is templated on N: a 4-state model (CA + barometer bias random walk)
    // runs unchanged. Production stays at N = 3 (documented: the estimator, the
    // process-model interface and NavigationEstimate would change for N = 4).
    struct BiasModel {
        nav::Vec<4> propagate(const nav::Vec<4>& x, float dt, int) const
        {
            nav::Vec<4> o = x;
            o[0] = x[0] + x[1] * dt + 0.5f * x[2] * dt * dt;
            o[1] = x[1] + x[2] * dt;
            return o;
        }
        void processNoise(float dt, const nav::Vec<4>&, int, nav::Mat<4, 4>& Q) const
        {
            Q = nav::Mat<4, 4>::zero();
            Q(0, 0) = dt * dt * dt * dt * dt / 20;
            Q(0, 1) = Q(1, 0) = dt * dt * dt * dt / 8;
            Q(0, 2) = Q(2, 0) = dt * dt * dt / 6;
            Q(1, 1) = dt * dt * dt / 3;
            Q(1, 2) = Q(2, 1) = dt * dt / 2;
            Q(2, 2) = dt;
            Q(3, 3) = 1e-4f * dt;
        }
    } model;
    nav::UnscentedKalmanFilter<4> u;
    CHECK(u.configure(nav::UkfParams{1.0f, 2.0f, -1.0f}));   // n + kappa = 3
    nav::Mat<4, 4> P = nav::Mat<4, 4>::identity();
    CHECK(u.initialize(nav::Vec<4>{}, P));
    const auto baro = [](const nav::Vec<4>& x) { nav::Vec<1> z{}; z[0] = x[0] + x[3]; return z; };   // h + bias
    const auto gnssH = [](const nav::Vec<4>& x) { nav::Vec<1> z{}; z[0] = x[0]; return z; };          // unbiased height
    for (int k = 0; k < 2000; ++k) {
        CHECK(u.predict(model, 0.02f, 0) == nav::UkfStatus::Ok);
        nav::Innovation<4, 1> in{};
        CHECK(u.evaluate(nav::Vec<1>{{3.0f}}, nav::Mat<1, 1>{{{0.25f}}}, baro, in) == nav::UkfStatus::Ok);
        CHECK(u.apply(in) == nav::UkfStatus::Ok);
        if (k % 10 == 0) {
            CHECK(u.evaluate(nav::Vec<1>{{0.0f}}, nav::Mat<1, 1>{{{1.0f}}}, gnssH, in) == nav::UkfStatus::Ok);
            CHECK(u.apply(in) == nav::UkfStatus::Ok);
        }
    }
    CHECK(std::fabs(u.state()[3] - 3.0f) < 0.5f);   // bias separated from height
}

TEST(nav_est_persistent_barometer_disagreement_recovered_after_delay)
{
    Est x;
    x.run(0, 3000);
    x.hand.h = 130.0f;   // both barometers jump +30 m (cannot be voted out)
    uint32_t firstAccept = 0u;
    for (uint32_t k = 0; k < 400u; ++k) {
        const NavigationEstimate& e = x.est.update(x.hand.at(x.hand.t0 + 3000u + 10u * k));
        if (firstAccept == 0u && e.baro.last == MeasurementOutcome::AcceptedInflated) firstAccept = 10u * k;
    }
    const NavigationEstimate& e = x.est.latest();
    CHECK(firstAccept >= 1900u && firstAccept <= 2200u);   // baroRejectRecoveryMs = 2000
    CHECK(e.baro.recoveries == 1u);
    CHECK(e.baro.rejectedGate >= 90u);
    CHECK(std::fabs(e.altitude_m - 130.0f) < 1.0f);
    std::printf("    [metric] +30 m step on both barometers: rejected %u samples, re-anchored after %u ms\n",
                unsigned(e.baro.rejectedGate), unsigned(firstAccept));
}
