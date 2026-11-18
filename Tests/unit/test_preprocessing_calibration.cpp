/**
 * Preprocessing milestone - stationary detection, gyro bias calibration,
 * barometer reference calibration and the barometric formula.
 */
#include <cmath>
#include <cstdio>

#include "TestFramework.hpp"
#include "sim/SensorSimulator.hpp"

#include "Processing/Calibration.hpp"
#include "Processing/CalibrationController.hpp"
#include "Processing/StationaryDetector.hpp"

using namespace pg;

namespace {

constexpr float kG = units::kStandardGravity_mps2;

/// Feeds a detector with 100 Hz samples from @p t0 for @p durationMs.
StationaryStatus feed(StationaryDetector& d, sim::Rng& rng, uint32_t t0, uint32_t durationMs, const Vec3f& gyroBias,
                      float gyroNoise, float accelNoise, Vec3f (*motion)(uint32_t) = nullptr)
{
    StationaryStatus st = d.status();
    for (uint32_t t = t0; t < t0 + durationMs; t += 10u) {
        Vec3f w = vec::add(gyroBias, rng.gaussian3(gyroNoise));
        Vec3f a = vec::add(Vec3f{0.0f, 0.0f, -kG}, rng.gaussian3(accelNoise));
        if (motion != nullptr) {
            const Vec3f m = motion(t);
            w = vec::add(w, m);
        }
        st = d.add(t, w, a);
    }
    return st;
}

}  // namespace

// ============================================================== stationary ==

TEST(prep_stationary_perfectly_still_after_full_window)
{
    StationaryDetector d{StationaryConfig{}};
    sim::Rng rng(1);
    CHECK(feed(d, rng, 0, 4990, Vec3f{0, 0, 0}, 0.0f, 0.0f) == StationaryStatus::InsufficientData);
    CHECK(feed(d, rng, 4990, 20, Vec3f{0, 0, 0}, 0.0f, 0.0f) == StationaryStatus::Stationary);
    CHECK(d.report().samples >= d.minSamples());
    CHECK_EQ(d.minSamples(), 400u);
}

TEST(prep_stationary_realistic_noise_and_uncalibrated_bias)
{
    StationaryDetector d{StationaryConfig{}};
    sim::Rng rng(2);
    // 1.5 dps bias (uncalibrated), 0.0012 rad/s gyro noise, 0.008 m/s^2 accel noise.
    const Vec3f bias{0.026f, -0.015f, 0.01f};
    CHECK(feed(d, rng, 0, 5100, bias, 0.0012f, 0.008f) == StationaryStatus::Stationary);
    CHECK_NEAR(d.report().gyroMean_radps.x, 0.026, 0.001);
}

TEST(prep_stationary_constant_rotation_rejected)
{
    StationaryDetector d{StationaryConfig{}};
    sim::Rng rng(3);
    // 0.2 rad/s yaw: above the instantaneous limit -> Moving on the first sample.
    CHECK(feed(d, rng, 0, 6000, Vec3f{0.0f, 0.0f, 0.2f}, 0.001f, 0.005f) == StationaryStatus::Moving);
    CHECK(d.report().lastCause == MotionCause::GyroMagnitude);
}

TEST(prep_stationary_slow_constant_rotation_not_mistaken_for_bias)
{
    // A slow 0.05 rad/s rotation is below the instantaneous limit and constant,
    // so EVERY gyro-only test sees a perfect "bias", and |a| stays exactly g.
    // Only the direction of gravity in sensor axes reveals it: the per-axis
    // accelerometer drift test must reject it (otherwise the rotation would be
    // stored as a 2.9 dps gyro bias).
    for (float rate : {0.05f, 0.005f}) {
        StationaryDetector d{StationaryConfig{}};
        bool everStationary = false;
        for (uint32_t t = 0; t < 12000u; t += 10u) {
            const float ang = rate * static_cast<float>(t) * 1e-3f;          // about sensor X
            const Vec3f a{0.0f, kG * std::sin(ang), -kG * std::cos(ang)};    // |a| = g
            everStationary = everStationary || d.add(t, Vec3f{rate, 0.0f, 0.0f}, a) == StationaryStatus::Stationary;
        }
        CHECK(!everStationary);
        CHECK(d.report().lastCause == MotionCause::AccelDrift);
    }
}

TEST(prep_stationary_translational_acceleration_rejected)
{
    StationaryDetector d{StationaryConfig{}};
    sim::Rng rng(4);
    feed(d, rng, 0, 3000, Vec3f{0, 0, 0}, 0.001f, 0.005f);
    // Push forward at 1.5 m/s^2: |a| = sqrt(g^2 + 1.5^2) = 9.92 -> only 0.11 above g
    // (inside the instantaneous tolerance), but the |a| standard deviation /
    // segment means change: rejected by the window statistics.
    for (uint32_t t = 3000; t < 3600; t += 10u) {
        d.add(t, Vec3f{0.0f, 0.0f, 0.0f}, Vec3f{1.5f, 0.0f, -kG});
    }
    feed(d, rng, 3600, 1500, Vec3f{0, 0, 0}, 0.001f, 0.005f);
    CHECK(d.status() != StationaryStatus::Stationary);
    CHECK(d.report().motionRestarts >= 1u);
    // A hard 3 m/s^2 shove violates the instantaneous limit immediately.
    StationaryDetector d2{StationaryConfig{}};
    CHECK(d2.add(0, Vec3f{0, 0, 0}, Vec3f{0, 0, -kG}) == StationaryStatus::InsufficientData);
    CHECK(d2.add(10, Vec3f{0, 0, 0}, Vec3f{0, 0, -kG - 3.0f}) == StationaryStatus::Moving);
    CHECK(d2.report().lastCause == MotionCause::AccelMagnitude);
}

TEST(prep_stationary_short_spike_restarts_whole_window)
{
    StationaryDetector d{StationaryConfig{}};
    sim::Rng rng(5);
    feed(d, rng, 0, 4500, Vec3f{0, 0, 0}, 0.001f, 0.005f);
    d.add(4500, Vec3f{0.5f, 0.0f, 0.0f}, Vec3f{0, 0, -kG});  // one 10 ms bump
    CHECK(d.status() == StationaryStatus::Moving);
    // 4.5 s of stillness BEFORE the bump does not count: another full window is needed.
    CHECK(feed(d, rng, 4510, 4000, Vec3f{0, 0, 0}, 0.001f, 0.005f) == StationaryStatus::InsufficientData);
    CHECK(feed(d, rng, 8510, 1100, Vec3f{0, 0, 0}, 0.001f, 0.005f) == StationaryStatus::Stationary);
    CHECK_EQ(d.report().windowStartMs, 4510u);
}

TEST(prep_stationary_intermittent_missing_data)
{
    // Every 4th sample missing (75 % present): 375 per 5 s < 400 minimum ->
    // the window must stretch until 400 samples were seen.
    StationaryDetector d{StationaryConfig{}};
    uint32_t firstStillMs = 0;
    for (uint32_t t = 0; t < 7000u; t += 10u) {
        if ((t / 10u) % 4u == 3u) continue;
        if (d.add(t, Vec3f{0, 0, 0}, Vec3f{0, 0, -kG}) == StationaryStatus::Stationary && firstStillMs == 0u) {
            firstStillMs = t;
        }
    }
    CHECK(firstStillMs > 5000u);
    CHECK(d.report().samples >= 400u);
    // A gap longer than maxGapMs (50 ms) restarts the window.
    StationaryDetector g{StationaryConfig{}};
    for (uint32_t t = 0; t < 3000u; t += 10u) g.add(t, Vec3f{0, 0, 0}, Vec3f{0, 0, -kG});
    g.add(3100, Vec3f{0, 0, 0}, Vec3f{0, 0, -kG});
    CHECK_EQ(g.report().gapRestarts, 1u);
    CHECK_EQ(g.report().windowStartMs, 3100u);
    CHECK(g.report().lastCause == MotionCause::DataGap);
}

TEST(prep_stationary_slowly_drifting_gyro)
{
    // Warm-up drift of 0.002 rad/s over 5 s: accepted (bias = average).
    StationaryDetector ok{StationaryConfig{}};
    for (uint32_t t = 0; t <= 5000u; t += 10u) {
        ok.add(t, Vec3f{0.01f + 0.002f * static_cast<float>(t) / 5000.0f, 0, 0}, Vec3f{0, 0, -kG});
    }
    CHECK(ok.status() == StationaryStatus::Stationary);
    // Fast drift of 0.02 rad/s over 5 s (std ~ 0.0058 < 0.01, so the noise test
    // alone would pass): rejected by the segment-mean drift test.
    StationaryDetector bad{StationaryConfig{}};
    for (uint32_t t = 0; t <= 5000u; t += 10u) {
        bad.add(t, Vec3f{0.01f + 0.02f * static_cast<float>(t) / 5000.0f, 0, 0}, Vec3f{0, 0, -kG});
    }
    CHECK(bad.status() != StationaryStatus::Stationary);
    CHECK(bad.report().lastCause == MotionCause::GyroDrift);
}

TEST(prep_stationary_noisy_window_rejected_by_statistics)
{
    StationaryDetector d{StationaryConfig{}};
    sim::Rng rng(6);
    // 0.02 rad/s gyro noise (body sway): each sample under 0.10 rad/s mostly, but std 0.02 > 0.01.
    const StationaryStatus st = feed(d, rng, 0, 5100, Vec3f{0, 0, 0}, 0.02f, 0.005f);
    CHECK(st != StationaryStatus::Stationary);
    CHECK(d.report().motionRestarts >= 1u);
}

// ================================================================ gyro cal ==

TEST(prep_gyrocal_known_offsets_with_noise_error_metric)
{
    GyroBiasCalibrator c{GyroCalibrationConfig{}};
    sim::Rng rng(7);
    const Vec3f bias{0.0123f, -0.0311f, 0.0047f};  // +0.70, -1.78, +0.27 dps
    const float sigma = 0.0015f;
    c.begin();
    for (int i = 0; i < 500; ++i) c.add(vec::add(bias, rng.gaussian3(sigma)));
    CHECK(c.finish(5000, 1));
    const GyroCalibration& r = c.active();
    const float err = vec::distance(r.bias_radps, bias);
    // Expected 1-sigma error per axis = sigma / sqrt(500) = 6.7e-5 rad/s.
    CHECK(err < 4.0f * sigma / std::sqrt(500.0f) * 1.7320508f);
    CHECK_NEAR(r.biasStdError_radps.x, sigma / std::sqrt(500.0f), 1.5e-5);
    CHECK_NEAR(r.noiseStd_radps.y, sigma, 0.0002);
    CHECK_EQ(r.samples, 500u);
    CHECK_EQ(r.timestampMs, 5000u);
    CHECK(c.state() == CalibrationState::Valid);
    std::printf("    gyro bias error %.2e rad/s (%.4f dps), std err %.2e rad/s\n", static_cast<double>(err),
                static_cast<double>(err / units::kDegToRad), static_cast<double>(r.biasStdError_radps.x));
    const Vec3f corrected = c.correct(vec::add(bias, Vec3f{0.1f, 0.0f, 0.0f}));
    CHECK_NEAR(corrected.x, 0.1, 2e-4);
    CHECK_NEAR(corrected.y, 0.0, 2e-4);
}

TEST(prep_gyrocal_insufficient_samples_keeps_previous)
{
    GyroBiasCalibrator c{GyroCalibrationConfig{}};
    c.begin();
    for (int i = 0; i < 500; ++i) c.add(Vec3f{0.01f, 0.0f, 0.0f});
    CHECK(c.finish(5000, 1));
    c.begin();  // repeated request
    CHECK(c.state() == CalibrationState::Collecting);
    CHECK(c.active().valid);  // old result still in use while collecting
    for (int i = 0; i < 100; ++i) c.add(Vec3f{0.05f, 0.0f, 0.0f});
    CHECK(!c.finish(9000, 2));
    CHECK(c.lastFailure() == CalibrationFailure::InsufficientSamples);
    CHECK(c.active().valid);
    CHECK_NEAR(c.active().bias_radps.x, 0.01, 1e-6);
    CHECK_EQ(c.active().sessionId, 1u);
}

TEST(prep_gyrocal_too_noisy_or_implausible_fails)
{
    GyroBiasCalibrator c{GyroCalibrationConfig{}};
    sim::Rng rng(8);
    c.begin();
    for (int i = 0; i < 500; ++i) c.add(rng.gaussian3(0.05f));
    CHECK(!c.finish(1, 1));
    CHECK(c.lastFailure() == CalibrationFailure::TooNoisy);
    CHECK(c.state() == CalibrationState::Failed);
    c.begin();
    for (int i = 0; i < 500; ++i) c.add(Vec3f{0.5f, 0.0f, 0.0f});  // 28.6 dps "bias" = rotation
    CHECK(!c.finish(2, 2));
    CHECK(c.lastFailure() == CalibrationFailure::InvalidValue);
}

TEST(prep_accel_calibration_interface_is_identity_until_valid)
{
    AccelCalibration a{};
    const Vec3f m{0.1f, -0.2f, -9.7f};
    const Vec3f out = applyAccelCalibration(a, m);
    CHECK_NEAR(out.z, -9.7, 1e-6);
    a.bias_mps2 = Vec3f{0.1f, -0.2f, 0.1f};
    a.valid = true;
    const Vec3f out2 = applyAccelCalibration(a, m);
    CHECK_NEAR(out2.x, 0.0, 1e-6);
    CHECK_NEAR(out2.z, -9.8, 1e-5);
}

// ========================================================== session (IMUs) ==

namespace {

/// Runs the controller from a simulator for @p ms, starting at @p t0.
void runController(CalibrationController& cc, sim::SensorSimulator& s, uint32_t t0, uint32_t ms)
{
    for (uint32_t t = t0; t < t0 + ms; t += 10u) {
        for (const SensorMessage& m : s.step(t)) {
            if (m.type == SensorMessageType::Imu && m.imu.header.valid) {
                cc.onImu(static_cast<size_t>(m.imu.header.id), m.imu.header.timestampMs, m.imu.gyro_radps,
                         m.imu.accel_mps2);
            } else if (m.type == SensorMessageType::Baro && m.baro.header.valid) {
                cc.onBaro(static_cast<size_t>(m.baro.header.id) - 3u, m.baro.header.timestampMs, m.baro.pressure_pa);
            }
        }
        cc.poll(t);
    }
}

}  // namespace

TEST(prep_session_three_imus_different_signed_biases)
{
    PreprocessingConfig cfg{};
    CalibrationController cc(cfg);
    sim::SensorSimulator s(9);
    const Vec3f b[3] = {{0.02f, -0.01f, 0.005f}, {-0.03f, 0.015f, -0.002f}, {0.0f, 0.04f, -0.025f}};
    for (int i = 0; i < 3; ++i) {
        s.imu[i].gyroBias = b[i];
        s.imu[i].gyroNoise = 0.0015f;
        s.imu[i].accelNoise = 0.01f;
    }
    cc.request(0);
    runController(cc, s, 0, 6000);
    CHECK(cc.status().state == CalibrationState::Valid);
    float worst = 0.0f;
    for (size_t i = 0; i < 3; ++i) {
        CHECK(cc.gyro(i).active().valid);
        const float e = vec::distance(cc.gyro(i).active().bias_radps, b[i]);
        worst = (e > worst) ? e : worst;
    }
    CHECK(worst < 3e-4f);
    std::printf("    3-IMU session: worst bias error %.2e rad/s, completed at window start %u ms\n",
                static_cast<double>(worst), cc.status().windowStartMs);
}

TEST(prep_session_missing_imu_does_not_block_others)
{
    CalibrationController cc(PreprocessingConfig{});
    sim::SensorSimulator s(10);
    s.imu[2].present = false;
    s.imu[0].gyroBias = Vec3f{0.01f, 0, 0};
    cc.request(0);
    runController(cc, s, 0, 5200);
    CHECK(cc.status().state == CalibrationState::Valid);
    CHECK(cc.gyro(0).active().valid);
    CHECK(cc.gyro(1).active().valid);
    CHECK(!cc.gyro(2).active().valid);
    CHECK(cc.gyro(2).state() == CalibrationState::Failed);
    CHECK(cc.gyro(2).lastFailure() == CalibrationFailure::InsufficientSamples);
}

TEST(prep_session_interrupted_by_motion_restarts_and_completes_later)
{
    CalibrationController cc(PreprocessingConfig{});
    sim::SensorSimulator s(11);
    s.angularRate = [](uint32_t t) {
        return (t >= 3000u && t < 3300u) ? Vec3f{0.0f, 0.6f, 0.0f} : Vec3f{0, 0, 0};
    };
    cc.request(0);
    runController(cc, s, 0, 5000);
    CHECK(cc.collecting());                       // the 0.3 s of motion restarted the window
    CHECK(cc.status().motionRestarts >= 1u);
    runController(cc, s, 5000, 4000);
    CHECK(cc.status().state == CalibrationState::Valid);
    CHECK(cc.status().windowStartMs >= 3291u);    // window began after the last moving sample (3290)
}

TEST(prep_session_no_stillness_times_out_and_retries)
{
    PreprocessingConfig cfg{};
    cfg.session.sessionTimeoutMs = 8000u;
    CalibrationController cc(cfg);
    sim::SensorSimulator s(12);
    s.angularRate = [](uint32_t t) { return Vec3f{0.3f * std::sin(static_cast<float>(t) * 0.004f) + 0.2f, 0, 0}; };
    cc.request(0);
    runController(cc, s, 0, 8100);
    CHECK_EQ(cc.status().failedSessions, 1u);
    CHECK(cc.status().lastFailure == CalibrationFailure::TimedOut);
    CHECK(cc.collecting());                       // retryOnFailure: a new session started
    CHECK_EQ(cc.status().sessionId, 2u);
    // Without retry: terminal Failed state.
    cfg.session.retryOnFailure = false;
    CalibrationController c2(cfg);
    sim::SensorSimulator s2(12);
    s2.angularRate = s.angularRate;
    c2.request(0);
    runController(c2, s2, 0, 8100);
    CHECK(c2.status().state == CalibrationState::Failed);
    CHECK(!c2.gyro(0).active().valid);
}

TEST(prep_session_steady_barometer_alone_never_calibrates)
{
    // No IMU data at all: "altitude not changing" is NOT evidence of stillness.
    PreprocessingConfig cfg{};
    cfg.session.sessionTimeoutMs = 7000u;
    cfg.session.retryOnFailure = false;
    CalibrationController cc(cfg);
    sim::SensorSimulator s(13);
    for (int i = 0; i < 3; ++i) s.imu[i].present = false;
    cc.request(0);
    runController(cc, s, 0, 7100);
    CHECK(cc.status().state == CalibrationState::Failed);
    CHECK(cc.status().lastFailure == CalibrationFailure::NoStationaryImu);
    CHECK(!cc.baro(0).active().valid);
}

// ===================================================== barometer reference ==

TEST(prep_baro_formula_reference_gives_zero_and_known_points)
{
    CHECK_NEAR(baro::relativeAltitude_m(101325.0, 101325.0), 0.0, 1e-9);
    // Independent check through the inverse formula (sim::pressureAt).
    for (double h : {-50.0, 10.0, 100.0, 1000.0, 3000.0}) {
        const double p = sim::SensorSimulator::pressureAt(h, 101325.0);
        CHECK_NEAR(baro::relativeAltitude_m(p, 101325.0), h, 1e-6);
    }
    // Well-known ISA anchor: 1000 m ~ 898.75 hPa.
    CHECK_NEAR(baro::relativeAltitude_m(89875.0, 101325.0), 1000.0, 1.0);
}

TEST(prep_baro_formula_high_takeoff_scale_error_and_isa_option)
{
    // Take-off at 1500 m (ISA), climb to 1800 m. The specified formula uses the
    // SEA-LEVEL scale height 44330 m and over-states the climb by
    // 44330 / (44330 - 1500) = 1.035: 310.5 m instead of 300 m.
    const double p0 = sim::SensorSimulator::pressureAt(1500.0, 101325.0);
    const double p = sim::SensorSimulator::pressureAt(1800.0, 101325.0);
    CHECK_NEAR(baro::relativeAltitude_m(p, p0), 300.0 * 44330.0 / (44330.0 - 1500.0), 1e-6);
    // ISA temperature at the reference: exact within the ISA model.
    CHECK_NEAR(baro::isaScaleHeight_m(101325.0), 44330.0, 1e-9);
    CHECK_NEAR(baro::relativeAltitude_m(p, p0, baro::isaScaleHeight_m(p0)), 300.0, 1e-6);
    // Selected through the calibration config.
    BaroCalibrationConfig cfg{};
    cfg.altitudeModel = BaroAltitudeModel::IsaAtReference;
    BaroReferenceCalibrator c{cfg};
    c.begin(0);
    for (uint32_t t = 0; t < 5000u; t += 20u) c.add(t, static_cast<float>(p0));
    CHECK(c.finish(5000, 1));
    CHECK_NEAR(c.active().scaleHeight_m, 44330.0 - 1500.0, 0.5);
    BaroReferenceCalibrator d{BaroCalibrationConfig{}};
    d.begin(0);
    for (uint32_t t = 0; t < 5000u; t += 20u) d.add(t, static_cast<float>(p0));
    CHECK(d.finish(5000, 1));
    CHECK_NEAR(d.active().scaleHeight_m, 44330.0, 1e-9);  // default = specified formula
}

TEST(prep_baro_formula_rejects_zero_negative_nonfinite)
{
    CHECK(std::isnan(baro::relativeAltitude_m(0.0, 101325.0)));
    CHECK(std::isnan(baro::relativeAltitude_m(-1.0, 101325.0)));
    CHECK(std::isnan(baro::relativeAltitude_m(101325.0, 0.0)));
    CHECK(std::isnan(baro::relativeAltitude_m(std::nan(""), 101325.0)));
}

TEST(prep_baro_reference_mean_count_variance)
{
    BaroReferenceCalibrator c{BaroCalibrationConfig{}};
    sim::Rng rng(14);
    c.begin(0);
    for (uint32_t t = 0; t < 5000u; t += 20u) c.add(t, static_cast<float>(95000.0 + rng.gaussian() * 1.5));
    CHECK(c.finish(5000, 3));
    CHECK_NEAR(c.active().referencePressure_pa, 95000.0, 0.4);
    CHECK_NEAR(c.active().pressureStd_pa, 1.5, 0.25);
    CHECK_EQ(c.active().samples, 250u);
    CHECK_EQ(c.active().sessionId, 3u);
    CHECK(c.state() == CalibrationState::Valid);
}

TEST(prep_baro_reference_unstable_and_drifting_rejected)
{
    BaroReferenceCalibrator noisy{BaroCalibrationConfig{}};
    sim::Rng rng(15);
    noisy.begin(0);
    for (uint32_t t = 0; t < 5000u; t += 20u) noisy.add(t, static_cast<float>(101000.0 + rng.gaussian() * 25.0));
    CHECK(!noisy.finish(5000, 1));
    CHECK(noisy.lastFailure() == CalibrationFailure::TooNoisy);

    // Carried up ~2 m in 5 s (24 Pa ramp: std 7 Pa passes, segment spread ~19 Pa fails).
    BaroReferenceCalibrator ramp{BaroCalibrationConfig{}};
    ramp.begin(0);
    for (uint32_t t = 0; t < 5000u; t += 20u) ramp.add(t, 101000.0f - 24.0f * static_cast<float>(t) / 5000.0f);
    CHECK(!ramp.finish(5000, 1));
    CHECK(ramp.lastFailure() == CalibrationFailure::Drifting);
    CHECK(ramp.state() == CalibrationState::Failed);

    // Weather-rate drift (0.01 Pa/s) is accepted.
    BaroReferenceCalibrator weather{BaroCalibrationConfig{}};
    weather.begin(0);
    for (uint32_t t = 0; t < 5000u; t += 20u) weather.add(t, 101000.0f + 0.01f * static_cast<float>(t) / 1000.0f);
    CHECK(weather.finish(5000, 1));
}

TEST(prep_baro_reference_insufficient_and_invalid_inputs)
{
    BaroReferenceCalibrator c{BaroCalibrationConfig{}};
    c.begin(0);
    for (uint32_t t = 0; t < 1000u; t += 20u) c.add(t, 101000.0f);
    c.add(1000, 0.0f);       // ignored
    c.add(1020, -3.0f);      // ignored
    c.add(1040, std::nanf(""));
    CHECK_EQ(c.collectedSamples(), 50u);
    CHECK(!c.finish(1100, 1));
    CHECK(c.lastFailure() == CalibrationFailure::InsufficientSamples);
    CHECK(!c.active().valid);
}

TEST(prep_session_two_baros_independent_references_with_offset)
{
    CalibrationController cc(PreprocessingConfig{});
    sim::SensorSimulator s(16);
    s.baro[1].offsetPa = 35.0;  // constant 35 Pa offset between the two devices
    s.baro[0].noisePa = 1.0;
    s.baro[1].noisePa = 1.0;
    cc.request(0);
    runController(cc, s, 0, 5200);
    CHECK(cc.status().state == CalibrationState::Valid);
    CHECK(cc.baro(0).active().valid);
    CHECK(cc.baro(1).active().valid);
    CHECK_NEAR(cc.baro(1).active().referencePressure_pa - cc.baro(0).active().referencePressure_pa, 35.0, 0.6);
    CHECK_EQ(cc.baro(0).active().sessionId, cc.baro(1).active().sessionId);
}

TEST(prep_session_one_invalid_baro_other_still_calibrates)
{
    CalibrationController cc(PreprocessingConfig{});
    sim::SensorSimulator s(17);
    s.baro[1].present = false;
    cc.request(0);
    runController(cc, s, 0, 5200);
    CHECK(cc.baro(0).active().valid);
    CHECK(!cc.baro(1).active().valid);
    CHECK(cc.baro(1).lastFailure() == CalibrationFailure::InsufficientSamples);
}

TEST(prep_session_interrupted_baro_samples_before_motion_discarded)
{
    CalibrationController cc(PreprocessingConfig{});
    sim::SensorSimulator s(18);
    // Pressure 50 Pa higher before t = 2000 (device was lower), then a shake while
    // being placed; the reference must only use samples after the motion.
    s.referencePressure = [](uint32_t t) { return t < 2000u ? 101375.0 : 101325.0; };
    s.angularRate = [](uint32_t t) { return (t >= 1900u && t < 2000u) ? Vec3f{0.8f, 0, 0} : Vec3f{0, 0, 0}; };
    cc.request(0);
    runController(cc, s, 0, 7500);
    CHECK(cc.baro(0).active().valid);
    CHECK_NEAR(cc.baro(0).active().referencePressure_pa, 101325.0, 0.5);
}
