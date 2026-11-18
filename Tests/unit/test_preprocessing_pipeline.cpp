/**
 * Preprocessing milestone - integration / end-to-end tests:
 * simulated SensorMessage stream -> PreprocessingPipeline -> ProcessedSensorFrame.
 *
 * Time model: the simulator produces each 10 ms acquisition cycle's messages
 * at t; the "processing task" ingests them at t + 1 ms and builds a frame at
 * t + 1 ms (it runs right after acquisition blocks).
 */
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <mutex>
#include <thread>

#include "TestFramework.hpp"
#include "sim/SensorSimulator.hpp"

#include "Acquisition/SampleQueue.hpp"
#include "Acquisition/SensorDataHub.hpp"
#include "Processing/PreprocessingPipeline.hpp"
#include "Processing/ProcessedFrameMailbox.hpp"

using namespace pg;

namespace {

constexpr float kG = units::kStandardGravity_mps2;

struct Harness {
    explicit Harness(uint64_t seed, const PreprocessingConfig& cfg = PreprocessingConfig{}) : p(cfg), s(seed) {}
    PreprocessingPipeline p;
    sim::SensorSimulator s;
    ProcessedSensorFrame f{};
    std::vector<SensorMessage> extra;  ///< injected before the next cycle's messages

    /// Runs cycles t = t0, t0+10, ... for @p ms; @p each is called after every frame.
    void run(uint32_t t0, uint32_t ms, const std::function<void(uint32_t)>& each = nullptr)
    {
        for (uint32_t k = 0; k < ms / 10u; ++k) {
            const uint32_t t = t0 + k * 10u;
            for (const SensorMessage& m : extra) p.ingest(m, t + 1u);
            extra.clear();
            for (const SensorMessage& m : s.step(t)) p.ingest(m, t + 1u);
            p.buildFrame(t + 1u, f);
            if (each) each(t);
        }
    }
    void realisticNoise()
    {
        const Vec3f biases[3] = {{0.012f, -0.020f, 0.006f}, {-0.025f, 0.009f, 0.015f}, {0.004f, 0.031f, -0.018f}};
        for (int i = 0; i < 3; ++i) {
            s.imu[i].gyroBias = biases[i];
            s.imu[i].gyroNoise = 0.0012f;
            s.imu[i].accelNoise = 0.01f;
        }
        s.baro[0].noisePa = 1.0;
        s.baro[1].noisePa = 1.0;
    }
};

bool near(const Vec3f& a, const Vec3f& b, float tol)
{
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol;
}

}  // namespace

TEST(prep_e2e_startup_calibration_then_calibrated_body_outputs)
{
    Harness h(31);
    h.realisticNoise();
    h.p.start(0);

    h.run(0, 2000);
    // Before calibration: accel usable, gyro NOT (no bias yet) - and said so.
    CHECK(h.f.specificForce_mps2.valid);
    CHECK(h.f.specificForce_mps2.status == RedundancyStatus::Healthy);
    CHECK(!h.f.angularRate_radps.valid);
    CHECK(h.f.has(quality::GyroNotCalibrated));
    CHECK(h.f.has(quality::CalibrationInProgress));
    CHECK(h.f.calibrationState == CalibrationState::Collecting);

    h.run(2000, 3200);
    CHECK(h.f.calibrationState == CalibrationState::Valid);
    CHECK(!h.f.has(quality::CalibrationInProgress));
    CHECK(!h.f.has(quality::GyroNotCalibrated));
    for (int i = 0; i < 3; ++i) CHECK(h.f.imu[i].gyroCalibration == CalibrationState::Valid);
    for (int j = 0; j < 2; ++j) CHECK(h.f.baro[j].referenceState == CalibrationState::Valid);

    // Now rotate (body frame truth) and check the calibrated, fused output.
    const Vec3f w{0.20f, -0.10f, 0.30f};
    h.s.angularRate = [w](uint32_t) { return w; };
    float worstGyro = 0.0f;
    h.run(5200, 1000, [&](uint32_t) {
        CHECK(h.f.angularRate_radps.valid);
        worstGyro = std::fmax(worstGyro, vec::distance(h.f.angularRate_radps.value, w));
    });
    CHECK(worstGyro < 0.004f);  // 3-sample mean of noise 0.0012 + residual bias
    CHECK(h.f.angularRate_radps.status == RedundancyStatus::Healthy);
    CHECK_EQ(h.f.angularRate_radps.contributingMask, 0x7u);
    CHECK(near(h.f.specificForce_mps2.value, Vec3f{0, 0, -kG}, 0.03f));
    CHECK(h.f.baroAltitude_m.valid);
    CHECK_NEAR(h.f.baroAltitude_m.value, 0.0, 0.3);
    CHECK(h.f.has(quality::MountingUnverified));
    CHECK(h.f.has(quality::AccelNotCalibrated));
    CHECK(h.f.has(quality::MagNotIronCalibrated));
    CHECK(h.f.bodyFrame == BodyFrame::FrdBody);
    CHECK(h.f.gyroDtValid);
    CHECK_EQ(h.f.gyroDtMs, 10u);
    std::printf("    worst fused gyro error after calibration: %.2e rad/s\n", static_cast<double>(worstGyro));
}

TEST(prep_e2e_barometric_altitude_tracks_climb_with_offset_between_baros)
{
    Harness h(32);
    h.realisticNoise();
    h.s.baro[1].offsetPa = 40.0;  // ~3.4 m apparent difference in ABSOLUTE terms
    h.p.start(0);
    h.run(0, 5400);
    CHECK(h.f.calibrationState == CalibrationState::Valid);
    // Climb 100 m in 20 s (5 m/s, a strong thermal).
    h.s.altitude = [](uint32_t t) { return t > 5400u ? 5.0 * (t - 5400u) * 1e-3 : 0.0; };
    float worst = 0.0f;
    h.run(5400, 20000, [&](uint32_t t) {
        CHECK(h.f.baroAltitude_m.valid);
        const double truth = 5.0 * ((h.f.baroAltitude_m.timestampMs > 5400u) ? h.f.baroAltitude_m.timestampMs - 5400u : 0u) * 1e-3;
        // Two barometers may be 10 ms apart (5 cm at 5 m/s): compare with the newer one's time.
        worst = std::fmax(worst, static_cast<float>(std::fabs(h.f.baroAltitude_m.value - truth)));
        (void)t;
    });
    CHECK(worst < 0.5f);
    CHECK(h.f.baroAltitude_m.status == RedundancyStatus::Healthy);  // constant offset cancelled by per-baro P0
    CHECK_EQ(h.f.baroAltitude_m.contributingMask, 0x3u);
    CHECK_NEAR(h.f.baroAltitude_m.value, 100.0, 0.6);
    std::printf("    climb: worst altitude error %.3f m over 100 m\n", static_cast<double>(worst));
}

TEST(prep_e2e_slow_weather_drift_appears_as_altitude_change)
{
    // Documented limitation, demonstrated: pressure falls 1 hPa (weather) while the
    // device does not move; the barometric altitude rises ~8 m. Preprocessing
    // cannot tell weather from climb; a future estimator must (GNSS / model).
    Harness h(33);
    h.p.start(0);
    h.run(0, 5400);
    h.s.referencePressure = [](uint32_t t) { return 101325.0 - (t > 5400u ? 100.0 * (t - 5400u) / 60000.0 : 0.0); };
    h.run(5400, 60000);
    CHECK(h.f.baroAltitude_m.valid);
    CHECK_NEAR(h.f.baroAltitude_m.value, 8.3, 0.5);
}

TEST(prep_e2e_biased_imu_detected_excluded_output_unaffected)
{
    Harness h(34);
    h.realisticNoise();
    h.p.start(0);
    h.run(0, 5400);
    CHECK(h.f.angularRate_radps.valid);
    h.s.imu[1].gyroFaultBias = Vec3f{0.3f, 0.0f, 0.0f};  // IMU1 develops a 17 dps error
    int frame = 0;
    h.run(5400, 300, [&](uint32_t) {
        ++frame;
        CHECK(h.f.angularRate_radps.valid);
        CHECK(near(h.f.angularRate_radps.value, Vec3f{0, 0, 0}, 0.005f));  // never pulled by the fault
        // Frames 1..5: IMU1 is this frame's outlier (left out of the value).
        // From the 5th consecutive outlier on it is EXCLUDED - no longer a voter,
        // so it appears in excludedMask instead of outlierMask.
        if (frame <= 5) CHECK_EQ(h.f.angularRate_radps.outlierMask, 0x2u);
        if (frame >= 5) CHECK_EQ(h.f.angularRate_radps.excludedMask, 0x2u);
        if (frame > 5) CHECK_EQ(h.f.angularRate_radps.outlierMask, 0u);
    });
    CHECK(h.f.angularRate_radps.status == RedundancyStatus::Degraded);
    CHECK(h.f.has(quality::ImuDegraded));
    CHECK_EQ(h.f.angularRate_radps.contributingMask, 0x5u);
    // Accelerometers of IMU1 are fine: still Healthy with all three.
    CHECK(h.f.specificForce_mps2.status == RedundancyStatus::Healthy);
    // Recovery: fault disappears -> re-included after the hysteresis.
    h.s.imu[1].gyroFaultBias = Vec3f{0, 0, 0};
    h.run(5700, 300);
    CHECK_EQ(h.f.angularRate_radps.contributingMask, 0x7u);
    CHECK(h.f.angularRate_radps.status == RedundancyStatus::Healthy);
}

TEST(prep_e2e_two_imus_fail_same_way_common_mode_limit)
{
    // Honest limit: if TWO IMUs share the same fault, the majority is wrong and the
    // healthy one is out-voted. The voter reports the healthy one as the outlier.
    Harness h(35);
    h.p.start(0);
    h.run(0, 5400);
    h.s.imu[0].gyroFaultBias = Vec3f{0.3f, 0, 0};
    h.s.imu[1].gyroFaultBias = Vec3f{0.3f, 0, 0};
    h.run(5400, 100);
    CHECK(h.f.angularRate_radps.valid);
    // After 5 frames the HEALTHY IMU2 has been voted out (excluded) ...
    CHECK_EQ(h.f.angularRate_radps.excludedMask, 0x4u);
    CHECK_EQ(h.p.gyroVoter().exclusions(2), 1u);
    // ... and the output follows the faulty pair: wrong, but self-consistent.
    CHECK_NEAR(h.f.angularRate_radps.value.x, 0.3, 0.01);
}

TEST(prep_e2e_all_imus_disagree_no_output)
{
    Harness h(36);
    h.p.start(0);
    h.run(0, 5400);
    h.s.imu[0].accelFaultBias = Vec3f{3.0f, 0, 0};
    h.s.imu[1].accelFaultBias = Vec3f{-3.0f, 0, 0};
    h.run(5400, 50);
    CHECK(!h.f.specificForce_mps2.valid);
    CHECK(h.f.specificForce_mps2.status == RedundancyStatus::Ambiguous);
    CHECK(h.f.has(quality::ImuAmbiguous));
}

TEST(prep_e2e_baro_disagreement_ambiguous_and_unavailable_recovery)
{
    Harness h(37);
    h.p.start(0);
    h.run(0, 5400);
    CHECK(h.f.baroAltitude_m.status == RedundancyStatus::Healthy);
    h.s.baro[1].faultPa = 60.0;  // ~5 m disagreement
    h.run(5400, 100);
    CHECK(!h.f.baroAltitude_m.valid);
    CHECK(h.f.baroAltitude_m.status == RedundancyStatus::Ambiguous);
    CHECK(h.f.has(quality::BaroAmbiguous));
    h.s.baro[1].faultPa = 0.0;
    h.run(5500, 100);
    CHECK(h.f.baroAltitude_m.status == RedundancyStatus::Healthy);

    // Barometer 1 disappears, then returns.
    h.s.baro[1].present = false;
    h.run(5600, 300);
    CHECK(h.f.baroAltitude_m.valid);
    CHECK(h.f.baroAltitude_m.status == RedundancyStatus::Degraded);
    CHECK_EQ(h.f.baroAltitude_m.contributingMask, 0x1u);
    CHECK(h.f.baro[1].present);
    CHECK(!h.f.baro[1].fresh);
    h.s.baro[1].present = true;
    int changed = 0;
    h.run(5900, 600, [&](uint32_t) { changed += h.f.has(quality::BaroSourceChanged) ? 1 : 0; });
    CHECK(h.f.baroAltitude_m.status == RedundancyStatus::Healthy);
    CHECK_EQ(changed, 1);  // flagged exactly once, when it rejoined
}

TEST(prep_e2e_all_sensors_stop_frames_report_missing_not_zero)
{
    Harness h(38);
    h.p.start(0);
    h.run(0, 5400);
    for (int i = 0; i < 3; ++i) h.s.imu[i].present = false;
    for (int j = 0; j < 2; ++j) h.s.baro[j].present = false;
    h.run(5400, 100);
    CHECK(!h.f.specificForce_mps2.valid);
    CHECK(!h.f.angularRate_radps.valid);
    CHECK(!h.f.baroAltitude_m.valid);
    CHECK(h.f.has(quality::ImuUnavailable));
    CHECK(h.f.has(quality::BaroUnavailable));
    CHECK(h.f.has(quality::NoNewImuData));
    CHECK(!h.f.imu[0].fresh);
    CHECK(h.f.imu[0].ageMs > 30u);
    CHECK(!h.f.gyroDtValid);
    CHECK(h.p.stats().framesWithoutImu > 0u);
}

TEST(prep_e2e_delayed_samples_stale_or_skewed)
{
    Harness h(39);
    h.p.start(0);
    h.run(0, 5400);
    h.s.imu[2].timestampDelayMs = 20u;  // fresh (< 30 ms) but 20 ms behind the others (> 15 ms skew)
    h.run(5400, 50);
    CHECK_EQ(h.f.specificForce_mps2.candidateMask, 0x3u);
    CHECK(h.f.specificForce_mps2.status == RedundancyStatus::Degraded);
    h.s.imu[2].timestampDelayMs = 45u;  // stale at ingest: rejected with reason Stale
    h.run(5450, 50);
    CHECK((h.f.imu[2].lastRejectReasons & reject::Stale) != 0u);
    CHECK(!h.f.imu[2].fresh);
}

TEST(prep_e2e_nan_and_invalid_samples_excluded_with_reasons)
{
    Harness h(40);
    h.p.start(0);
    h.run(0, 5400);
    h.s.imu[0].nanOutput = true;
    h.s.baro[0].invalidFlag = true;
    h.run(5400, 100);
    CHECK((h.f.imu[0].lastRejectReasons & reject::NonFinite) != 0u);
    CHECK((h.f.baro[0].lastRejectReasons & reject::FlagInvalid) != 0u);
    CHECK(h.f.has(quality::SamplesRejected));
    CHECK(h.f.specificForce_mps2.valid);
    CHECK_EQ(h.f.specificForce_mps2.contributingMask, 0x6u);
    CHECK(h.f.baroAltitude_m.valid);
    CHECK_EQ(h.f.baroAltitude_m.contributingMask, 0x2u);
    CHECK(vec::isFinite(h.f.specificForce_mps2.value));
}

TEST(prep_e2e_random_outliers_do_not_reach_output_or_deselect)
{
    Harness h(41);
    h.realisticNoise();
    h.p.start(0);
    h.run(0, 5400);
    h.s.imu[0].outlierProbability = 0.03;
    h.s.imu[0].outlierMagnitude = 6.0f;
    float worst = 0.0f;
    h.run(5400, 10000, [&](uint32_t) {
        if (h.f.specificForce_mps2.valid) {
            worst = std::fmax(worst, vec::distance(h.f.specificForce_mps2.value, Vec3f{0, 0, -kG}));
        }
    });
    CHECK(worst < 0.05f);
    CHECK_EQ(h.p.accelVoter().exclusions(0), 0u);  // isolated outliers never reach 5 in a row
}

TEST(prep_e2e_mixed_rates_jitter_and_missing_readings)
{
    Harness h(42);
    h.s.mag.present = true;
    h.s.lidar.present = true;
    h.s.imu[1].missProbability = 0.2;   // 20 % of IMU1 reads fail (no sample)
    h.s.baro[0].missProbability = 0.1;
    h.p.start(0);
    int imuValid = 0, frames = 0;
    h.run(0, 8000, [&](uint32_t t) {
        if (t < 6000u) return;
        ++frames;
        imuValid += h.f.specificForce_mps2.valid ? 1 : 0;
    });
    CHECK_EQ(imuValid, frames);              // a missing read of one IMU never removes the output
    CHECK(h.f.calibrationState == CalibrationState::Valid);
    CHECK(h.f.mag.valid);
    CHECK(h.f.lidar.valid);
    CHECK_NEAR(h.f.lidar.distance_m, 12.5, 1e-6);
    CHECK(!h.f.has(quality::SamplesLost));   // missing reads are not "lost": no sequence gap

    // GNSS: separate time domains, own staleness (2 s).
    GnssSample g{};
    g.header = SampleHeader{SensorId::Gnss0, 8000u, 1u, true};
    g.gnssFixOk = true;
    g.positionValid = true;
    g.fixType = GnssFixType::Fix3D;
    g.gnssTimeOfWeek_ms = 345600123u;
    h.extra.push_back(SensorMessage::fromGnss(g));
    h.run(8000, 10);
    CHECK(h.f.gnss.valid);
    CHECK_EQ(h.f.gnss.timeOfWeek_ms, 345600123u);  // passed through, never converted
    CHECK_EQ(h.f.gnss.timestampMs, 8000u);
    h.run(8010, 2100);
    CHECK(!h.f.gnss.valid);
    CHECK(h.f.has(quality::GnssUnavailable));
}

TEST(prep_e2e_timestamp_jitter_tolerated_and_dt_follows_samples)
{
    // Read-completion times jitter by 0..4 ms (bus contention, rotating service
    // order). Nothing may be rejected and dt must follow the real timestamps.
    Harness h(47);
    for (int i = 0; i < 3; ++i) h.s.imu[i].timestampJitterMs = 4u;
    h.p.start(0);
    h.run(0, 5400);
    uint32_t minDt = 1000u, maxDt = 0u, dtValid = 0u, frames = 0u;
    h.run(5400, 2000, [&](uint32_t) {
        ++frames;
        CHECK(h.f.angularRate_radps.status == RedundancyStatus::Healthy);
        if (h.f.gyroDtValid) {
            ++dtValid;
            minDt = (h.f.gyroDtMs < minDt) ? h.f.gyroDtMs : minDt;
            maxDt = (h.f.gyroDtMs > maxDt) ? h.f.gyroDtMs : maxDt;
        }
    });
    CHECK_EQ(h.p.stats().messagesRejected, 0u);
    CHECK_EQ(dtValid, frames);
    CHECK(minDt >= 6u);
    CHECK(maxDt <= 14u);
    CHECK(maxDt > minDt);  // dt is measured, not assumed to be 10 ms
    CHECK(h.f.angularRate_radps.sampleSpreadMs <= 4u);
}

TEST(prep_concurrency_frame_mailbox_never_torn)
{
    struct MutexLock final : ICriticalSection {
        std::mutex m;
        void enter() override { m.lock(); }
        void exit() override { m.unlock(); }
    } lock;
    ProcessedFrameMailbox box(lock);
    std::thread writer([&] {
        ProcessedSensorFrame f{};
        for (uint32_t k = 1; k <= 20000u; ++k) {
            f.sequence = k;
            f.timestampMs = k * 10u;            // invariant: timestamp == 10 * sequence
            f.samplesLostSinceLastFrame = k;    // invariant: equals sequence
            box.publish(f);
        }
    });
    uint32_t torn = 0u, reads = 0u, last = 0u, backwards = 0u;
    ProcessedSensorFrame r{};
    while (last < 20000u) {
        if (box.read(r)) {
            ++reads;
            torn += (r.timestampMs != r.sequence * 10u || r.samplesLostSinceLastFrame != r.sequence) ? 1u : 0u;
            backwards += (r.sequence < last) ? 1u : 0u;
            last = r.sequence;
        }
    }
    writer.join();
    CHECK_EQ(torn, 0u);
    CHECK_EQ(backwards, 0u);
    CHECK(reads > 0u);
    CHECK_EQ(box.writes(), 20000u);
}

TEST(prep_e2e_queue_loss_with_drop_oldest_hub)
{
    // Real hub + bounded queue between "acquisition" and "processing"; the consumer
    // only drains every 50 ms (a stalled task). DropOldest keeps the newest data.
    Harness h(43);
    SensorDataHub hub;
    BoundedSampleQueue<16> q;
    CHECK(hub.subscribe(&q, OverflowPolicy::DropOldest));
    h.p.start(0);
    uint32_t validAfterDrain = 0, drains = 0;
    for (uint32_t t = 0; t < 8000u; t += 10u) {
        for (const SensorMessage& m : h.s.step(t)) hub.publish(m);
        if (t % 50u == 40u) {
            SensorMessage m;
            while (q.tryPop(m)) h.p.ingest(m, t + 1u);
            h.p.buildFrame(t + 1u, h.f);
            ++drains;
            validAfterDrain += h.f.specificForce_mps2.valid ? 1u : 0u;
        }
    }
    CHECK(hub.overwritesFor(0) > 0u);          // queue overflowed and dropped the oldest
    CHECK(h.p.stats().samplesLost > 0u);       // ...and the pipeline saw it as sequence gaps
    CHECK_EQ(validAfterDrain, drains);         // newest samples always fresh enough
    // Only ~70 of 100 IMU samples/s survive (stale ones are rejected), so the
    // 400-sample minimum is reached LATER than 5 s: the window stretches rather
    // than accepting a sparse one, and calibration still completes.
    CHECK(h.f.calibrationState == CalibrationState::Valid);
    CHECK(h.p.calibration().status().lastCompletedMs > 5500u);
    std::printf("    stalled consumer: hub overwrites %u, pipeline lost %u, rejected %u, calibrated at %u ms\n",
                hub.overwritesFor(0), h.p.stats().samplesLost, h.p.stats().messagesRejected,
                h.p.calibration().status().lastCompletedMs);
}

TEST(prep_e2e_repeated_calibration_request_keeps_old_bias_meanwhile)
{
    Harness h(44);
    h.realisticNoise();
    h.p.start(0);
    h.run(0, 5400);
    CHECK_EQ(h.f.calibrationSessionId, 1u);
    h.p.requestCalibration(5401);
    h.run(5400, 100);
    CHECK(h.f.has(quality::CalibrationInProgress));
    CHECK(h.f.angularRate_radps.valid);  // previous bias still applied while collecting
    CHECK_EQ(h.f.calibrationSessionId, 2u);
    h.run(5500, 5200);
    CHECK(h.f.calibrationState == CalibrationState::Valid);
    CHECK_EQ(h.p.calibration().baro(0).active().sessionId, 2u);
    CHECK_EQ(h.p.calibration().status().completedSessions, 2u);
}

TEST(prep_e2e_runs_across_timestamp_wraparound)
{
    Harness h(45);
    const uint32_t t0 = 0xFFFFFFFFu - 2999u - 9u;  // wrap happens 3 s into calibration
    h.p.start(t0 + 1u);
    h.run(t0 - (t0 % 10u), 7000);
    CHECK(h.f.calibrationState == CalibrationState::Valid);
    CHECK(h.f.specificForce_mps2.valid);
    CHECK(h.f.angularRate_radps.valid);
    CHECK(h.f.baroAltitude_m.valid);
    CHECK(h.f.gyroDtValid);
    CHECK_EQ(h.f.gyroDtMs, 10u);
    CHECK_EQ(h.p.stats().messagesRejected, 0u);
}

TEST(prep_e2e_memory_footprint_and_host_timing)
{
    std::printf("    sizeof: pipeline %zu B, frame %zu B, validator %zu B, calib %zu B, voter %zu B\n",
                sizeof(PreprocessingPipeline), sizeof(ProcessedSensorFrame), sizeof(SampleValidator),
                sizeof(CalibrationController), sizeof(RedundancyVoter));
    CHECK(sizeof(PreprocessingPipeline) < 8192u);
    CHECK(sizeof(ProcessedSensorFrame) < 1024u);

    Harness h(46);
    h.realisticNoise();
    h.s.mag.present = true;
    h.s.lidar.present = true;
    h.p.start(0);
    // Pre-generate 20 s of messages so only the pipeline is timed.
    std::vector<std::vector<SensorMessage>> cycles;
    for (uint32_t t = 0; t < 20000u; t += 10u) cycles.push_back(h.s.step(t));
    size_t msgs = 0;
    const auto start = std::chrono::steady_clock::now();
    for (size_t k = 0; k < cycles.size(); ++k) {
        const uint32_t t = static_cast<uint32_t>(k * 10u);
        for (const SensorMessage& m : cycles[k]) {
            h.p.ingest(m, t + 1u);
            ++msgs;
        }
        h.p.buildFrame(t + 1u, h.f);
    }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    std::printf("    host: %zu cycles, %zu messages: %.2f us per cycle (ingest + frame), %.3f us per message avg"
                " (sanitizer build if enabled)\n",
                cycles.size(), msgs, us / static_cast<double>(cycles.size()), us / static_cast<double>(msgs));
    CHECK(h.f.calibrationState == CalibrationState::Valid);
}
