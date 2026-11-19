/**
 * Attitude milestone - end to end:
 *   simulated SensorMessages -> PreprocessingPipeline (validation, gyro-bias
 *   calibration, alignment, 3-IMU voting) -> ProcessedSensorFrame ->
 *   attitudeInputFromFrame -> AttitudeEstimator -> AttitudeMailbox.
 * Truth from sim/AttitudeSimulator.hpp drives sim::SensorSimulator.
 */
#include <cmath>
#include <functional>
#include <mutex>
#include <thread>

#include "TestFramework.hpp"
#include "sim/AttitudeSimulator.hpp"
#include "sim/SensorSimulator.hpp"

#include "Algorithms/Attitude/AttitudeEstimator.hpp"
#include "Algorithms/Attitude/AttitudeFrameAdapter.hpp"
#include "Algorithms/Attitude/AttitudeMailbox.hpp"
#include "Processing/PreprocessingPipeline.hpp"

using namespace pg;
using attsim::M3;
using attsim::V3;

namespace {

struct E2E {
    explicit E2E(uint64_t seed, const AttitudeConfig& acfg = AttitudeConfig{}) : est(acfg), s(seed)
    {
        const Vec3f biases[3] = {{0.012f, -0.020f, 0.006f}, {-0.025f, 0.009f, 0.015f}, {0.004f, 0.031f, -0.018f}};
        for (int i = 0; i < 3; ++i) {
            s.imu[i].gyroBias = biases[i];  // removed by the milestone-2 calibration
            s.imu[i].gyroNoise = 0.0012f;
            s.imu[i].accelNoise = 0.01f;
        }
        s.baro[0].noisePa = 1.0;
        s.baro[1].noisePa = 1.0;
        s.angularRate = [this](uint32_t) { return attsim::toF(truth.omega()); };
        s.specificForce = [this](uint32_t) { return attsim::toF(truth.specificForceBody()); };
        p.start(0);
    }
    PreprocessingPipeline p;
    AttitudeEstimator est;
    sim::SensorSimulator s;
    attsim::AttitudeTruth truth;
    ProcessedSensorFrame f{};
    uint32_t extraBuilds = 0u;  ///< republish the frame this many extra times per cycle

    /// One acquisition cycle every 10 ms from @p t0 for @p ms milliseconds.
    void run(uint32_t t0, uint32_t ms, const std::function<void(uint32_t)>& each = nullptr)
    {
        for (uint32_t k = 0; k < ms / 10u; ++k) {
            const uint32_t t = t0 + k * 10u;
            truth.advanceTo(t * 1e-3);
            s.magneticFieldBody = attsim::toF(truth.magBody());
            for (const SensorMessage& m : s.step(t)) p.ingest(m, t + 1u);
            p.buildFrame(t + 1u, f);
            est.update(attitudeInputFromFrame(f));
            for (uint32_t r = 0; r < extraBuilds; ++r) {  // e.g. idle-timeout frames with no new data
                p.buildFrame(t + 2u + r, f);
                est.update(attitudeInputFromFrame(f));
            }
            if (each) each(t);
        }
    }
    const AttitudeEstimate& e() const { return est.latest(); }
    double err() const { return attsim::quatErrorDeg(truth.R, e().q); }
};

}  // namespace

TEST(att_e2e_calibrate_level_then_track_90_degree_roll)
{
    E2E x(101);
    x.run(0, 2000);
    // Gyro not calibrated yet: no attitude, said so, nothing fabricated.
    CHECK(x.e().state == AttitudeState::Uninitialized);
    CHECK(x.e().has(attitude::GyroNotCalibrated) && x.e().has(attitude::NotInitialized));
    CHECK(!x.e().linearAccel.valid);
    x.run(2000, 4000);  // calibration (5 s) completes, then levelling (0.5 s)
    CHECK(x.f.calibrationState == CalibrationState::Valid);
    CHECK(x.e().state == AttitudeState::Tracking6Axis);
    x.truth.omegaBody = [](double t, const M3&) { return V3{attsim::smoothRate(t, 7.0, 1.0, attsim::kPi / 2), 0, 0}; };
    attsim::Stats e;
    x.run(6000, 4000, [&](uint32_t) { e.add(x.err()); });
    CHECK(x.err() < 2.0);
    CHECK(x.e().has(attitude::MountingUnverified));   // inherited from preprocessing
    CHECK(x.e().has(attitude::AccelNotCalibrated));
    CHECK(x.e().has(attitude::HeadingUnreferenced));
    CHECK_EQ(x.e().gyroSourceMask, 0x7);
    CHECK(x.e().linearAccel.valid && x.e().linearAccel.reliable);
    CHECK(std::fabs(x.e().linearAccel.vertical_mps2) < 0.2f);
    std::printf("    [metric] E2E (pipeline, 3 IMUs, per-IMU biases calibrated) 90-deg roll: final %.3f deg, max %.3f deg, "
                "vertical accel %.4f m/s^2\n",
                x.err(), e.max, double(x.e().linearAccel.vertical_mps2));
}

TEST(att_e2e_republished_frames_not_integrated_twice)
{
    auto rot = [](double t, const M3&) {
        return V3{attsim::smoothRate(t, 6.5, 2.0, 1.0), attsim::smoothRate(t, 6.5, 2.0, -0.5), 0};
    };
    E2E a(102), b(102);
    a.truth.omegaBody = rot;
    b.truth.omegaBody = rot;
    b.extraBuilds = 2u;
    a.run(0, 10000);
    b.run(0, 10000);
    CHECK(b.e().notNewSamples >= 2u * 300u);
    CHECK(a.e().q.w == b.e().q.w && a.e().q.x == b.e().q.x && a.e().q.y == b.e().q.y && a.e().q.z == b.e().q.z);
    CHECK(a.err() < 2.0);
}

TEST(att_e2e_faulty_imu_excluded_attitude_unaffected)
{
    E2E x(103);
    x.run(0, 6000);
    x.s.imu[1].gyroFaultBias = Vec3f{0.3f, 0.0f, 0.0f};  // 17 deg/s fault after calibration
    x.truth.omegaBody = [](double t, const M3&) { return V3{0, 0, attsim::smoothRate(t, 7.0, 2.0, attsim::kPi / 2)}; };
    attsim::Stats e;
    x.run(6000, 5000, [&](uint32_t) { e.add(x.err()); });
    CHECK(x.e().gyroSourceMask == 0x5);  // IMU 1 voted out
    CHECK(x.err() < 2.0);
    CHECK(e.max < 2.0);
    std::printf("    [metric] E2E IMU1 gyro fault 0.3 rad/s: voted out (mask 0x%x), attitude error max %.3f deg\n",
                unsigned(x.e().gyroSourceMask), e.max);
}

TEST(att_e2e_moving_at_startup_never_fabricates_attitude)
{
    E2E x(104);
    x.truth.omegaBody = [](double t, const M3&) { return V3{0, 0, t < 10.0 ? 0.3 : 0.0}; };
    bool everValid = false;
    x.run(0, 10000, [&](uint32_t) { everValid = everValid || x.e().orientationValid; });
    CHECK(!everValid);  // calibration impossible while turning -> no gyro -> no attitude
    CHECK(x.e().has(attitude::GyroNotCalibrated));
    x.p.requestCalibration(10000);
    x.run(10000, 7000);
    CHECK(x.f.calibrationState == CalibrationState::Valid);
    CHECK(x.e().orientationValid);
    CHECK(attsim::tiltErrorDeg(x.truth.R, x.e().q) < 1.0);
}

TEST(att_e2e_uncalibrated_magnetometer_in_frame_keeps_6axis)
{
    E2E x(105);
    x.s.mag.present = true;
    x.run(0, 8000);
    CHECK(x.f.mag.valid && !x.f.mag.ironCalibrated);
    CHECK(x.e().magCorrection == CorrectionStatus::Disabled);
    CHECK(x.e().heading == HeadingReference::Unreferenced);
    CHECK(x.e().state == AttitudeState::Tracking6Axis);
}

TEST(att_frame_adapter_maps_fields)
{
    ProcessedSensorFrame f{};
    f.timestampMs = 500u;
    f.angularRate_radps.value = Vec3f{1, 2, 3};
    f.angularRate_radps.timestampMs = 498u;
    f.angularRate_radps.valid = true;
    f.angularRate_radps.contributingMask = 0x3;
    f.angularRate_radps.status = RedundancyStatus::Degraded;
    f.specificForce_mps2.value = Vec3f{0, 0, -9};
    f.specificForce_mps2.timestampMs = 497u;
    f.specificForce_mps2.valid = true;
    f.mag.fieldBody_uT = Vec3f{20, 0, 45};
    f.mag.timestampMs = 480u;
    f.mag.valid = true;
    f.mag.ironCalibrated = false;
    f.qualityFlags = quality::MountingUnverified | quality::GyroNotCalibrated;
    const AttitudeInput in = attitudeInputFromFrame(f);
    CHECK_EQ(in.nowMs, 500u);
    CHECK_EQ(in.gyroTimestampMs, 498u);
    CHECK(in.gyroValid && !in.gyroCalibrated);
    CHECK_EQ(in.gyroSourceMask, 0x3);
    CHECK(in.gyroStatus == RedundancyStatus::Degraded);
    CHECK_EQ(in.accelTimestampMs, 497u);
    CHECK(in.magValid && !in.magCalibrated);
    CHECK_EQ(in.magTimestampMs, 480u);
    CHECK(!in.mountingVerified && in.accelCalibrated);
}

TEST(att_mailbox_concurrent_reads_are_consistent)
{
    struct MutexLock final : ICriticalSection {
        std::mutex m;
        void enter() override { m.lock(); }
        void exit() override { m.unlock(); }
    } lock;
    AttitudeMailbox box(lock);
    AttitudeEstimate probe{};
    CHECK(!box.read(probe));
    std::thread writer([&] {
        AttitudeEstimate e{};
        for (uint32_t k = 1; k <= 20000u; ++k) {
            e.sequence = k;
            e.timestampMs = 10u * k;       // invariant
            e.q = Quaternion{float(k), 0, 0, 0};
            e.gaps = k;                    // invariant
            box.publish(e);
        }
    });
    uint32_t torn = 0u, last = 0u, backwards = 0u, reads = 0u;
    AttitudeEstimate r{};
    while (last < 20000u) {
        if (box.read(r)) {
            ++reads;
            torn += (r.timestampMs != 10u * r.sequence || r.gaps != r.sequence || r.q.w != float(r.sequence)) ? 1u : 0u;
            backwards += (r.sequence < last) ? 1u : 0u;
            last = r.sequence;
        }
    }
    writer.join();
    CHECK_EQ(torn, 0u);
    CHECK_EQ(backwards, 0u);
    CHECK(reads > 0u);
}
