/**
 * Vertical UKF milestone - end to end, exactly the firmware data path:
 *   simulated SensorMessages -> PreprocessingPipeline (validation, calibration,
 *   barometer reference, REAL 2-barometer voter, 3-IMU voter)
 *   -> ProcessedSensorFrame -> AttitudeEstimator (Madgwick, gravity compensation)
 *   -> navigationInputFrom(frame, attitude) -> VerticalNavigationEstimator
 *   -> NavigationMailbox.
 * Plus: GNSS NAV-PVT-derived samples through the pipeline, and the mailbox
 * under concurrent access (ThreadSanitizer run).
 */
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>

#include "TestFramework.hpp"
#include "sim/SensorSimulator.hpp"

#include "Algorithms/Attitude/AttitudeEstimator.hpp"
#include "Algorithms/Attitude/AttitudeFrameAdapter.hpp"
#include "Algorithms/Navigation/ConstantAccelerationModel.hpp"
#include "Algorithms/Navigation/NavigationInputAdapter.hpp"
#include "Algorithms/Navigation/NavigationMailbox.hpp"
#include "Algorithms/Navigation/VerticalNavigationEstimator.hpp"
#include "Processing/PreprocessingPipeline.hpp"

using namespace pg;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// Vertical truth: still until t1, then a = A sin(w (t - t1)) (climbs on average A / w).
struct Vertical {
    double t1 = 8.0, A = 1.0, w = 2 * kPi / 6.0, h0 = 300.0;
    double a(double t) const { return t < t1 ? 0.0 : A * std::sin(w * (t - t1)); }
    double v(double t) const { return t < t1 ? 0.0 : A / w * (1 - std::cos(w * (t - t1))); }
    double h(double t) const { return t < t1 ? h0 : h0 + A / w * ((t - t1) - std::sin(w * (t - t1)) / w); }
};

struct E2E {
    explicit E2E(uint64_t seed)
        : model(ConstantAccelerationModel::fromConfig(NavigationConfig{})), nav(NavigationConfig{}, model), s(seed)
    {
        for (int i = 0; i < 3; ++i) {
            s.imu[i].gyroNoise = 0.0012f;
            s.imu[i].accelNoise = 0.02f;
        }
        s.baro[0].noisePa = 2.0;   // ~0.17 m per barometer
        s.baro[1].noisePa = 2.0;
        s.specificForce = [this](uint32_t ms) {
            return Vec3f{0.0f, 0.0f, -static_cast<float>(units::kStandardGravity_mps2 + truth.a(ms * 1e-3))};
        };
        s.altitude = [this](uint32_t ms) { return truth.h(ms * 1e-3); };
        p.start(0);
    }
    Vertical truth;
    PreprocessingPipeline p;
    AttitudeEstimator att{AttitudeConfig{}};
    ConstantAccelerationModel model;
    VerticalNavigationEstimator nav;
    sim::SensorSimulator s;
    ProcessedSensorFrame f{};
    bool gnss = false;
    uint32_t gnssSeq = 0;
    double hRef = NAN;   ///< truth altitude when the barometer reference was taken

    void run(uint32_t t0, uint32_t ms, const std::function<void(uint32_t)>& each = nullptr)
    {
        for (uint32_t k = 0; k < ms / 10u; ++k) {
            const uint32_t t = t0 + k * 10u;
            for (const SensorMessage& m : s.step(t)) p.ingest(m, t + 1u);
            if (gnss && t % 200u == 0u) p.ingest(SensorMessage::fromGnss(gnssSample(t)), t + 1u);
            p.buildFrame(t + 1u, f);
            att.update(attitudeInputFromFrame(f));
            nav.update(navigationInputFrom(f, att.latest()));
            if (std::isnan(hRef) && f.baroAltitude_m.valid) hRef = truth.h(t * 1e-3) - f.baroAltitude_m.value;
            if (each) each(t);
        }
    }

    GnssSample gnssSample(uint32_t t)
    {
        GnssSample g{};
        g.header.id = SensorId::Gnss0;
        g.header.timestampMs = t;
        g.header.sequence = ++gnssSeq;
        g.header.valid = true;
        g.verticalVelocity_mps = static_cast<float>(truth.v(t * 1e-3));   // driver output: positive UP
        g.speedAccuracy_mps = 0.25f;
        g.horizontalAccuracy_m = 2.0f;
        g.verticalAccuracy_m = 3.0f;
        g.heightMsl_m = 300.0f;
        g.gnssTimeOfWeek_ms = 100000u + t;
        g.numSatellites = 12u;
        g.fixType = GnssFixType::Fix3D;
        g.gnssFixOk = true;
        g.positionValid = true;
        g.heightValid = true;
        return g;
    }
    const NavigationEstimate& e() const { return nav.latest(); }
};

}  // namespace

TEST(nav_e2e_pipeline_attitude_ukf_track_vertical_motion)
{
    E2E x(201);
    x.run(0, 6000);   // calibration (gyro bias + barometer reference) and levelling
    CHECK(x.f.calibrationState == CalibrationState::Valid);
    CHECK(x.att.latest().state == AttitudeState::Tracking6Axis);
    CHECK(x.e().state == NavState::Tracking);
    CHECK(x.e().has(navflag::AccelNotCalibrated));   // inherited from preprocessing, via attitude
    double maxH = 0, maxV = 0, sumA2 = 0;
    int n = 0;
    const uint32_t baroBefore = x.e().baro.accepted;
    x.run(6000, 30000, [&](uint32_t t) {
        if (t < 9000u) return;
        const NavigationEstimate& e = x.e();
        const double tt = e.timestampMs * 1e-3;
        maxH = std::fmax(maxH, std::fabs(e.altitude_m - (x.truth.h(tt) - x.hRef)));
        maxV = std::fmax(maxV, std::fabs(e.verticalVelocity_mps - x.truth.v(tt)));
        sumA2 += std::pow(e.verticalAccel_mps2 - x.truth.a(tt), 2);
        ++n;
    });
    const NavigationEstimate& e = x.e();
    std::printf("    [metric] E2E pipeline -> Madgwick -> UKF, 1 m/s^2 sinusoidal vertical motion: max |h err| %.3f m, "
                "max |vz err| %.3f m/s, az rmse %.3f m/s^2; baro observations in 30 s %u; state %d\n",
                maxH, maxV, std::sqrt(sumA2 / n), unsigned(e.baro.accepted - baroBefore), int(e.state));
    CHECK(e.state == NavState::Tracking);
    CHECK(e.altitudeValid && e.velocityValid && e.accelerationValid);
    CHECK(maxH < 1.0);
    CHECK(maxV < 0.4);
    CHECK(std::sqrt(sumA2 / n) < 0.3);
    // The real voter + tracker: one observation per fresh barometer pair (~50 per second).
    CHECK(e.baro.accepted - baroBefore >= 1450u && e.baro.accepted - baroBefore <= 1501u);
    CHECK_EQ(e.numericalFaults, 0u);
}

TEST(nav_e2e_gnss_vertical_velocity_through_pipeline)
{
    E2E x(202);
    x.gnss = true;
    x.run(0, 6000);
    const uint32_t before = x.e().gnss.accepted;
    x.run(6000, 20000);
    const NavigationEstimate& e = x.e();
    CHECK(e.gnss.accepted - before >= 95u);   // 5 Hz for 20 s, each solution once
    CHECK(e.gnss.notNew > 0u);                // republished in the frames between solutions
    CHECK(std::fabs(e.verticalVelocity_mps - x.truth.v(e.timestampMs * 1e-3)) < 0.3);
    std::printf("    [metric] E2E with GNSS 5 Hz: %u GNSS velocities assimilated in 20 s (%u republished copies skipped)\n",
                unsigned(e.gnss.accepted - before), unsigned(e.gnss.notNew));
}

TEST(nav_e2e_barometers_absent_until_reference_then_initialise)
{
    E2E x(203);
    x.run(0, 3000);   // calibrating: no barometric reference yet
    CHECK(!x.f.baroAltitude_m.valid);
    CHECK(x.e().state == NavState::Uninitialized);
    CHECK(!x.e().altitudeValid && std::isnan(x.e().altitude_m));
    x.run(3000, 4000);
    CHECK(x.f.baroAltitude_m.valid);
    CHECK(x.e().state == NavState::Tracking);
    CHECK(std::fabs(x.e().altitude_m) < 0.5f);   // relative to the reference just taken
}

TEST(nav_mailbox_concurrent_reads_are_consistent)
{
    struct MutexLock final : ICriticalSection {
        std::mutex m;
        void enter() override { m.lock(); }
        void exit() override { m.unlock(); }
    } lock;
    NavigationMailbox box(lock);
    NavigationEstimate probe{};
    CHECK(!box.read(probe));
    std::thread writer([&] {
        NavigationEstimate e{};
        for (uint32_t k = 1; k <= 20000u; ++k) {
            e.sequence = k;
            e.timestampMs = 10u * k;
            e.altitude_m = float(k);
            e.covariance[2][2] = float(k);
            e.baro.accepted = k;
            box.publish(e);
        }
    });
    uint32_t torn = 0u, last = 0u, backwards = 0u, reads = 0u;
    NavigationEstimate r{};
    while (last < 20000u) {
        if (box.read(r)) {
            ++reads;
            torn += (r.timestampMs != 10u * r.sequence || r.altitude_m != float(r.sequence) ||
                     r.covariance[2][2] != float(r.sequence) || r.baro.accepted != r.sequence) ? 1u : 0u;
            backwards += (r.sequence < last) ? 1u : 0u;
            last = r.sequence;
        }
    }
    writer.join();
    CHECK_EQ(torn, 0u);
    CHECK_EQ(backwards, 0u);
    CHECK(reads > 0u);
}
