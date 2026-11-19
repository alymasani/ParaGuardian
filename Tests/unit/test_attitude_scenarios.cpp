/**
 * Attitude milestone - simulation scenarios 1-17 (brief section 14) and the
 * gain / rejection sensitivity sweeps (section 15). Each test prints its
 * measured errors as "[metric]" lines; MADGWICK_VALIDATION_REPORT.md quotes them.
 *
 * Provisional development targets (NOT flight guarantees):
 *   stationary orientation after convergence  < 1 deg
 *   controlled 90-degree rotation, final       < 2 deg
 *   stationary gravity compensation            < 0.2 m/s^2
 * Noisy / sustained-acceleration scenarios report performance and assert only
 * properties (no NaN, rejection engaged, rejection better than none).
 */
#include <cmath>
#include <functional>
#include <vector>

#include "TestFramework.hpp"
#include "sim/AttitudeSimulator.hpp"

#include "Algorithms/Attitude/AttitudeEstimator.hpp"

using namespace pg;
using attsim::M3;
using attsim::V3;

namespace {

constexpr double kTargetStationaryDeg = 1.0;
constexpr double kTargetRotationDeg = 2.0;
constexpr double kTargetGravityComp = 0.2;

struct Run {
    explicit Run(const AttitudeConfig& cfg = AttitudeConfig{}, uint64_t seed = 5u) : est(cfg), h(seed) {}
    AttitudeEstimator est;
    attsim::AttitudeHarness h;
    bool sawNan = false;
    bool sawFalseHealthy = false;

    void step(AttitudeInput in)
    {
        est.update(in);
        const AttitudeEstimate& e = est.latest();
        if (e.orientationValid && !attsim::finite(e.q)) sawNan = true;
        if (e.linearAccel.valid && !std::isfinite(e.linearAccel.vertical_mps2)) sawNan = true;
        if (!e.orientationValid && (e.state == AttitudeState::Tracking6Axis || e.linearAccel.valid)) {
            sawFalseHealthy = true;
        }
    }
    void run(uint32_t from, uint32_t to, const std::function<void(uint32_t)>& each = nullptr, uint32_t period = 10u)
    {
        for (uint32_t t = from; int32_t(to - t) > 0; t += period) {
            step(h.sample(t));
            if (each) each(t);
        }
    }
    const AttitudeEstimate& e() const { return est.latest(); }
    double err() const { return attsim::quatErrorDeg(h.truth.R, e().q); }
    double tilt() const { return attsim::tiltErrorDeg(h.truth.R, e().q); }
    /// |estimated linear acceleration - true linear acceleration| (m/s^2)
    double linErr() const
    {
        const V3 a = h.truth.accel();
        const Vec3f& b = e().linearAccel.enu_mps2;
        return std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
    }
    double vertErr() const { return std::fabs(e().linearAccel.vertical_mps2 - h.truth.accel().z); }
};

/// Phi(t) of a smooth roll-in at t0 over T to `amp` (integral of attsim::smoothRate).
double ramp(double t, double t0, double T, double amp)
{
    if (t <= t0) return 0.0;
    if (t >= t0 + T) return amp;
    const double u = (t - t0) / T;
    return amp * (u - std::sin(2 * attsim::kPi * u) / (2 * attsim::kPi));
}

/// Coordinated turn: roll in over 3 s at t0, hold, roll out over 3 s; airspeed V.
struct Turn {
    double t0 = 2.0, rollIn = 3.0, hold = 20.0, rollOut = 3.0, bankDeg = 30.0, V = 10.0;
    double t1() const { return t0 + rollIn + hold; }
    double bank(double t) const
    {
        const double b = bankDeg * attsim::kDeg;
        return ramp(t, t0, rollIn, b) - ramp(t, t1(), rollOut, b);
    }
    double bankRate(double t) const
    {
        const double b = bankDeg * attsim::kDeg;
        return attsim::smoothRate(t, t0, rollIn, b) - attsim::smoothRate(t, t1(), rollOut, b);
    }
    void install(attsim::AttitudeTruth& truth) const
    {
        const Turn self = *this;
        truth.omegaBody = [self](double t, const M3& R) {
            const double psiDot = attsim::kG * std::tan(self.bank(t)) / self.V;  // clockwise from above
            const V3 yaw = attsim::mulT(R, V3{0, 0, -psiDot});                  // about ENU up, negative
            return V3{self.bankRate(t) + yaw.x, yaw.y, yaw.z};
        };
        truth.accelEnu = [self](double t, const M3& R) {
            const V3 f = attsim::mul(R, V3{1, 0, 0});
            const double n = std::hypot(f.x, f.y);
            const V3 right{f.y / n, -f.x / n, 0};
            return attsim::scale(right, attsim::kG * std::tan(self.bank(t)));  // centripetal, horizontal
        };
    }
};

struct TurnResult {
    double maxTilt = 0, finalTilt = 0, maxVertErr = 0, maxLinErr = 0, rmsVertErr = 0;
    bool degradedSeen = false, nan = false;
};

TurnResult runTurn(const AttitudeConfig& cfg, const attsim::SensorErrors& err, double bankDeg = 30.0)
{
    Run r(cfg, 9u);
    r.h.err = err;
    Turn turn;
    turn.bankDeg = bankDeg;
    turn.install(r.h.truth);
    TurnResult out;
    attsim::Stats v;
    r.run(1000u, 1000u + 35000u, [&](uint32_t) {
        if (!r.e().orientationValid) return;
        out.maxTilt = std::max(out.maxTilt, r.tilt());
        if (r.e().linearAccel.valid) {
            out.maxVertErr = std::max(out.maxVertErr, r.vertErr());
            out.maxLinErr = std::max(out.maxLinErr, r.linErr());
            v.add(r.vertErr());
        }
        out.degradedSeen = out.degradedSeen || r.e().state == AttitudeState::Degraded;
    });
    out.finalTilt = r.tilt();
    out.rmsVertErr = v.rms();
    out.nan = r.sawNan;
    return out;
}

AttitudeConfig noRejection()
{
    AttitudeConfig c;
    c.accelRejectionDeg = 180.0f;
    c.accelMagnitudeTolerance_mps2 = 1000.0f;
    c.accelRejectionRate_radps = 1000.0f;
    return c;
}

/// Turbulence: band-limited linear acceleration (+-2..3 m/s^2) and small rotations, starting at 3 s.
void installTurbulence(attsim::AttitudeTruth& truth)
{
    truth.accelEnu = [](double t, const M3&) {
        const double k = ramp(t, 3.0, 2.0, 1.0);
        return attsim::scale(V3{1.2 * std::sin(1.3 * t) + 0.8 * std::sin(3.1 * t + 1),
                                1.0 * std::sin(0.9 * t + 2) + 0.7 * std::sin(2.3 * t),
                                1.5 * std::sin(1.7 * t + 0.5) + 0.5 * std::sin(4.1 * t)},
                             k);
    };
    truth.omegaBody = [](double t, const M3&) {
        const double k = ramp(t, 3.0, 2.0, 1.0);
        return attsim::scale(V3{0.15 * std::sin(1.1 * t), 0.12 * std::sin(0.7 * t + 1), 0.2 * std::sin(0.5 * t)}, k);
    };
}

struct TurbulenceResult {
    double maxTilt = 0, rmsTilt = 0, rmsVertErr = 0, acceptedFraction = 0;
};
TurbulenceResult runTurbulence(const AttitudeConfig& cfg)
{
    Run r(cfg, 41u);
    r.h.err = attsim::realisticErrors();
    installTurbulence(r.h.truth);
    attsim::Stats s, v;
    uint32_t acc = 0, n = 0;
    r.run(1000u, 61000u, [&](uint32_t t) {
        if (t < 6000u || !r.e().orientationValid) return;
        s.add(r.tilt());
        if (r.e().linearAccel.valid) v.add(r.vertErr());
        ++n;
        if (r.e().accelCorrection == CorrectionStatus::Accepted) ++acc;
    });
    return TurbulenceResult{s.max, s.rms(), v.rms(), n ? double(acc) / n : 0.0};
}

}  // namespace

// ---------------------------------------------------------------------------
// 1-3 stationary
// ---------------------------------------------------------------------------
TEST(att_sim_01_stationary_level)
{
    Run r;
    r.h.err = attsim::realisticErrors();
    attsim::Stats e, g;
    r.run(1000u, 31000u, [&](uint32_t t) {
        if (t >= 3000u) {  // after levelling + 1.5 s convergence
            e.add(r.err());
            g.add(r.linErr());
        }
    });
    CHECK(e.max < kTargetStationaryDeg);
    CHECK(g.max < kTargetGravityComp);
    CHECK(r.e().state == AttitudeState::Tracking6Axis && r.e().linearAccel.reliable);
    CHECK(!r.sawNan && !r.sawFalseHealthy);
    std::printf("    [metric] S1 stationary level 30 s: orientation max %.3f / rms %.3f deg; |a_lin| max %.4f / rms %.4f m/s^2\n",
                e.max, e.rms(), g.max, g.rms());
}

TEST(att_sim_02_03_stationary_known_roll_and_pitch)
{
    const double cases[][2] = {{45, 0}, {-120, 0}, {0, -30}, {0, 60}, {20, 75}};
    for (const auto& c : cases) {
        Run r;
        r.h.err = attsim::realisticErrors();
        r.h.truth.R = attsim::fromNedEuler(c[0] * attsim::kDeg, c[1] * attsim::kDeg, 0);
        attsim::Stats t, g;
        r.run(1000u, 11000u, [&](uint32_t ms) {
            if (ms >= 3000u) {
                t.add(r.err());
                g.add(r.linErr());
            }
        });
        CHECK(t.max < kTargetStationaryDeg);
        CHECK(g.max < kTargetGravityComp);
        std::printf("    [metric] S2/3 stationary roll %+.0f pitch %+.0f: orientation max %.3f deg, |a_lin| max %.4f m/s^2\n",
                    c[0], c[1], t.max, g.max);
    }
}

// ---------------------------------------------------------------------------
// 4-7 controlled rotations
// ---------------------------------------------------------------------------
namespace {
double rotationScenario(const V3& axisAngleRad, double seconds, const char* name, double* maxDuring = nullptr)
{
    Run r;
    r.h.err = attsim::realisticErrors();
    r.h.truth.omegaBody = [axisAngleRad, seconds](double t, const M3&) {
        const double k = attsim::smoothRate(t, 2.0, seconds, 1.0);
        return attsim::scale(axisAngleRad, k);
    };
    attsim::Stats during;
    r.run(1000u, 1000u + 2000u + uint32_t(seconds * 1000) + 1000u, [&](uint32_t) {
        if (r.e().orientationValid) during.add(r.err());
    });
    std::printf("    [metric] %s: final error %.3f deg (max during manoeuvre %.3f deg), state %d\n", name, r.err(),
                during.max, int(r.e().state));
    if (maxDuring) *maxDuring = during.max;
    return r.err();
}
}  // namespace

TEST(att_sim_04_controlled_90_roll)
{
    CHECK(rotationScenario(V3{attsim::kPi / 2, 0, 0}, 1.0, "S4 90-deg roll in 1 s") < kTargetRotationDeg);
}

TEST(att_sim_05_controlled_90_pitch)
{
    CHECK(rotationScenario(V3{0, attsim::kPi / 2, 0}, 1.0, "S5 90-deg pitch in 1 s (ends at gimbal lock)") <
          kTargetRotationDeg);
}

TEST(att_sim_06_controlled_yaw_6axis)
{
    CHECK(rotationScenario(V3{0, 0, attsim::kPi / 2}, 2.0, "S6 90-deg yaw in 2 s (6-axis)") < kTargetRotationDeg);
}

TEST(att_sim_07_multi_axis_rotation)
{
    CHECK(rotationScenario(V3{60 * attsim::kDeg, -40 * attsim::kDeg, 120 * attsim::kDeg}, 3.0,
                           "S7 multi-axis rotation vector (60,-40,120) deg in 3 s") < kTargetRotationDeg);
}

// ---------------------------------------------------------------------------
// 8 gyro bias and noise
// ---------------------------------------------------------------------------
TEST(att_sim_08_uncompensated_gyro_bias_tilt_bounded_yaw_drifts)
{
    Run r;
    r.h.err = attsim::realisticErrors();
    r.h.err.gyroBias = V3{0.01, -0.01, 0.01};  // 0.57 deg/s residual per axis (e.g. thermal drift)
    attsim::Stats tilt;
    r.run(1000u, 121000u, [&](uint32_t t) {
        if (t >= 3000u) tilt.add(r.tilt());
    });
    const double yawDrift = attsim::headingErrorDeg(r.h.truth.R, r.e().q);
    CHECK(tilt.max < kTargetStationaryDeg);
    CHECK(r.e().heading == HeadingReference::Unreferenced);
    CHECK(std::fabs(yawDrift) > 30.0);  // unobservable in 6-axis: drifts ~0.57 deg/s
    std::printf("    [metric] S8 bias 0.01 rad/s/axis, 120 s: tilt max %.3f deg (bounded by gravity), yaw drift %.1f deg (unobservable)\n",
                tilt.max, yawDrift);
}

// ---------------------------------------------------------------------------
// 9-10 dropout and irregular dt
// ---------------------------------------------------------------------------
TEST(att_sim_09_sensor_dropout)
{
    Run r;
    r.h.err = attsim::realisticErrors();
    const V3 rv{60 * attsim::kDeg, -40 * attsim::kDeg, 120 * attsim::kDeg};
    r.h.truth.omegaBody = [rv](double t, const M3&) { return attsim::scale(rv, attsim::smoothRate(t, 2.0, 3.0, 1.0)); };
    sim::Rng drop(77);
    uint32_t dropped = 0, invalid = 0;
    for (uint32_t t = 1000u; t < 7000u; t += 10u) {
        AttitudeInput in = r.h.sample(t);
        if (t > 1700u && drop.uniform() < 0.2) {
            ++dropped;  // whole sample missing (20 %)
            continue;
        }
        if (t > 1700u && drop.uniform() < 0.05) {
            in.accelValid = false;  // 5 % flagged invalid accelerometer
            ++invalid;
        }
        r.step(in);
    }
    CHECK(r.err() < kTargetRotationDeg);
    CHECK(!r.sawNan);
    std::printf("    [metric] S9 20%% dropped + 5%% invalid-accel samples (%u, %u) during multi-axis rotation: final %.3f deg\n",
                dropped, invalid, r.err());
}

TEST(att_sim_10_irregular_dt)
{
    Run r;
    r.h.err = attsim::realisticErrors();
    const V3 rv{0, 70 * attsim::kDeg, -90 * attsim::kDeg};
    r.h.truth.omegaBody = [rv](double t, const M3&) { return attsim::scale(rv, attsim::smoothRate(t, 2.0, 2.0, 1.0)); };
    sim::Rng j(78);
    attsim::Stats e;
    uint32_t t = 1000u;
    while (t < 7000u) {
        r.step(r.h.sample(t));
        if (r.e().orientationValid) e.add(r.err());
        t += 5u + uint32_t(j.next() % 13u);  // 5..17 ms spacing
    }
    CHECK(r.err() < kTargetRotationDeg);
    std::printf("    [metric] S10 irregular dt 5..17 ms: final %.3f deg, max %.3f deg\n", r.err(), e.max);
}

// ---------------------------------------------------------------------------
// 11-12 accelerometer disturbances
// ---------------------------------------------------------------------------
TEST(att_sim_11_sudden_accelerometer_disturbance)
{
    Run r;
    r.h.err = attsim::realisticErrors();
    // 0.4 s real horizontal push of 3 m/s^2 (physical: in a_ENU) + a single 30 m/s^2 spike sample
    r.h.truth.accelEnu = [](double t, const M3&) { return V3{(t > 3.0 && t < 3.4) ? 3.0 : 0.0, 0, 0}; };
    r.h.accelDisturbanceBody = [](double t) { return V3{(t > 5.0 && t < 5.0099) ? 30.0 : 0.0, 0, 0}; };
    attsim::Stats e;
    uint32_t rejected = 0;
    r.run(1000u, 9000u, [&](uint32_t t) {
        if (t >= 3000u) e.add(r.tilt());
        if (r.e().accelCorrection == CorrectionStatus::RejectedDirection ||
            r.e().accelCorrection == CorrectionStatus::RejectedMagnitude)
            ++rejected;
    });
    CHECK(rejected >= 30u);  // the push (17 deg apparent tilt) and the spike were gated
    CHECK(e.max < kTargetStationaryDeg);
    std::printf("    [metric] S11 3 m/s^2 push 0.4 s + 3 g spike: %u corrections rejected, tilt max %.3f deg\n", rejected,
                e.max);
}

TEST(att_sim_12_sustained_acceleration_coordinated_turn)
{
    const TurnResult on = runTurn(AttitudeConfig{}, attsim::realisticErrors());
    const TurnResult off = runTurn(noRejection(), attsim::realisticErrors());
    CHECK(!on.nan && !off.nan);
    CHECK(on.degradedSeen);               // long rejection is reported, not hidden
    CHECK(on.maxTilt < off.maxTilt);      // gating helps
    CHECK(on.finalTilt < kTargetRotationDeg);
    std::printf("    [metric] S12 coordinated turn 30 deg bank 20 s @10 m/s, gate 10 deg: tilt max %.2f deg, final %.2f deg, "
                "vertical-accel err max %.3f / rms %.3f m/s^2, |a_lin| err max %.3f\n",
                on.maxTilt, on.finalTilt, on.maxVertErr, on.rmsVertErr, on.maxLinErr);
    std::printf("    [metric] S12 same turn WITHOUT rejection: tilt max %.2f deg, final %.2f deg, vertical-accel err max %.3f m/s^2, "
                "|a_lin| err max %.3f\n",
                off.maxTilt, off.finalTilt, off.maxVertErr, off.maxLinErr);
}

TEST(att_sim_12b_turn_truth_is_physically_coordinated)
{
    // Independent check of the simulator: in a coordinated turn the lateral specific force is ~0
    // and |f| = g / cos(bank).
    attsim::AttitudeTruth truth;
    Turn turn;
    turn.install(truth);
    double maxLat = 0, maxMagErr = 0;
    for (int k = 1; k < 3000; ++k) {
        truth.advanceTo(k * 0.01);
        const V3 f = truth.specificForceBody();
        maxLat = std::max(maxLat, std::fabs(f.y));
        maxMagErr = std::max(maxMagErr, std::fabs(attsim::norm(f) - attsim::kG / std::cos(turn.bank(k * 0.01))));
    }
    CHECK(maxLat < 0.02);
    CHECK(maxMagErr < 0.02);
}

// ---------------------------------------------------------------------------
// 13-14 magnetic disturbance, recovery after a disturbance
// ---------------------------------------------------------------------------
TEST(att_sim_13_magnetic_disturbance_during_9axis_turn)
{
    AttitudeConfig cfg;
    cfg.magPolicy = MagPolicy::CalibratedOnly;
    Run r(cfg);
    r.h.err = attsim::realisticErrors();
    r.h.magPresent = true;
    r.h.truth.omegaBody = [](double t, const M3&) { return V3{0, 0, attsim::smoothRate(t, 4.0, 4.0, attsim::kPi / 2)}; };
    const V3 m = r.h.truth.magEnu;
    r.h.magDisturbanceEnu = [m](double t) {  // 4 s disturbance (field rotated 45 deg): 2 s in the turn, 2 s after
        if (t < 6.0 || t > 10.0) return V3{0, 0, 0};
        const double c = std::cos(45 * attsim::kDeg), s = std::sin(45 * attsim::kDeg);
        return V3{c * m.x - s * m.y - m.x, s * m.x + c * m.y - m.y, 0};
    };
    attsim::Stats hd;
    uint32_t rej = 0, noGravity = 0, acceptedWhileDisturbed = 0;
    r.run(1000u, 14000u, [&](uint32_t t) {
        if (t >= 3000u) hd.add(attsim::headingErrorDeg(r.h.truth.R, r.e().q));
        const CorrectionStatus mc = r.e().magCorrection;
        if (mc == CorrectionStatus::RejectedDirection) ++rej;
        if (mc == CorrectionStatus::NoGravityReference) ++noGravity;
        const double ts = r.h.tOf(t);
        if (ts > 6.03 && ts < 10.0 && (mc == CorrectionStatus::Accepted || mc == CorrectionStatus::AcceptedRecovery))
            ++acceptedWhileDisturbed;
    });
    CHECK(rej > 50u);                     // after the turn: rejected by the heading gate
    CHECK(noGravity > 50u);               // during the turn: no magnetometer step without gravity correction
    CHECK_EQ(acceptedWhileDisturbed, 0u);
    CHECK(hd.max < kTargetRotationDeg);
    CHECK(r.e().heading == HeadingReference::MagneticReferenced);
    std::printf("    [metric] S13 9-axis 90-deg yaw with 4 s 45-deg magnetic disturbance: heading err max %.3f deg, %u heading-gate "
                "rejections, %u steps skipped (turning)\n",
                hd.max, rej, noGravity);
}

TEST(att_sim_14_recovery_after_gap_induced_error)
{
    // Gyro samples lost for 300 ms in the middle of a 90-deg roll: the hole is not integrated,
    // so the estimate is left ~30+ deg off. Gravity gating then rejects; recovery re-converges
    // once the device is quasi-static.
    Run r;
    r.h.err = attsim::realisticErrors();
    r.h.truth.omegaBody = [](double t, const M3&) { return V3{attsim::smoothRate(t, 2.0, 1.0, attsim::kPi / 2), 0, 0}; };
    double worst = 0;
    uint32_t recoveredAt = 0;
    bool degraded = false;
    for (uint32_t t = 1000u; t < 12000u; t += 10u) {
        AttitudeInput in = r.h.sample(t);
        if (t >= 3300u && t < 3600u) in.gyroValid = false;
        r.step(in);
        if (t >= 3600u) {
            worst = std::max(worst, r.err());
            degraded = degraded || r.e().state == AttitudeState::Degraded;
            if (recoveredAt == 0u && r.err() < kTargetRotationDeg && r.e().state == AttitudeState::Tracking6Axis)
                recoveredAt = t;
        }
    }
    CHECK(worst > 20.0);          // the gap really caused a large error
    CHECK(degraded);
    CHECK(r.e().recoveries >= 1u);
    CHECK(recoveredAt != 0u);
    CHECK(r.err() < kTargetRotationDeg);
    std::printf("    [metric] S14 300 ms gyro gap mid-roll: error after gap %.1f deg, back < 2 deg and Tracking at +%.2f s, final %.3f deg\n",
                worst, (recoveredAt - 3600u) / 1000.0, r.err());
}

// ---------------------------------------------------------------------------
// 15 free fall, 16 extended 6-axis, 17 duplicate / out-of-order
// ---------------------------------------------------------------------------
TEST(att_sim_15_free_fall_near_zero_specific_force)
{
    Run r;
    r.h.err = attsim::realisticErrors();
    r.h.truth.accelEnu = [](double t, const M3&) { return V3{0, 0, (t > 3.0 && t < 4.5) ? -attsim::kG : 0.0}; };
    double before = 0, worstVert = 0, maxTilt = 0;
    uint32_t magRejects = 0;
    r.run(1000u, 8000u, [&](uint32_t t) {
        if (t == 3990u) before = r.tilt();
        if (t > 4050u && t < 5450u) {
            worstVert = std::max(worstVert, r.vertErr());
            if (r.e().accelCorrection == CorrectionStatus::RejectedMagnitude) ++magRejects;
            CHECK(r.e().linearAccel.valid);
        }
        if (t >= 3000u) maxTilt = std::max(maxTilt, r.tilt());
    });
    CHECK(magRejects > 100u);  // |f| ~ 0 never used as a gravity direction
    CHECK(worstVert < kTargetGravityComp);
    CHECK(maxTilt < kTargetStationaryDeg);
    CHECK(!r.sawNan);
    std::printf("    [metric] S15 1.5 s free fall: vertical accel error max %.4f m/s^2 (reads -g), tilt max %.3f deg (before %.3f)\n",
                worstVert, maxTilt, before);
}

TEST(att_sim_16_extended_6axis_operation)
{
    // 10 minutes: repeated gentle manoeuvres separated by quiet periods; no magnetometer.
    Run r;
    r.h.err = attsim::realisticErrors();
    r.h.truth.omegaBody = [](double t, const M3&) {
        const double c = std::fmod(t, 30.0);
        return V3{attsim::smoothRate(c, 5.0, 4.0, 0.6) - attsim::smoothRate(c, 12.0, 4.0, 0.6),
                  attsim::smoothRate(c, 18.0, 3.0, 0.3) - attsim::smoothRate(c, 23.0, 3.0, 0.3),
                  attsim::smoothRate(c, 5.0, 10.0, 1.0)};
    };
    attsim::Stats quietTilt, anyTilt;
    bool alwaysUnreferenced = true;
    r.run(1000u, 601000u, [&](uint32_t t) {
        if (t < 3000u) return;
        anyTilt.add(r.tilt());
        if (std::fmod(r.h.tOf(t), 30.0) > 27.0) quietTilt.add(r.tilt());
        alwaysUnreferenced = alwaysUnreferenced && r.e().heading == HeadingReference::Unreferenced;
    });
    CHECK(alwaysUnreferenced);
    CHECK(quietTilt.max < kTargetStationaryDeg);
    CHECK(!r.sawNan && !r.sawFalseHealthy);
    std::printf("    [metric] S16 10 min 6-axis: tilt max %.3f deg (quiet periods %.3f), yaw drift %.2f deg (unreferenced)\n",
                anyTilt.max, quietTilt.max, attsim::headingErrorDeg(r.h.truth.R, r.e().q));
}

TEST(att_sim_17_duplicate_and_out_of_order_stream)
{
    auto make = [] {
        Run r;
        r.h.err = attsim::realisticErrors();
        r.h.truth.omegaBody = [](double t, const M3&) {
            return V3{attsim::smoothRate(t, 2.0, 2.0, 1.0), 0, attsim::smoothRate(t, 2.0, 2.0, -1.2)};
        };
        return r;
    };
    // Same sample sequence for both (identical seeds): clean vs. corrupted delivery.
    Run clean = make();
    Run messy = make();
    sim::Rng mix(91);
    AttitudeInput prev{};
    bool havePrev = false;
    uint32_t extra = 0;
    for (uint32_t t = 1000u; t < 6000u; t += 10u) {
        const AttitudeInput a = clean.h.sample(t);
        const AttitudeInput b = messy.h.sample(t);
        clean.step(a);
        messy.step(b);
        messy.step(b);  // every sample delivered twice
        ++extra;
        if (havePrev && mix.uniform() < 0.1) {
            AttitudeInput late = prev;  // an older sample arriving late
            late.nowMs = b.nowMs;
            messy.step(late);
            ++extra;
        }
        prev = b;
        havePrev = true;
    }
    CHECK(messy.e().q.w == clean.e().q.w && messy.e().q.x == clean.e().q.x && messy.e().q.y == clean.e().q.y &&
          messy.e().q.z == clean.e().q.z);  // bit-identical: nothing integrated twice
    CHECK(messy.e().notNewSamples + messy.e().outOfOrderSamples >= 400u);
    std::printf("    [metric] S17 %u duplicate/late deliveries: final quaternion bit-identical to clean run (error %.3f deg)\n",
                extra, clean.err());
}

// ---------------------------------------------------------------------------
// Sensitivity sweeps
// ---------------------------------------------------------------------------
TEST(att_sweep_beta_noise_drift_and_acceleration)
{
    const float betas[] = {0.01f, 0.033f, 0.1f, 0.3f, 0.5f};
    double chatter[5], noiseRms[5], biasTilt[5], turnTilt[5], turnTiltOff[5];
    std::printf("    [metric] beta sweep (rad/s): limit-cycle max (bias 0.002, no noise) | noisy-stationary tilt rms | bias 0.05 rad/s tilt max "
                "| turn30 max tilt (all gates) | turn30 max tilt (no gates)\n");
    for (int i = 0; i < 5; ++i) {
        AttitudeConfig cfg;
        cfg.beta_radps = betas[i];
        {  // (0) noise-free, small 0.002 rad/s bias: the normalised-gradient step limit-cycles by ~2 beta dt
            Run r(cfg, 30u);
            r.h.err.gyroBias = V3{0.002, 0, 0};
            attsim::Stats s;
            r.run(1000u, 11000u, [&](uint32_t t) {
                if (t >= 3000u) s.add(r.tilt());
            });
            chatter[i] = s.max;
        }
        {  // (a) noisy stationary
            Run r(cfg, 31u);
            r.h.err.gyroNoise = 0.01;
            r.h.err.accelNoise = 0.1;
            attsim::Stats s;
            r.run(1000u, 31000u, [&](uint32_t t) {
                if (t >= 6000u) s.add(r.tilt());
            });
            noiseRms[i] = s.rms();
        }
        {  // (b) large uncompensated bias (2.9 deg/s): needs 2*beta > bias to be cancelled
            Run r(cfg, 32u);
            r.h.err.gyroBias = V3{0.05, 0, 0};
            attsim::Stats s;
            r.run(1000u, 61000u, [&](uint32_t t) {
                if (t >= 3000u) s.add(r.tilt());
            });
            biasTilt[i] = s.max;
        }
        turnTilt[i] = runTurn(cfg, attsim::realisticErrors()).maxTilt;
        AttitudeConfig off = noRejection();
        off.beta_radps = betas[i];
        turnTiltOff[i] = runTurn(off, attsim::realisticErrors()).maxTilt;
        std::printf("    [metric]   beta %.3f: %.3f | %.3f | %.2f | %.2f | %.2f deg\n", double(betas[i]), chatter[i],
                    noiseRms[i], biasTilt[i], turnTilt[i], turnTiltOff[i]);
        CHECK(turnTilt[i] <= turnTiltOff[i] + 1e-6);
    }
    CHECK(chatter[4] > chatter[1]);    // limit cycle grows with beta (~2 beta dt) once 2 beta > bias
    CHECK(noiseRms[4] > noiseRms[0]);  // larger gain lets more accelerometer noise through
    CHECK(biasTilt[0] > 2.0);          // beta 0.01: 2 beta = 0.02 < 0.05 rad/s bias -> cannot hold the tilt
    CHECK(biasTilt[2] < biasTilt[0]);  // beta 0.1 cancels it
}

TEST(att_sweep_rejection_angle)
{
    const float gates[] = {5.0f, 10.0f, 20.0f, 180.0f};
    std::printf("    [metric] angle-gate sweep (beta 0.1, turn gate OFF): turn30 max tilt | turn15 max tilt | turbulence tilt max/rms\n");
    for (float g : gates) {
        AttitudeConfig cfg;
        cfg.accelRejectionDeg = g;
        cfg.accelRejectionRate_radps = 1000.0f;
        if (g >= 180.0f) cfg.accelMagnitudeTolerance_mps2 = 1000.0f;
        const double t30 = runTurn(cfg, attsim::realisticErrors(), 30.0).maxTilt;
        const double t15 = runTurn(cfg, attsim::realisticErrors(), 15.0).maxTilt;
        const TurbulenceResult tb = runTurbulence(cfg);
        std::printf("    [metric]   gate %5.1f deg: %.2f | %.2f | %.2f / %.2f deg\n", double(g), t30, t15, tb.maxTilt,
                    tb.rmsTilt);
        CHECK(tb.maxTilt > 0.0);  // the scenario really ran (levelled before the turbulence)
    }
}

TEST(att_sweep_turn_gate_rate)
{
    const float rates[] = {1000.0f, 0.3f, 0.15f, 0.10f, 0.05f};
    std::printf("    [metric] turn-gate sweep (beta 0.1, angle gate 10): turn30 max tilt | turn15 max tilt | turn5 max tilt | "
                "turbulence tilt max/rms, vertical-accel rms, accepted fraction\n");
    double t15[5];
    for (int i = 0; i < 5; ++i) {
        AttitudeConfig cfg;
        cfg.accelRejectionRate_radps = rates[i];
        const double t30 = runTurn(cfg, attsim::realisticErrors(), 30.0).maxTilt;
        t15[i] = runTurn(cfg, attsim::realisticErrors(), 15.0).maxTilt;
        const double t5 = runTurn(cfg, attsim::realisticErrors(), 5.0).maxTilt;
        const TurbulenceResult tb = runTurbulence(cfg);
        std::printf("    [metric]   rate gate %7.2f rad/s: %.2f | %.2f | %.2f | %.2f / %.2f deg, %.3f m/s^2, %.2f\n",
                    double(rates[i]), t30, t15[i], t5, tb.maxTilt, tb.rmsTilt, tb.rmsVertErr, tb.acceptedFraction);
    }
    CHECK(t15[3] < 0.5 * t15[0]);  // the default gate (0.10) removes most of the coordinated-turn error
}
