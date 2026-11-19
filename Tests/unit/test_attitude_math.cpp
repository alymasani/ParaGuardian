/**
 * Attitude milestone - quaternion conventions, Madgwick gradient derivations,
 * gravity compensation. Every expected value comes from the independent
 * double-precision code in sim/AttitudeSimulator.hpp or from hand-derived
 * physics, never from the production function under test.
 */
#include <cmath>

#include "TestFramework.hpp"
#include "sim/AttitudeSimulator.hpp"

#include "Algorithms/Attitude/GravityCompensator.hpp"
#include "Algorithms/Attitude/MadgwickFilter.hpp"
#include "Algorithms/Attitude/Quaternion.hpp"
#include "Processing/AxisAlignment.hpp"

using namespace pg;
using attsim::M3;
using attsim::V3;

namespace {

constexpr float kG = units::kStandardGravity_mps2;

Quaternion randomUnit(sim::Rng& rng)
{
    Quaternion q{float(rng.gaussian()), float(rng.gaussian()), float(rng.gaussian()), float(rng.gaussian())};
    Quaternion n{};
    quat::normalize(q, n);
    return n;
}

bool nearV(const Vec3f& a, const V3& b, double tol)
{
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol;
}

/// Independent objective: E(q) = 1/2 |R(q/|q|)^T d - s|^2 with the simulator's matrix formula.
double objective(const double* qd, const V3& d, const V3& s)
{
    const double n = std::sqrt(qd[0] * qd[0] + qd[1] * qd[1] + qd[2] * qd[2] + qd[3] * qd[3]);
    const double a = qd[0] / n, b = qd[1] / n, c = qd[2] / n, e4 = qd[3] / n;
    const M3 r{{{a * a + b * b - c * c - e4 * e4, 2 * (b * c - a * e4), 2 * (b * e4 + a * c)},
                {2 * (b * c + a * e4), a * a - b * b + c * c - e4 * e4, 2 * (c * e4 - a * b)},
                {2 * (b * e4 - a * c), 2 * (c * e4 + a * b), a * a - b * b - c * c + e4 * e4}}};
    const V3 p = attsim::mulT(r, d);
    const V3 e{p.x - s.x, p.y - s.y, p.z - s.z};
    return 0.5 * attsim::dot(e, e);
}

/// Compares the TANGENTIAL part of an analytic gradient with central finite differences
/// of the independent objective (radial parts differ by construction: the Madgwick
/// objective uses unit-norm identities, E above is scale-invariant).
double tangentialGradientError(const Quaternion& q, const MadgwickFilter::Gradient& g, const V3& d, const V3& s)
{
    const double qv[4] = {q.w, q.x, q.y, q.z};
    double fd[4];
    const double h = 1e-4;
    for (int i = 0; i < 4; ++i) {
        double p[4] = {qv[0], qv[1], qv[2], qv[3]};
        double m[4] = {qv[0], qv[1], qv[2], qv[3]};
        p[i] += h;
        m[i] -= h;
        fd[i] = (objective(p, d, s) - objective(m, d, s)) / (2 * h);
    }
    // project both on the tangent space of the unit sphere at q
    double ga[4] = {g.g[0], g.g[1], g.g[2], g.g[3]};
    double da = 0, df = 0;
    for (int i = 0; i < 4; ++i) {
        da += ga[i] * qv[i];
        df += fd[i] * qv[i];
    }
    double err = 0, scale = 0;
    for (int i = 0; i < 4; ++i) {
        // Madgwick's f is 2x the rotation-matrix residual only in the unit-norm form;
        // both objectives are 1/2|f|^2 with the same f on the sphere, so tangents must agree.
        const double a = ga[i] - da * qv[i];
        const double b = fd[i] - df * qv[i];
        err = std::max(err, std::fabs(a - b));
        scale = std::max(scale, std::fabs(b));
    }
    return err / std::max(scale, 1e-3);
}

}  // namespace

// ---------------------------------------------------------------------------
// Quaternion conventions
// ---------------------------------------------------------------------------
TEST(att_quat_rotate_and_matrix_match_independent_formula)
{
    sim::Rng rng(11);
    for (int k = 0; k < 200; ++k) {
        const Quaternion q = randomUnit(rng);
        const Vec3f v{float(rng.gaussian()), float(rng.gaussian()), float(rng.gaussian())};
        const V3 ref = attsim::mul(attsim::matrixFromQuat(q), V3{v.x, v.y, v.z});
        CHECK(nearV(quat::rotate(q, v), ref, 2e-6 * (1 + attsim::norm(ref))));
        CHECK(nearV(mat::mul(quat::toMatrix(q), v), ref, 2e-6 * (1 + attsim::norm(ref))));
        const V3 inv = attsim::mulT(attsim::matrixFromQuat(q), V3{v.x, v.y, v.z});
        CHECK(nearV(quat::rotateInverse(q, v), inv, 2e-6 * (1 + attsim::norm(inv))));
    }
}

TEST(att_quat_hamilton_product_composes_like_matrices)
{
    // q_A_from_C = q_A_from_B (x) q_B_from_C  <=>  R_AC = R_AB * R_BC
    sim::Rng rng(12);
    for (int k = 0; k < 200; ++k) {
        const Quaternion a = randomUnit(rng), b = randomUnit(rng);
        const M3 ref = attsim::mul(attsim::matrixFromQuat(a), attsim::matrixFromQuat(b));
        const M3 got = attsim::matrixFromQuat(quat::multiply(a, b));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) CHECK_NEAR(got.m[i][j], ref.m[i][j], 2e-6);
    }
}

TEST(att_quat_matrix_is_proper_rotation)
{
    sim::Rng rng(13);
    for (int k = 0; k < 100; ++k) {
        const Mat3f r = quat::toMatrix(randomUnit(rng));
        CHECK(AxisAlignment::isProperRotation(r));
        CHECK_NEAR(mat::determinant(r), 1.0, 1e-5);
    }
}

TEST(att_quat_axis_angle_right_hand_rule)
{
    // +90 deg about ENU up (z): east (x) -> north (y)
    const Quaternion q = quat::fromAxisAngle(Vec3f{0, 0, 1}, quat::kPi / 2);
    CHECK(nearV(quat::rotate(q, Vec3f{1, 0, 0}), V3{0, 1, 0}, 1e-6));
    CHECK(nearV(quat::rotate(q, Vec3f{0, 1, 0}), V3{-1, 0, 0}, 1e-6));
}

TEST(att_quat_enu_from_ned_matches_axis_alignment_matrix)
{
    const Mat3f r = quat::toMatrix(quat::kEnuFromNed);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) CHECK_NEAR(r.m[i][j], kEnuFromNed.m[i][j], 1e-6);
    // self-inverse: applying twice is the identity rotation
    const Quaternion qq = quat::multiply(quat::kEnuFromNed, quat::kEnuFromNed);
    CHECK_NEAR(std::fabs(qq.w), 1.0, 1e-6);
    // NED north (1,0,0) is ENU (0,1,0); NED down is ENU -z
    CHECK(nearV(quat::rotate(quat::kEnuFromNed, Vec3f{1, 0, 0}), V3{0, 1, 0}, 1e-6));
    CHECK(nearV(quat::rotate(quat::kEnuFromNed, Vec3f{0, 0, 1}), V3{0, 0, -1}, 1e-6));
}

TEST(att_quat_euler_matches_independent_matrix_and_physical_meaning)
{
    sim::Rng rng(14);
    for (int k = 0; k < 200; ++k) {
        const double r = (rng.uniform() - 0.5) * 2 * 170 * attsim::kDeg;
        const double p = (rng.uniform() - 0.5) * 2 * 85 * attsim::kDeg;
        const double y = (rng.uniform() - 0.5) * 2 * 179 * attsim::kDeg;
        const Quaternion q = quat::fromEulerNedZyx(float(r), float(p), float(y));
        const M3 ref = attsim::fromNedEuler(r, p, y);
        const M3 got = attsim::matrixFromQuat(q);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) CHECK_NEAR(got.m[i][j], ref.m[i][j], 3e-6);
        const EulerAngles e = quat::toEulerNedZyx(q);  // round trip
        CHECK_NEAR(e.roll, r, 1e-4);
        CHECK_NEAR(e.pitch, p, 1e-4);
        CHECK_NEAR(e.yaw, y, 1e-4);
    }
    const float d90 = quat::kPi / 2;
    // yaw +90: body forward (x) points EAST (ENU +x)
    CHECK(nearV(quat::rotate(quat::fromEulerNedZyx(0, 0, d90), Vec3f{1, 0, 0}), V3{1, 0, 0}, 1e-6));
    // yaw 0, level: forward = NORTH, right = EAST, down = -UP
    const Quaternion lvl = quat::fromEulerNedZyx(0, 0, 0);
    CHECK(nearV(quat::rotate(lvl, Vec3f{1, 0, 0}), V3{0, 1, 0}, 1e-6));
    CHECK(nearV(quat::rotate(lvl, Vec3f{0, 1, 0}), V3{1, 0, 0}, 1e-6));
    CHECK(nearV(quat::rotate(lvl, Vec3f{0, 0, 1}), V3{0, 0, -1}, 1e-6));
    // pitch +30 (nose up): forward has a positive UP component sin 30
    CHECK_NEAR(quat::rotate(quat::fromEulerNedZyx(0, 0.5235988f, 0), Vec3f{1, 0, 0}).z, 0.5, 1e-6);
    // roll +30 (right side down): body right (y) has a NEGATIVE up component
    CHECK_NEAR(quat::rotate(quat::fromEulerNedZyx(0.5235988f, 0, 0), Vec3f{0, 1, 0}).z, -0.5, 1e-6);
}

TEST(att_quat_rest_specific_force_maps_to_plus_g_up_in_enu)
{
    // Independent physics: a level FRD accelerometer at rest reads (0,0,-g); any yaw.
    for (float yaw : {0.0f, 1.0f, -2.5f}) {
        const Vec3f fEnu = quat::rotate(quat::fromEulerNedZyx(0, 0, yaw), Vec3f{0, 0, -kG});
        CHECK(nearV(fEnu, V3{0, 0, kG}, 1e-5));
    }
    // Rolled 90 deg right: gravity reaction along body -y... f_body = (0, -g, 0)
    const Vec3f f2 = quat::rotate(quat::fromEulerNedZyx(quat::kPi / 2, 0, 0), Vec3f{0, -kG, 0});
    CHECK(nearV(f2, V3{0, 0, kG}, 1e-5));
}

TEST(att_quat_euler_gimbal_lock_stays_finite)
{
    const EulerAngles e = quat::toEulerNedZyx(quat::fromEulerNedZyx(0.3f, quat::kPi / 2, 0.2f));
    CHECK(std::isfinite(e.roll) && std::isfinite(e.pitch) && std::isfinite(e.yaw));
    CHECK_NEAR(e.pitch, quat::kPi / 2, 2e-3);
}

TEST(att_quat_normalize_rejects_degenerate_and_scales_valid)
{
    Quaternion out{9, 9, 9, 9};
    CHECK(!quat::normalize(Quaternion{0, 0, 0, 0}, out));
    CHECK(!quat::normalize(Quaternion{std::nanf(""), 0, 0, 1}, out));
    CHECK(!quat::normalize(Quaternion{INFINITY, 0, 0, 0}, out));
    CHECK(!quat::normalize(Quaternion{1e-9f, 0, 0, 0}, out));
    CHECK_EQ(out.w, 9.0f);  // untouched on failure
    CHECK(quat::normalize(Quaternion{2, 0, 0, 0}, out));
    CHECK_NEAR(out.w, 1.0, 1e-7);
    CHECK(quat::normalize(Quaternion{1e6f, 1e6f, 0, 0}, out));
    CHECK_NEAR(quat::norm(out), 1.0, 1e-6);
}

TEST(att_quat_angle_between_matches_independent_metric)
{
    sim::Rng rng(15);
    for (int k = 0; k < 100; ++k) {
        const Quaternion a = randomUnit(rng), b = randomUnit(rng);
        const double ref = attsim::matrixErrorDeg(attsim::matrixFromQuat(a), b);
        CHECK_NEAR(quat::angleBetween(a, b) * quat::kDegPerRad, ref, 0.05);
    }
    // q and -q are the same rotation
    const Quaternion q = randomUnit(rng);
    CHECK_NEAR(quat::angleBetween(q, Quaternion{-q.w, -q.x, -q.y, -q.z}), 0.0, 1e-3);
}

// ---------------------------------------------------------------------------
// Madgwick gradient derivations
// ---------------------------------------------------------------------------
TEST(att_madgwick_accel_gradient_matches_finite_difference_and_vanishes_at_truth)
{
    sim::Rng rng(21);
    const V3 up{0, 0, 1};
    for (int k = 0; k < 100; ++k) {
        const Quaternion q = randomUnit(rng);
        // measurement from a DIFFERENT orientation so the residual is non-zero
        const Quaternion q2 = randomUnit(rng);
        const V3 s = attsim::mulT(attsim::matrixFromQuat(q2), up);
        const MadgwickFilter::Gradient g = MadgwickFilter::accelGradient(q, attsim::toF(s));
        CHECK(tangentialGradientError(q, g, up, s) < 2e-3);
        // at the true orientation the objective and its gradient are zero
        const V3 sTrue = attsim::mulT(attsim::matrixFromQuat(q), up);
        const MadgwickFilter::Gradient g0 = MadgwickFilter::accelGradient(q, attsim::toF(sTrue));
        for (float gi : g0.g) CHECK(std::fabs(gi) < 1e-5f);
    }
}

TEST(att_madgwick_mag_gradient_matches_finite_difference_and_vanishes_at_truth)
{
    sim::Rng rng(22);
    for (int k = 0; k < 100; ++k) {
        const Quaternion q = randomUnit(rng);
        const double incl = (rng.uniform() - 0.5) * 2 * 80 * attsim::kDeg;
        const V3 b{0, std::cos(incl), std::sin(incl)};  // ENU reference, north + dip
        const Quaternion q2 = randomUnit(rng);
        const V3 m = attsim::mulT(attsim::matrixFromQuat(q2), b);
        const MadgwickFilter::Gradient g = MadgwickFilter::magGradient(q, attsim::toF(m), float(b.y), float(b.z));
        CHECK(tangentialGradientError(q, g, b, m) < 2e-3);
        const V3 mTrue = attsim::mulT(attsim::matrixFromQuat(q), b);
        const MadgwickFilter::Gradient g0 = MadgwickFilter::magGradient(q, attsim::toF(mTrue), float(b.y), float(b.z));
        for (float gi : g0.g) CHECK(std::fabs(gi) < 1e-5f);
        // the reference re-derived from the measurement at the true attitude is b itself
        float bn = 0, bu = 0;
        MadgwickFilter::magReference(q, attsim::toF(mTrue), bn, bu);
        CHECK_NEAR(bn, b.y, 1e-5);
        CHECK_NEAR(bu, b.z, 1e-5);
    }
}

TEST(att_madgwick_correction_step_reduces_tilt_error)
{
    // Truth level; estimate tilted 20 deg about body x. One step (no rotation) must reduce the error by ~2*beta*dt.
    MadgwickFilter f;
    CHECK(f.reset(quat::fromEulerNedZyx(20 * quat::kRadPerDeg, 0, 0)));
    const M3 truth = attsim::fromNedEuler(0, 0, 0);
    const Vec3f a{0, 0, -1};  // level: measured up direction in FRD is -z
    const double e0 = attsim::tiltErrorDeg(truth, f.orientation());
    CHECK(f.update(Vec3f{0, 0, 0}, &a, nullptr, 0.5f, 0.01f));
    const double e1 = attsim::tiltErrorDeg(truth, f.orientation());
    CHECK(e1 < e0);
    CHECK_NEAR(e0 - e1, 2 * 0.5 * 0.01 / attsim::kDeg, 0.05);  // rate 2*beta rad/s of rotation angle
    CHECK_NEAR(quat::norm(f.orientation()), 1.0, 1e-6);
}

TEST(att_madgwick_pure_gyro_integration_matches_truth)
{
    // 90 deg/s about body x for 1 s at 100 Hz, beta = 0: compare with exact Rodrigues truth.
    MadgwickFilter f;
    f.reset(quat::fromEulerNedZyx(0, 0, 0.7f));
    M3 truth = attsim::fromNedEuler(0, 0, 0.7);
    const V3 w{attsim::kPi / 2, 0, 0};
    for (int k = 0; k < 100; ++k) {
        CHECK(f.update(attsim::toF(w), nullptr, nullptr, 0.0f, 0.01f));
        truth = attsim::mul(truth, attsim::expm(attsim::scale(w, 0.01)));
    }
    CHECK(attsim::quatErrorDeg(truth, f.orientation()) < 0.02);
    // body-frame (right) multiplication: after yaw 0.7 the roll still happens about BODY x
    CHECK(attsim::matrixErrorDeg(truth, f.orientation()) < 0.02);
}

TEST(att_madgwick_rejects_bad_step_and_keeps_state)
{
    MadgwickFilter f;
    const Quaternion q0 = f.orientation();
    CHECK(!f.update(Vec3f{std::nanf(""), 0, 0}, nullptr, nullptr, 0.1f, 0.01f));
    CHECK(!f.update(Vec3f{0, 0, 0}, nullptr, nullptr, 0.1f, 0.0f));
    CHECK(!f.update(Vec3f{0, 0, 0}, nullptr, nullptr, 0.1f, -1.0f));
    CHECK(!f.reset(Quaternion{0, 0, 0, 0}));
    CHECK_EQ(f.orientation().w, q0.w);
    CHECK(attsim::finite(f.orientation()));
}

// ---------------------------------------------------------------------------
// Gravity compensation (brief section 11, tests 1-10)
// ---------------------------------------------------------------------------
namespace {
/// Independent expected value: a = R f + (0,0,-g) with the simulator's matrix.
V3 expectedLinear(const Quaternion& q, const Vec3f& f)
{
    const V3 r = attsim::mul(attsim::matrixFromQuat(q), V3{f.x, f.y, f.z});
    return V3{r.x, r.y, r.z - attsim::kG};
}
}  // namespace

TEST(att_gravity_level_rest_is_zero)
{
    GravityCompensator gc;
    const Quaternion q = quat::fromEulerNedZyx(0, 0, 1.2f);
    const LinearAcceleration a = gc.compute(q, true, true, Vec3f{0, 0, -kG}, true, 5u);
    CHECK(a.valid && a.reliable);
    CHECK(nearV(a.enu_mps2, V3{0, 0, 0}, 1e-5));
    CHECK_EQ(a.timestampMs, 5u);
}

TEST(att_gravity_rolled_and_pitched_90_rest_is_zero)
{
    GravityCompensator gc;
    // roll +90: right side down -> body y points down -> f_body = R^T (0,0,g) = (0, -g, 0)
    const Quaternion qr = quat::fromEulerNedZyx(quat::kPi / 2, 0, 0);
    const V3 fr = attsim::mulT(attsim::fromNedEuler(attsim::kPi / 2, 0, 0), V3{0, 0, attsim::kG});
    CHECK(nearV(attsim::toF(fr), V3{0, -attsim::kG, 0}, 1e-5));
    CHECK(nearV(gc.compute(qr, true, true, attsim::toF(fr), true, 0).enu_mps2, V3{0, 0, 0}, 1e-4));
    // pitch +90: nose up -> body x points up -> f_body = (g, 0, 0)
    const Quaternion qp = quat::fromEulerNedZyx(0, quat::kPi / 2, 0);
    CHECK(nearV(gc.compute(qp, true, true, Vec3f{kG, 0, 0}, true, 0).enu_mps2, V3{0, 0, 0}, 1e-4));
}

TEST(att_gravity_known_vertical_accelerations)
{
    GravityCompensator gc;
    sim::Rng rng(31);
    for (double aUp : {2.0, -2.0}) {
        for (int k = 0; k < 20; ++k) {
            const double r = (rng.uniform() - 0.5) * 3, p = (rng.uniform() - 0.5) * 3, y = (rng.uniform() - 0.5) * 6;
            const M3 R = attsim::fromNedEuler(r, p, y);
            const V3 f = attsim::mulT(R, V3{0, 0, aUp + attsim::kG});  // f = R^T (a - g_ENU)
            const Quaternion q = quat::fromEulerNedZyx(float(r), float(p), float(y));
            const LinearAcceleration a = gc.compute(q, true, true, attsim::toF(f), true, 0);
            CHECK(a.valid);
            CHECK(nearV(a.enu_mps2, expectedLinear(q, attsim::toF(f)), 1e-4));
            CHECK_NEAR(a.vertical_mps2, aUp, 1e-4);
            CHECK_NEAR(a.enu_mps2.x, 0.0, 1e-4);
            CHECK_NEAR(a.enu_mps2.y, 0.0, 1e-4);
        }
    }
}

TEST(att_gravity_coordinated_turn_gives_horizontal_centripetal_only)
{
    // Simulation assumption: accelerometer at the rotation centre, steady coordinated
    // turn, 30 deg bank, heading north (turning right = toward east). Centripetal
    // acceleration g tan(30) points EAST, vertical 0; specific force along body -z.
    GravityCompensator gc;
    const double bank = 30 * attsim::kDeg;
    const double ac = attsim::kG * std::tan(bank);
    const M3 R = attsim::fromNedEuler(bank, 0, 0);
    const V3 f = attsim::mulT(R, V3{ac, 0, attsim::kG});
    CHECK_NEAR(f.x, 0.0, 1e-9);
    CHECK_NEAR(f.y, 0.0, 1e-9);  // no lateral specific force: "coordinated"
    CHECK_NEAR(f.z, -attsim::kG / std::cos(bank), 1e-9);
    const LinearAcceleration a = gc.compute(quat::fromEulerNedZyx(float(bank), 0, 0), true, true, attsim::toF(f), true, 0);
    CHECK(nearV(a.enu_mps2, V3{ac, 0, 0}, 1e-4));
    // Pure yaw spin at the centre (no translation): f stays (0,0,-g) -> zero.
    CHECK(nearV(gc.compute(quat::fromEulerNedZyx(0, 0, 2.0f), true, true, Vec3f{0, 0, -kG}, true, 0).enu_mps2,
                V3{0, 0, 0}, 1e-5));
}

TEST(att_gravity_invalid_quaternion_rejected)
{
    GravityCompensator gc;
    const LinearAcceleration a = gc.compute(Quaternion{std::nanf(""), 0, 0, 0}, true, true, Vec3f{0, 0, -kG}, true, 0);
    CHECK(!a.valid && !a.reliable);
    CHECK(std::isnan(a.vertical_mps2));  // never a plausible-looking 0
    CHECK(!gc.compute(Quaternion{0, 0, 0, 0}, true, true, Vec3f{0, 0, -kG}, true, 0).valid);
}

TEST(att_gravity_unnormalized_quaternion_policy)
{
    GravityCompensator gc;
    const Quaternion q = quat::fromEulerNedZyx(0.4f, -0.2f, 1.0f);
    const float s = 1.005f;  // round-off sized deviation: renormalised
    const Quaternion qs{q.w * s, q.x * s, q.y * s, q.z * s};
    const Vec3f f = attsim::toF(attsim::mulT(attsim::fromNedEuler(0.4, -0.2, 1.0), V3{0, 0, attsim::kG}));
    const LinearAcceleration a = gc.compute(qs, true, true, f, true, 0);
    CHECK(a.valid);
    CHECK(nearV(a.enu_mps2, V3{0, 0, 0}, 1e-4));
    const Quaternion qb{q.w * 1.2f, q.x * 1.2f, q.y * 1.2f, q.z * 1.2f};  // corrupted: rejected
    CHECK(!gc.compute(qb, true, true, f, true, 0).valid);
}

TEST(att_gravity_invalid_accel_or_orientation_is_invalid_not_zero)
{
    GravityCompensator gc;
    const Quaternion q = quat::identity();
    LinearAcceleration a = gc.compute(q, true, true, Vec3f{0, 0, -kG}, false, 0);
    CHECK(!a.valid && std::isnan(a.vertical_mps2));
    a = gc.compute(q, true, true, Vec3f{std::nanf(""), 0, 0}, true, 0);
    CHECK(!a.valid);
    a = gc.compute(q, false, false, Vec3f{0, 0, -kG}, true, 0);  // orientation unavailable
    CHECK(!a.valid && !a.reliable && std::isnan(a.vertical_mps2));
    // valid orientation but not trusted (Degraded): valid, NOT reliable
    a = gc.compute(quat::fromEulerNedZyx(0, 0, 0), true, false, Vec3f{0, 0, -kG}, true, 0);
    CHECK(a.valid && !a.reliable);
}

TEST(att_gravity_free_fall_reads_minus_g)
{
    GravityCompensator gc;
    const LinearAcceleration a = gc.compute(quat::fromEulerNedZyx(0.3f, 0.2f, 0.1f), true, true, Vec3f{0, 0, 0}, true, 0);
    CHECK(a.valid);
    CHECK_NEAR(a.vertical_mps2, -attsim::kG, 1e-5);
}

TEST(att_gravity_sensitivity_to_tilt_error_and_bias)
{
    // Truth level at rest; estimate tilted by 1 deg: horizontal ~ g sin 1deg, vertical ~ g (1 - cos 1deg).
    GravityCompensator gc;
    const LinearAcceleration a =
        gc.compute(quat::fromEulerNedZyx(1 * quat::kRadPerDeg, 0, 0), true, true, Vec3f{0, 0, -kG}, true, 0);
    const double h = std::hypot(a.enu_mps2.x, a.enu_mps2.y);
    CHECK_NEAR(h, attsim::kG * std::sin(attsim::kDeg), 2e-4);
    CHECK_NEAR(a.vertical_mps2, -attsim::kG * (1 - std::cos(attsim::kDeg)), 2e-4);
    std::printf("    [metric] 1 deg tilt error -> horizontal %.4f m/s^2, vertical %.5f m/s^2\n", h,
                double(a.vertical_mps2));
    // A +0.05 m/s^2 bias on body z (level) appears 1:1 (sign: body z is down) in vertical.
    const LinearAcceleration b = gc.compute(quat::fromEulerNedZyx(0, 0, 0), true, true, Vec3f{0, 0, -kG + 0.05f}, true, 0);
    CHECK_NEAR(b.vertical_mps2, -0.05, 1e-5);
    std::printf("    [metric] accel bias +0.05 m/s^2 on body z (level) -> vertical %.4f m/s^2\n",
                double(b.vertical_mps2));
}
