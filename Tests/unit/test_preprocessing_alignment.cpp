/**
 * Preprocessing milestone - sensor-to-body axis alignment and frame conventions.
 */
#include <cmath>

#include "TestFramework.hpp"
#include "sim/SensorSimulator.hpp"

#include "Processing/AxisAlignment.hpp"

using namespace pg;

namespace {
constexpr float kPi = 3.14159265358979f;

bool near(const Vec3f& a, const Vec3f& b, float tol)
{
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol;
}
}  // namespace

TEST(prep_align_identity_placeholder_is_flagged_unverified)
{
    AxisAlignment al;
    const Vec3f v{1.0f, -2.0f, 3.0f};
    CHECK(near(al.toBody(SensorId::Imu0, v), v, 0.0f));
    CHECK(!al.mounting(SensorId::Imu0).verified);
    CHECK(!al.allVerified());
    CHECK(AxisAlignment::isProperRotation(mat::identity()));
}

TEST(prep_align_90_degree_rotations)
{
    AxisAlignment al;
    // Chip rotated +90 deg about body Z (yaw): sensor +X points to body +Y.
    CHECK(al.setMounting(SensorId::Imu1, mat::rotationZ(kPi / 2.0f), false));
    CHECK(near(al.toBody(SensorId::Imu1, Vec3f{1, 0, 0}), Vec3f{0, 1, 0}, 1e-6f));
    CHECK(near(al.toBody(SensorId::Imu1, Vec3f{0, 1, 0}), Vec3f{-1, 0, 0}, 1e-6f));
    CHECK(near(mat::mul(mat::rotationX(kPi / 2.0f), Vec3f{0, 1, 0}), Vec3f{0, 0, 1}, 1e-6f));
    CHECK(near(mat::mul(mat::rotationY(kPi / 2.0f), Vec3f{0, 0, 1}), Vec3f{1, 0, 0}, 1e-6f));
}

TEST(prep_align_180_degree_and_two_axis_sign_change_accepted)
{
    // Chip on the underside of the board: 180 deg about X flips Y and Z.
    const AxisMapEntry flipYZ[3] = {{0, +1}, {1, -1}, {2, -1}};
    Mat3f r{};
    CHECK(AxisAlignment::fromAxisMap(flipYZ, r));
    const Mat3f rx = mat::rotationX(kPi);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) CHECK_NEAR(r.m[i][j], rx.m[i][j], 1e-6);
    AxisAlignment al;
    CHECK(al.setMounting(SensorId::Imu2, r, false));
    CHECK(near(al.toBody(SensorId::Imu2, Vec3f{1, 2, 3}), Vec3f{1, -2, -3}, 0.0f));
}

TEST(prep_align_single_axis_sign_change_rejected_as_mirror)
{
    const AxisMapEntry mirror[3] = {{0, +1}, {1, +1}, {2, -1}};  // det = -1
    Mat3f r{};
    CHECK(!AxisAlignment::fromAxisMap(mirror, r));
    AxisAlignment al;
    const Mat3f m{{{1, 0, 0}, {0, 1, 0}, {0, 0, -1}}};
    CHECK(!al.setMounting(SensorId::Imu0, m, true));
    CHECK_EQ(al.rejectedConfigurations(), 1u);
    CHECK(near(al.toBody(SensorId::Imu0, Vec3f{0, 0, 1}), Vec3f{0, 0, 1}, 0.0f));  // previous kept
}

TEST(prep_align_axis_permutation)
{
    // body X = sensor Y, body Y = -sensor X, body Z = sensor Z  (= rotationZ(+90deg)).
    const AxisMapEntry perm[3] = {{1, +1}, {0, -1}, {2, +1}};
    Mat3f r{};
    CHECK(AxisAlignment::fromAxisMap(perm, r));
    CHECK(near(mat::mul(r, Vec3f{0, 1, 0}), Vec3f{1, 0, 0}, 0.0f));
    const AxisMapEntry repeated[3] = {{0, +1}, {0, +1}, {2, +1}};
    CHECK(!AxisAlignment::fromAxisMap(repeated, r));
    const AxisMapEntry badSign[3] = {{0, +1}, {1, 0}, {2, +1}};
    CHECK(!AxisAlignment::fromAxisMap(badSign, r));
}

TEST(prep_align_rotations_preserve_magnitude)
{
    sim::Rng rng(21);
    const Mat3f r = mat::mul(mat::rotationZ(0.7f), mat::mul(mat::rotationY(-1.1f), mat::rotationX(2.3f)));
    CHECK(AxisAlignment::isProperRotation(r));
    for (int k = 0; k < 200; ++k) {
        const Vec3f v = rng.gaussian3(10.0f);
        CHECK_NEAR(vec::norm(mat::mul(r, v)), vec::norm(v), 1e-4 * (1.0 + vec::norm(v)));
    }
}

TEST(prep_align_invalid_matrices_rejected)
{
    AxisAlignment al;
    const Mat3f scaled{{{2, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const Mat3f sheared{{{1, 0.1f, 0}, {0, 1, 0}, {0, 0, 1}}};
    const Mat3f singular{{{1, 0, 0}, {1, 0, 0}, {0, 0, 1}}};
    Mat3f nan = mat::identity();
    nan.m[1][1] = std::nanf("");
    CHECK(!al.setMounting(SensorId::Imu0, scaled, false));
    CHECK(!al.setMounting(SensorId::Imu0, sheared, false));
    CHECK(!al.setMounting(SensorId::Imu0, singular, false));
    CHECK(!al.setMounting(SensorId::Imu0, nan, false));
    CHECK(!al.setMounting(SensorId::Baro0, mat::identity(), false));  // no vector output
    CHECK_EQ(al.rejectedConfigurations(), 5u);
}

TEST(prep_align_independent_mountings_same_rotation_for_accel_and_gyro)
{
    // Three IMUs mounted differently; the simulator generates each chip's own
    // view of ONE body motion. After alignment all three agree in the body frame.
    sim::SensorSimulator s(22);
    s.imu[0].bodyFromSensor = mat::identity();
    s.imu[1].bodyFromSensor = mat::rotationZ(kPi / 2.0f);
    s.imu[2].bodyFromSensor = mat::rotationX(kPi);
    s.angularRate = [](uint32_t) { return Vec3f{0.3f, -0.2f, 0.5f}; };
    s.specificForce = [](uint32_t) { return Vec3f{1.0f, 0.5f, -9.6f}; };
    AxisAlignment al;
    for (int i = 0; i < 3; ++i) CHECK(al.setMounting(static_cast<SensorId>(i), s.imu[i].bodyFromSensor, false));
    for (const SensorMessage& m : s.step(0)) {
        if (m.type != SensorMessageType::Imu) continue;
        const Vec3f w = al.toBody(m.imu.header.id, m.imu.gyro_radps);
        const Vec3f f = al.toBody(m.imu.header.id, m.imu.accel_mps2);
        CHECK(near(w, Vec3f{0.3f, -0.2f, 0.5f}, 1e-5f));
        CHECK(near(f, Vec3f{1.0f, 0.5f, -9.6f}, 1e-5f));
    }
    // The raw chip readings really were different.
    CHECK(!al.allVerified());
}

TEST(prep_align_ned_to_enu_and_frd_gravity_convention)
{
    // FRD at rest, level: specific force points UP = -Z_body.
    const Vec3f fBody{0.0f, 0.0f, -units::kStandardGravity_mps2};
    // Level and facing north: body = NED, so in ENU the specific force is +Up.
    const Vec3f fEnu = nedToEnu(fBody);
    CHECK(near(fEnu, Vec3f{0.0f, 0.0f, units::kStandardGravity_mps2}, 1e-6f));
    // NED velocity north 3, east 4, down 1 -> ENU (4, 3, -1).
    CHECK(near(nedToEnu(Vec3f{3, 4, 1}), Vec3f{4, 3, -1}, 0.0f));
    // Both frames are right-handed, so the relabelling is a proper rotation
    // (det +1, 180 deg about the N/E bisector); applied twice it is the identity.
    CHECK_NEAR(mat::determinant(kEnuFromNed), 1.0, 1e-6);
    CHECK(AxisAlignment::isProperRotation(kEnuFromNed));
    CHECK(near(nedToEnu(nedToEnu(Vec3f{1, 2, 3})), Vec3f{1, 2, 3}, 0.0f));
}
