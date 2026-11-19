/**
 ******************************************************************************
 * @file    Quaternion.hpp
 * @brief   Unit-quaternion mathematics for the attitude estimator.
 *
 * STORAGE: scalar first, struct { w, x, y, z }  (q = w + x i + y j + z k).
 *
 * MEANING (the ONLY convention used in ParaGuardian):
 *   q = q_ENU_from_body. It rotates a vector EXPRESSED IN BODY (FRD)
 *   COORDINATES into the same vector expressed in ENU coordinates:
 *       v_ENU = q (x) [0, v_body] (x) q*        (Hamilton product "(x)")
 *             = R(q) * v_body
 *   R(q) below is the matching 3x3 matrix (columns = body axes seen in ENU).
 *   The inverse direction is v_body = q* (x) [0, v_ENU] (x) q = R(q)^T v_ENU.
 *
 * MULTIPLICATION ORDER (Hamilton, right-handed, i*j = k):
 *   q_A_from_C = q_A_from_B (x) q_B_from_C      (frames chain like matrices)
 *   A body-frame angular rate w_body updates the orientation by RIGHT
 *   multiplication:  dq/dt = 1/2 * q (x) [0, w_body].
 *
 * Every function is allocation-free, header-only and portable (no HAL).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_ATTITUDE_QUATERNION_HPP
#define PARAGUARDIAN_ALGORITHMS_ATTITUDE_QUATERNION_HPP

#include <cmath>

#include "Processing/VectorMath.hpp"
#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

/// Roll / pitch / yaw for DISPLAY ONLY (never propagated). See quat::toEulerNedZyx.
struct EulerAngles {
    float roll;
    float pitch;
    float yaw;
};

namespace quat {

constexpr float kPi = 3.14159265358979f;
constexpr float kDegPerRad = 57.2957795130823f;
constexpr float kRadPerDeg = 0.0174532925199433f;

constexpr Quaternion identity() { return Quaternion{1.0f, 0.0f, 0.0f, 0.0f}; }

/**
 * q_ENU_from_NED: 180 deg rotation about the axis (1, 1, 0)/sqrt(2) (between
 * North and East). Its matrix is AxisAlignment's kEnuFromNed = [[0,1,0],[1,0,0],[0,0,-1]].
 * It is its own inverse (q (x) q = -1, i.e. the identity rotation).
 */
constexpr Quaternion kEnuFromNed{0.0f, 0.70710678118654752f, 0.70710678118654752f, 0.0f};

/// Hamilton product a (x) b.
constexpr Quaternion multiply(const Quaternion& a, const Quaternion& b)
{
    return Quaternion{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

constexpr Quaternion conjugate(const Quaternion& q) { return Quaternion{q.w, -q.x, -q.y, -q.z}; }
constexpr float dot(const Quaternion& a, const Quaternion& b) { return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; }
inline float norm(const Quaternion& q) { return std::sqrt(dot(q, q)); }
inline bool isFinite(const Quaternion& q)
{
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

/// Smallest norm that is still normalised (anything below is treated as a numerical failure).
constexpr float kMinNorm = 1e-6f;

/**
 * Normalises @p q into @p out. Returns false (and leaves @p out untouched) if
 * @p q is not finite or its norm is below kMinNorm: such a value carries no
 * orientation information and must never be "repaired" into a valid-looking one.
 */
inline bool normalize(const Quaternion& q, Quaternion& out)
{
    if (!isFinite(q)) {
        return false;
    }
    const float n = norm(q);
    if (!(n > kMinNorm) || !std::isfinite(n)) {
        return false;
    }
    const float inv = 1.0f / n;
    out = Quaternion{q.w * inv, q.x * inv, q.y * inv, q.z * inv};
    return true;
}

/**
 * v_ENU = R(q) v_body for a UNIT quaternion, without building the matrix:
 *   t = 2 (q_v x v);  v' = v + w t + q_v x t     (q_v = vector part).
 */
inline Vec3f rotate(const Quaternion& q, const Vec3f& v)
{
    const float tx = 2.0f * (q.y * v.z - q.z * v.y);
    const float ty = 2.0f * (q.z * v.x - q.x * v.z);
    const float tz = 2.0f * (q.x * v.y - q.y * v.x);
    return Vec3f{v.x + q.w * tx + (q.y * tz - q.z * ty),
                 v.y + q.w * ty + (q.z * tx - q.x * tz),
                 v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

/// v_body = R(q)^T v_ENU (inverse rotation, unit quaternion).
inline Vec3f rotateInverse(const Quaternion& q, const Vec3f& v) { return rotate(conjugate(q), v); }

/// R(q) for a UNIT quaternion: v_ENU = R * v_body (Mat3f::m[row][col]).
inline Mat3f toMatrix(const Quaternion& q)
{
    const float ww = q.w * q.w, xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    return Mat3f{{{ww + xx - yy - zz, 2.0f * (xy - wz), 2.0f * (xz + wy)},
                  {2.0f * (xy + wz), ww - xx + yy - zz, 2.0f * (yz - wx)},
                  {2.0f * (xz - wy), 2.0f * (yz + wx), ww - xx - yy + zz}}};
}

/// Rotation by @p angleRad about the UNIT axis @p axis (right-hand rule).
inline Quaternion fromAxisAngle(const Vec3f& axis, float angleRad)
{
    const float h = 0.5f * angleRad;
    const float s = std::sin(h);
    return Quaternion{std::cos(h), axis.x * s, axis.y * s, axis.z * s};
}

/**
 * q_ENU_from_body from aerospace Euler angles (radians) of the FRD body
 * relative to NED, sequence Z-Y-X ("yaw, then pitch, then roll", intrinsic):
 *     R_NED_from_body = Rz(yaw) * Ry(pitch) * Rx(roll)
 *     q_ENU_from_body = q_ENU_from_NED (x) q_NED_from_body
 * roll > 0: right side down. pitch > 0: nose up. yaw: heading clockwise from
 * North seen from above (0 = forward points North, +90 deg = East).
 */
inline Quaternion fromEulerNedZyx(float roll, float pitch, float yaw)
{
    const Quaternion qz = Quaternion{std::cos(0.5f * yaw), 0.0f, 0.0f, std::sin(0.5f * yaw)};
    const Quaternion qy = Quaternion{std::cos(0.5f * pitch), 0.0f, std::sin(0.5f * pitch), 0.0f};
    const Quaternion qx = Quaternion{std::cos(0.5f * roll), std::sin(0.5f * roll), 0.0f, 0.0f};
    return multiply(kEnuFromNed, multiply(qz, multiply(qy, qx)));
}

/**
 * Inverse of fromEulerNedZyx (radians): R_NED = C_ned_enu * R(q), then
 *   roll = atan2(R_NED[2][1], R_NED[2][2]), pitch = -asin(R_NED[2][0]),
 *   yaw = atan2(R_NED[1][0], R_NED[0][0])   in (-pi, pi].
 * Near pitch = +-90 deg (gimbal lock) roll and yaw are not separable; the
 * asin argument is clamped so the result stays finite. DISPLAY ONLY.
 */
inline EulerAngles toEulerNedZyx(const Quaternion& qEnuFromBody)
{
    const Quaternion qNed = multiply(kEnuFromNed, qEnuFromBody);  // q_NED_from_ENU == q_ENU_from_NED
    const Mat3f r = toMatrix(qNed);
    float s = -r.m[2][0];
    s = (s > 1.0f) ? 1.0f : ((s < -1.0f) ? -1.0f : s);
    return EulerAngles{std::atan2(r.m[2][1], r.m[2][2]), std::asin(s), std::atan2(r.m[1][0], r.m[0][0])};
}

/**
 * Shortest rotation angle (radians, 0..pi) between two unit quaternions:
 *   2 * acos(clamp(|a . b|, 0, 1)).  |.| because q and -q are the same rotation.
 */
inline float angleBetween(const Quaternion& a, const Quaternion& b)
{
    float d = std::fabs(dot(a, b));
    d = (d > 1.0f) ? 1.0f : d;
    return 2.0f * std::acos(d);
}

/// Angle (radians, 0..pi) between two non-zero vectors; returns pi for degenerate input.
inline float angleBetween(const Vec3f& a, const Vec3f& b)
{
    const float na = vec::norm(a);
    const float nb = vec::norm(b);
    if (!(na > 0.0f) || !(nb > 0.0f)) {
        return kPi;
    }
    float c = vec::dot(a, b) / (na * nb);
    c = (c > 1.0f) ? 1.0f : ((c < -1.0f) ? -1.0f : c);
    return std::acos(c);
}

}  // namespace quat
}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_ATTITUDE_QUATERNION_HPP
