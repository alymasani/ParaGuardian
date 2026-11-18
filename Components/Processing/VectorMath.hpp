/**
 ******************************************************************************
 * @file    VectorMath.hpp
 * @brief   Tiny fixed-size 3-vector / 3x3-matrix helpers for preprocessing.
 *
 * Reuses pg::Vec3f from Sensors/Common/SensorTypes.hpp (no competing type).
 * Everything is header-only, allocation-free and constexpr-friendly so it runs
 * the same on the STM32 (single-precision FPU instructions) and on a laptop.
 *
 * Matrix convention: Mat3f::m[row][col]; y = M * x means
 *   y.x = m[0][0]*x.x + m[0][1]*x.y + m[0][2]*x.z, etc.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_VECTOR_MATH_HPP
#define PARAGUARDIAN_PROCESSING_VECTOR_MATH_HPP

#include <cmath>

#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

struct Mat3f {
    float m[3][3];
};

namespace vec {

constexpr Vec3f make(float x, float y, float z) { return Vec3f{x, y, z}; }
constexpr Vec3f zero() { return Vec3f{0.0f, 0.0f, 0.0f}; }
constexpr Vec3f add(const Vec3f& a, const Vec3f& b) { return Vec3f{a.x + b.x, a.y + b.y, a.z + b.z}; }
constexpr Vec3f sub(const Vec3f& a, const Vec3f& b) { return Vec3f{a.x - b.x, a.y - b.y, a.z - b.z}; }
constexpr Vec3f scale(const Vec3f& a, float s) { return Vec3f{a.x * s, a.y * s, a.z * s}; }
constexpr float dot(const Vec3f& a, const Vec3f& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline float norm(const Vec3f& a) { return std::sqrt(dot(a, a)); }
inline float distance(const Vec3f& a, const Vec3f& b) { return norm(sub(a, b)); }
inline bool isFinite(const Vec3f& a) { return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z); }

/// Component-wise median of three vectors (robust "middle" value).
inline float median3(float a, float b, float c)
{
    const float lo = (a < b) ? a : b;
    const float hi = (a < b) ? b : a;
    return (c < lo) ? lo : ((c > hi) ? hi : c);
}
inline Vec3f median3(const Vec3f& a, const Vec3f& b, const Vec3f& c)
{
    return Vec3f{median3(a.x, b.x, c.x), median3(a.y, b.y, c.y), median3(a.z, b.z, c.z)};
}

}  // namespace vec

namespace mat {

constexpr Mat3f identity() { return Mat3f{{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}}; }

constexpr Vec3f mul(const Mat3f& a, const Vec3f& v)
{
    return Vec3f{a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
                 a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
                 a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}

inline Mat3f mul(const Mat3f& a, const Mat3f& b)
{
    Mat3f r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
        }
    }
    return r;
}

inline Mat3f transpose(const Mat3f& a)
{
    Mat3f r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.m[i][j] = a.m[j][i];
        }
    }
    return r;
}

inline float determinant(const Mat3f& a)
{
    return a.m[0][0] * (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1]) -
           a.m[0][1] * (a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0]) +
           a.m[0][2] * (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]);
}

/// Right-handed rotation by @p angleRad about the X / Y / Z axis (active rotation of a vector).
inline Mat3f rotationX(float angleRad)
{
    const float c = std::cos(angleRad);
    const float s = std::sin(angleRad);
    return Mat3f{{{1.0f, 0.0f, 0.0f}, {0.0f, c, -s}, {0.0f, s, c}}};
}
inline Mat3f rotationY(float angleRad)
{
    const float c = std::cos(angleRad);
    const float s = std::sin(angleRad);
    return Mat3f{{{c, 0.0f, s}, {0.0f, 1.0f, 0.0f}, {-s, 0.0f, c}}};
}
inline Mat3f rotationZ(float angleRad)
{
    const float c = std::cos(angleRad);
    const float s = std::sin(angleRad);
    return Mat3f{{{c, -s, 0.0f}, {s, c, 0.0f}, {0.0f, 0.0f, 1.0f}}};
}

}  // namespace mat

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_VECTOR_MATH_HPP
