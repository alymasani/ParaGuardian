/**
 * @file    AttitudeSimulator.hpp
 * @brief   Deterministic attitude "truth" generator and sensor model (host tests only).
 *
 * INDEPENDENCE: this file does NOT use the production quaternion code. Truth
 * orientation is a double-precision rotation matrix R = R_ENU_from_body,
 * propagated with the exact Rodrigues exponential of the body rate over
 * 0.5 ms sub-steps. Truth quaternions (for the 2*acos(|dot|) metric) come from
 * R by Shepperd's method; estimated quaternions are turned into matrices with
 * this file's own formula. Euler -> matrix is written out element by element.
 *
 * PHYSICS (ENU, z up, g_ENU = (0, 0, -g)):
 *   specific force   f_body = R^T (a_ENU - g_ENU) = R^T (a_ENU + (0, 0, g))
 *   gyroscope        w_meas = w_body + bias + noise
 *   magnetometer     m_body = R^T m_ENU (+ disturbance) + noise
 * Accelerating motion is simulated through a_ENU, never by tilting gravity.
 */
#ifndef PARAGUARDIAN_TESTS_ATTITUDE_SIMULATOR_HPP
#define PARAGUARDIAN_TESTS_ATTITUDE_SIMULATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>

#include "Algorithms/Attitude/AttitudeTypes.hpp"
#include "sim/SensorSimulator.hpp"   // sim::Rng

namespace attsim {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.80665;
constexpr double kDeg = kPi / 180.0;

struct V3 {
    double x, y, z;
};
struct M3 {
    double m[3][3];
};
struct Q4 {
    double w, x, y, z;
};

inline V3 add(const V3& a, const V3& b) { return V3{a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3 scale(const V3& a, double s) { return V3{a.x * s, a.y * s, a.z * s}; }
inline double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline double norm(const V3& a) { return std::sqrt(dot(a, a)); }
inline V3 cross(const V3& a, const V3& b)
{
    return V3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline M3 eye() { return M3{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; }
inline V3 mul(const M3& r, const V3& v)
{
    return V3{r.m[0][0] * v.x + r.m[0][1] * v.y + r.m[0][2] * v.z, r.m[1][0] * v.x + r.m[1][1] * v.y + r.m[1][2] * v.z,
              r.m[2][0] * v.x + r.m[2][1] * v.y + r.m[2][2] * v.z};
}
/// R^T v
inline V3 mulT(const M3& r, const V3& v)
{
    return V3{r.m[0][0] * v.x + r.m[1][0] * v.y + r.m[2][0] * v.z, r.m[0][1] * v.x + r.m[1][1] * v.y + r.m[2][1] * v.z,
              r.m[0][2] * v.x + r.m[1][2] * v.y + r.m[2][2] * v.z};
}
inline M3 mul(const M3& a, const M3& b)
{
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}
inline M3 transpose(const M3& a)
{
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[j][i];
    return r;
}
inline double det(const M3& a)
{
    return a.m[0][0] * (a.m[1][1] * a.m[2][2] - a.m[1][2] * a.m[2][1]) -
           a.m[0][1] * (a.m[1][0] * a.m[2][2] - a.m[1][2] * a.m[2][0]) +
           a.m[0][2] * (a.m[1][0] * a.m[2][1] - a.m[1][1] * a.m[2][0]);
}

/// Rodrigues: rotation matrix of rotation vector @p r (axis * angle).
inline M3 expm(const V3& r)
{
    const double th = norm(r);
    if (th < 1e-15) return eye();
    const V3 k = scale(r, 1.0 / th);
    const double c = std::cos(th), s = std::sin(th), v = 1.0 - c;
    return M3{{{c + k.x * k.x * v, k.x * k.y * v - k.z * s, k.x * k.z * v + k.y * s},
               {k.y * k.x * v + k.z * s, c + k.y * k.y * v, k.y * k.z * v - k.x * s},
               {k.z * k.x * v - k.y * s, k.z * k.y * v + k.x * s, c + k.z * k.z * v}}};
}

/// R_ENU_from_body from aerospace Z-Y-X Euler angles (radians) of the FRD body w.r.t. NED.
inline M3 fromNedEuler(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch),
                 cy = std::cos(yaw), sy = std::sin(yaw);
    const M3 rNed{{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                   {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                   {-sp, cp * sr, cp * cr}}};
    const M3 enuFromNed{{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}};
    return mul(enuFromNed, rNed);
}

/// Shepperd's method: unit quaternion (w >= 0) of a rotation matrix (v' = R v  <=>  q v q*).
inline Q4 quatFromMatrix(const M3& r)
{
    const double t = r.m[0][0] + r.m[1][1] + r.m[2][2];
    Q4 q{};
    if (t > 0) {
        const double s = 2.0 * std::sqrt(1.0 + t);
        q = Q4{0.25 * s, (r.m[2][1] - r.m[1][2]) / s, (r.m[0][2] - r.m[2][0]) / s, (r.m[1][0] - r.m[0][1]) / s};
    } else if (r.m[0][0] > r.m[1][1] && r.m[0][0] > r.m[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + r.m[0][0] - r.m[1][1] - r.m[2][2]);
        q = Q4{(r.m[2][1] - r.m[1][2]) / s, 0.25 * s, (r.m[0][1] + r.m[1][0]) / s, (r.m[0][2] + r.m[2][0]) / s};
    } else if (r.m[1][1] > r.m[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + r.m[1][1] - r.m[0][0] - r.m[2][2]);
        q = Q4{(r.m[0][2] - r.m[2][0]) / s, (r.m[0][1] + r.m[1][0]) / s, 0.25 * s, (r.m[1][2] + r.m[2][1]) / s};
    } else {
        const double s = 2.0 * std::sqrt(1.0 + r.m[2][2] - r.m[0][0] - r.m[1][1]);
        q = Q4{(r.m[1][0] - r.m[0][1]) / s, (r.m[0][2] + r.m[2][0]) / s, (r.m[1][2] + r.m[2][1]) / s, 0.25 * s};
    }
    if (q.w < 0) q = Q4{-q.w, -q.x, -q.y, -q.z};
    return q;
}

/// Matrix of a (float) quaternion, written independently of production toMatrix().
inline M3 matrixFromQuat(const pg::Quaternion& qf)
{
    const double w = qf.w, x = qf.x, y = qf.y, z = qf.z;
    const double n = std::sqrt(w * w + x * x + y * y + z * z);
    const double a = w / n, b = x / n, c = y / n, d = z / n;
    return M3{{{a * a + b * b - c * c - d * d, 2 * (b * c - a * d), 2 * (b * d + a * c)},
               {2 * (b * c + a * d), a * a - b * b + c * c - d * d, 2 * (c * d - a * b)},
               {2 * (b * d - a * c), 2 * (c * d + a * b), a * a - b * b - c * c + d * d}}};
}

inline double clamp1(double v) { return v > 1.0 ? 1.0 : (v < -1.0 ? -1.0 : v); }

/// Brief's metric: 2 acos(clamp(|q_true . q_est|)) in DEGREES.
inline double quatErrorDeg(const M3& rTrue, const pg::Quaternion& est)
{
    const Q4 t = quatFromMatrix(rTrue);
    const double n = std::sqrt(double(est.w) * est.w + double(est.x) * est.x + double(est.y) * est.y +
                               double(est.z) * est.z);
    const double d = std::fabs(t.w * est.w + t.x * est.x + t.y * est.y + t.z * est.z) / n;
    return 2.0 * std::acos(std::min(1.0, d)) / kDeg;
}
/// Cross-check metric: rotation angle of R_true^T R_est (degrees).
inline double matrixErrorDeg(const M3& rTrue, const pg::Quaternion& est)
{
    const M3 e = mul(transpose(rTrue), matrixFromQuat(est));
    return std::acos(clamp1((e.m[0][0] + e.m[1][1] + e.m[2][2] - 1.0) / 2.0)) / kDeg;
}
/// Tilt (roll/pitch) error: angle between true and estimated "up" in body axes (degrees). Yaw-independent.
inline double tiltErrorDeg(const M3& rTrue, const pg::Quaternion& est)
{
    const V3 up{0, 0, 1};
    const V3 a = mulT(rTrue, up);
    const V3 b = mulT(matrixFromQuat(est), up);
    return std::acos(clamp1(dot(a, b))) / kDeg;
}
/// Heading of body forward projected on the horizontal plane, degrees clockwise from north.
inline double headingDeg(const M3& r)
{
    const V3 f = mul(r, V3{1, 0, 0});
    return std::atan2(f.x, f.y) / kDeg;
}
inline double wrap180(double a)
{
    while (a > 180.0) a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}
inline double headingErrorDeg(const M3& rTrue, const pg::Quaternion& est)
{
    return wrap180(headingDeg(matrixFromQuat(est)) - headingDeg(rTrue));
}

inline pg::Vec3f toF(const V3& v) { return pg::Vec3f{float(v.x), float(v.y), float(v.z)}; }

/// Smooth "raised cosine" rate pulse: integral over [t0, t0+T] equals @p angle; zero outside.
inline double smoothRate(double t, double t0, double T, double angle)
{
    if (t < t0 || t > t0 + T) return 0.0;
    return angle / T * (1.0 - std::cos(2.0 * kPi * (t - t0) / T));
}

/// Ground truth. Body rate and linear acceleration may depend on time and on the current attitude.
class AttitudeTruth {
public:
    using VecFn = std::function<V3(double tS, const M3& r)>;
    /// Default: level, forward = north. (eye() would be body x = EAST and body z (down) = UP: upside down.)
    M3 R = fromNedEuler(0.0, 0.0, 0.0);
    VecFn omegaBody = [](double, const M3&) { return V3{0, 0, 0}; };
    VecFn accelEnu = [](double, const M3&) { return V3{0, 0, 0}; };
    V3 magEnu{0.0, 20.0, -45.0};  ///< uT: north 20, down 45 (mid-latitude, same as SensorSimulator)

    /// Advances the truth to @p tS (seconds, monotonic) with 0.5 ms sub-steps (midpoint rate).
    void advanceTo(double tS)
    {
        const double h = 0.0005;
        while (t_ + 1e-12 < tS) {
            const double step = std::min(h, tS - t_);
            const V3 w = omegaBody(t_ + 0.5 * step, R);
            R = mul(R, expm(scale(w, step)));  // body rates: right-multiply
            t_ += step;
        }
    }
    double time() const { return t_; }
    void setTime(double tS) { t_ = tS; }
    V3 omega() const { return omegaBody(t_, R); }
    V3 accel() const { return accelEnu(t_, R); }
    V3 specificForceBody() const
    {
        const V3 a = accel();
        return mulT(R, V3{a.x, a.y, a.z + kG});
    }
    V3 magBody() const { return mulT(R, magEnu); }

private:
    double t_ = 0.0;
};

struct SensorErrors {
    V3 gyroBias{0, 0, 0};
    double gyroNoise = 0.0;   ///< rad/s, 1 sigma per axis
    V3 accelBias{0, 0, 0};
    double accelNoise = 0.0;  ///< m/s^2
    double magNoise = 0.0;    ///< uT
};

/// Typical (simulation) noise: LSM6DSRX-class gyro/accel after 3-IMU averaging, conservative x2.
inline SensorErrors realisticErrors()
{
    SensorErrors e;
    e.gyroBias = V3{2e-4, -1.5e-4, 1e-4};  // residual after calibration (milestone 2 sim: <= 2e-4)
    e.gyroNoise = 0.002;
    e.accelNoise = 0.02;
    e.magNoise = 0.2;
    return e;
}

/// Turns truth into AttitudeInput samples at a given time, with noise and fault switches.
class AttitudeHarness {
public:
    explicit AttitudeHarness(uint64_t seed = 7u) : rng(seed) {}

    AttitudeTruth truth;
    SensorErrors err;
    sim::Rng rng;
    uint32_t epochMs = 1000u;      ///< MCU time of truth t = 0
    bool magPresent = false;
    bool magCalibrated = true;
    uint32_t magPeriodMs = 20u;    ///< 50 Hz
    std::function<V3(double tS)> magDisturbanceEnu = [](double) { return V3{0, 0, 0}; };
    std::function<V3(double tS)> accelDisturbanceBody = [](double) { return V3{0, 0, 0}; };

    double tOf(uint32_t ms) const { return double(ms - epochMs) * 1e-3; }

    /// Sample at MCU time @p ms (truth advanced to it). nowMs = ms.
    pg::AttitudeInput sample(uint32_t ms)
    {
        truth.advanceTo(tOf(ms));
        const V3 w = add(add(truth.omega(), err.gyroBias), noise(err.gyroNoise));
        const V3 f = add(add(add(truth.specificForceBody(), err.accelBias), noise(err.accelNoise)),
                         accelDisturbanceBody(tOf(ms)));
        pg::AttitudeInput in{};
        in.nowMs = ms;
        in.gyro_radps = toF(w);
        in.gyroTimestampMs = ms;
        in.gyroValid = true;
        in.gyroCalibrated = true;
        in.gyroSourceMask = 0x7;
        in.gyroStatus = pg::RedundancyStatus::Healthy;
        in.accel_mps2 = toF(f);
        in.accelTimestampMs = ms;
        in.accelValid = true;
        in.accelSourceMask = 0x7;
        in.accelStatus = pg::RedundancyStatus::Healthy;
        if (magPresent) {
            if (!haveMag_ || ms - lastMagMs_ >= magPeriodMs) {
                const V3 m = add(add(truth.magBody(), mulT(truth.R, magDisturbanceEnu(tOf(ms)))), noise(err.magNoise));
                mag_ = toF(m);
                lastMagMs_ = ms;
                haveMag_ = true;
            }
            in.mag_uT = mag_;
            in.magTimestampMs = lastMagMs_;
            in.magValid = true;
            in.magCalibrated = magCalibrated;
        }
        in.mountingVerified = true;
        in.accelCalibrated = true;
        return in;
    }

private:
    V3 noise(double s)
    {
        if (s <= 0.0) return V3{0, 0, 0};
        return V3{rng.gaussian() * s, rng.gaussian() * s, rng.gaussian() * s};
    }
    pg::Vec3f mag_{0, 0, 0};
    uint32_t lastMagMs_ = 0u;
    bool haveMag_ = false;
};

/// Running error statistics (degrees or m/s^2).
struct Stats {
    double max = 0.0, sumSq = 0.0, last = 0.0;
    uint32_t n = 0u;
    void add(double v)
    {
        max = std::max(max, std::fabs(v));
        sumSq += v * v;
        last = v;
        ++n;
    }
    double rms() const { return n ? std::sqrt(sumSq / n) : 0.0; }
};

inline bool finite(const pg::Quaternion& q)
{
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

}  // namespace attsim

#endif  // PARAGUARDIAN_TESTS_ATTITUDE_SIMULATOR_HPP
