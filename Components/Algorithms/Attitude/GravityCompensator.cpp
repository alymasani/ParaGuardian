/**
 ******************************************************************************
 * @file    GravityCompensator.cpp
 * @brief   See GravityCompensator.hpp.
 ******************************************************************************
 */
#include "Algorithms/Attitude/GravityCompensator.hpp"

#include <cmath>

namespace pg {

LinearAcceleration GravityCompensator::compute(const Quaternion& q, bool orientationValid, bool orientationTrusted,
                                               const Vec3f& f_body, bool accelValid, uint32_t accelTimestampMs) const
{
    const float nan = std::nanf("");
    LinearAcceleration out{Vec3f{nan, nan, nan}, nan, accelTimestampMs, false, false};

    if (!orientationValid || !accelValid || !vec::isFinite(f_body) || !quat::isFinite(q) || !std::isfinite(g_)) {
        return out;
    }
    const float n = quat::norm(q);
    if (std::fabs(n - 1.0f) > kNormTolerance) {
        return out;  // not a unit quaternion: corrupted, do not repair
    }
    Quaternion u{};
    if (!quat::normalize(q, u)) {
        return out;
    }
    const Vec3f fEnu = quat::rotate(u, f_body);
    const Vec3f a{fEnu.x, fEnu.y, fEnu.z - g_};
    if (!vec::isFinite(a)) {
        return out;
    }
    out.enu_mps2 = a;
    out.vertical_mps2 = a.z;
    out.valid = true;
    out.reliable = orientationTrusted;
    return out;
}

}  // namespace pg
