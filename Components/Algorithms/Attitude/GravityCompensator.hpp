/**
 ******************************************************************************
 * @file    GravityCompensator.hpp
 * @brief   Specific force (body) + orientation -> linear acceleration (ENU).
 *
 * PHYSICS
 *   An accelerometer measures SPECIFIC FORCE f = a - g_vec (it cannot feel
 *   gravity, only the support force opposing it). With g_ENU = (0, 0, -g):
 *       f_ENU = R(q) f_body                 (rotate into ENU)
 *       a_ENU = f_ENU + g_ENU = R(q) f_body + (0, 0, -g)
 *   At rest f_ENU = (0, 0, +g) -> a_ENU = 0. In free fall f = 0 -> a_ENU = (0, 0, -g).
 *
 * QUATERNION POLICY
 *   Non-finite -> rejected. |q| within 1 +- kNormTolerance -> renormalised
 *   (float round-off). Further from 1 -> rejected: the estimator always
 *   outputs unit quaternions, so a large deviation means corruption, and
 *   "fixing" it would hide the fault.
 *
 * SENSITIVITY (documented in the design doc):
 *   tilt error d   -> horizontal error ~ g sin d (0.17 m/s^2 per degree),
 *                     vertical error   ~ g (1 - cos d) (0.0015 m/s^2 at 1 deg)
 *   accel bias b   -> appears 1:1 in a_ENU (rotated): it is NOT removed here.
 *
 * Gravity is only removed; nothing is integrated (no velocity/altitude here).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_ATTITUDE_GRAVITY_COMPENSATOR_HPP
#define PARAGUARDIAN_ALGORITHMS_ATTITUDE_GRAVITY_COMPENSATOR_HPP

#include "Algorithms/Attitude/AttitudeTypes.hpp"

namespace pg {

class GravityCompensator {
public:
    static constexpr float kNormTolerance = 0.01f;

    explicit GravityCompensator(float gravity_mps2 = units::kStandardGravity_mps2) : g_(gravity_mps2) {}

    /**
     * @param q                 q_ENU_from_body
     * @param orientationValid  the estimator says q is a usable orientation
     * @param orientationTrusted the estimator is in a Tracking state (else result is valid but not reliable)
     * @param f_body            specific force, body FRD, m/s^2
     * @param accelValid        f_body is a valid, fresh measurement
     * Invalid inputs give valid = false and a ZERO-FREE result: values are NaN,
     * so an accidental use cannot look like "no acceleration".
     */
    LinearAcceleration compute(const Quaternion& q, bool orientationValid, bool orientationTrusted, const Vec3f& f_body,
                               bool accelValid, uint32_t accelTimestampMs) const;

    float gravity() const { return g_; }

private:
    float g_;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_ATTITUDE_GRAVITY_COMPENSATOR_HPP
