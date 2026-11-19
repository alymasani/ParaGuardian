/**
 ******************************************************************************
 * @file    MadgwickFilter.hpp
 * @brief   The Madgwick gradient-descent orientation filter (core mathematics).
 *
 * Reference: S. O. H. Madgwick, "An efficient orientation filter for inertial
 * and inertial/magnetic sensor arrays", University of Bristol report, 2010
 * (and Madgwick, Harrison, Vaidyanathan, IEEE ICORR 2011). This is the
 * ORIGINAL algorithm, not x-io's later "Fusion" AHRS update law.
 *
 * STATE: one unit quaternion q = q_ENU_from_body (see Quaternion.hpp).
 *
 * ONE STEP (dt seconds, body rates w, optional measured directions):
 *   1. gyro rate of change          qdot_w = 1/2 * q (x) [0, w]
 *   2. objective (accelerometer)    f_g(q) = R(q)^T e_up - a_hat          (3x1)
 *      e_up = (0,0,1) ENU. a_hat = measured specific force, normalised: at
 *      rest the accelerometer measures +g UP, so a_hat = "up" in body axes.
 *   3. objective (magnetometer)     f_b(q) = R(q)^T b - m_hat             (3x1)
 *      b = (0, b_n, b_u) ENU: the field's horizontal part points NORTH (+Y),
 *      re-derived every step from the measurement itself:
 *        h = R(q) m_hat,  b_n = sqrt(h_e^2 + h_n^2),  b_u = h_u
 *      so the reference always has the measured dip angle (Madgwick's
 *      "magnetic distortion compensation", adapted from his NWU x-north
 *      reference to ENU y-north).
 *   4. gradient                     grad = J_g^T f_g (+ J_b^T f_b)
 *   5. update  q <- normalise( q + (qdot_w - beta * grad/|grad|) * dt )
 *
 * The gradient is normalised, so the correction ALWAYS moves q at the rate
 * beta (rad/s, in quaternion units) towards agreement, whatever the size of
 * the error. beta therefore trades gyro-drift rejection (larger) against
 * sensitivity to non-gravitational acceleration and noise (smaller).
 *
 * This class performs no gating, timing or state handling - AttitudeEstimator
 * decides WHEN and WITH WHICH measurements it is called.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_ATTITUDE_MADGWICK_FILTER_HPP
#define PARAGUARDIAN_ALGORITHMS_ATTITUDE_MADGWICK_FILTER_HPP

#include "Algorithms/Attitude/Quaternion.hpp"

namespace pg {

class MadgwickFilter {
public:
    /// Gradient vector (d/dw, d/dx, d/dy, d/dz) of the objective.
    struct Gradient {
        float g[4];
    };

    MadgwickFilter() = default;

    /// Sets the orientation. Returns false (state unchanged) if @p q cannot be normalised.
    bool reset(const Quaternion& q);
    const Quaternion& orientation() const { return q_; }

    /**
     * One Madgwick step.
     * @param gyro_radps  body angular rate (bias-corrected)
     * @param accelUnit   normalised specific force, or nullptr = no accelerometer correction
     * @param magUnit     normalised magnetic field, or nullptr = no magnetometer correction
     *                    (only meaningful together with accelUnit)
     * @param beta_radps  correction rate; 0 = pure gyro integration
     * @param dt_s        > 0
     * @return false if the result was not finite/normalisable; the previous
     *         orientation is then KEPT (never replaced by garbage).
     */
    bool update(const Vec3f& gyro_radps, const Vec3f* accelUnit, const Vec3f* magUnit, float beta_radps, float dt_s);

    /// J_g^T f_g for unit quaternion q and unit specific-force direction a (exposed for tests).
    static Gradient accelGradient(const Quaternion& q, const Vec3f& a);
    /// J_b^T f_b with ENU reference b = (0, bn, bu) and unit field m (exposed for tests).
    static Gradient magGradient(const Quaternion& q, const Vec3f& m, float bn, float bu);
    /// ENU reference (bn, bu) for measurement m (unit) at orientation q.
    static void magReference(const Quaternion& q, const Vec3f& m, float& bn, float& bu);

private:
    Quaternion q_ = quat::identity();
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_ATTITUDE_MADGWICK_FILTER_HPP
