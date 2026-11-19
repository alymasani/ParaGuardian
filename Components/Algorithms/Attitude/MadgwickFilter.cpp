/**
 ******************************************************************************
 * @file    MadgwickFilter.cpp
 * @brief   See MadgwickFilter.hpp. Derivations: MADGWICK_ATTITUDE_DESIGN.md section 4.
 ******************************************************************************
 */
#include "Algorithms/Attitude/MadgwickFilter.hpp"

#include <cmath>

namespace pg {

bool MadgwickFilter::reset(const Quaternion& q)
{
    Quaternion n{};
    if (!quat::normalize(q, n)) {
        return false;
    }
    q_ = n;
    return true;
}

MadgwickFilter::Gradient MadgwickFilter::accelGradient(const Quaternion& q, const Vec3f& a)
{
    const float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    // f_g = R(q)^T e_up - a  (third row of R(q), unit-norm form of the (3,3) entry)
    const float f1 = 2.0f * (q1 * q3 - q0 * q2) - a.x;
    const float f2 = 2.0f * (q0 * q1 + q2 * q3) - a.y;
    const float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - a.z;
    // J_g = [[-2q2,  2q3, -2q0, 2q1],
    //        [ 2q1,  2q0,  2q3, 2q2],
    //        [   0, -4q1, -4q2,   0]];  gradient = J_g^T f_g
    Gradient g{};
    g.g[0] = -2.0f * q2 * f1 + 2.0f * q1 * f2;
    g.g[1] = 2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
    g.g[2] = -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
    g.g[3] = 2.0f * q1 * f1 + 2.0f * q2 * f2;
    return g;
}

MadgwickFilter::Gradient MadgwickFilter::magGradient(const Quaternion& q, const Vec3f& m, float bn, float bu)
{
    const float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
    // f_b = R(q)^T (0, bn, bu) - m = bn * (row 2 of R) + bu * (row 3 of R) - m
    const float f1 = 2.0f * bn * (q1 * q2 + q0 * q3) + 2.0f * bu * (q1 * q3 - q0 * q2) - m.x;
    const float f2 = bn * (1.0f - 2.0f * q1 * q1 - 2.0f * q3 * q3) + 2.0f * bu * (q2 * q3 + q0 * q1) - m.y;
    const float f3 = 2.0f * bn * (q2 * q3 - q0 * q1) + bu * (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2) - m.z;
    // Jacobian rows (d/dq0, d/dq1, d/dq2, d/dq3):
    //  f1: [ 2bn q3 - 2bu q2,  2bn q2 + 2bu q3,  2bn q1 - 2bu q0,  2bn q0 + 2bu q1]
    //  f2: [ 2bu q1,          -4bn q1 + 2bu q0,  2bu q3,          -4bn q3 + 2bu q2]
    //  f3: [-2bn q1,          -2bn q0 - 4bu q1,  2bn q3 - 4bu q2,  2bn q2         ]
    Gradient g{};
    g.g[0] = (2.0f * bn * q3 - 2.0f * bu * q2) * f1 + (2.0f * bu * q1) * f2 + (-2.0f * bn * q1) * f3;
    g.g[1] = (2.0f * bn * q2 + 2.0f * bu * q3) * f1 + (-4.0f * bn * q1 + 2.0f * bu * q0) * f2 +
             (-2.0f * bn * q0 - 4.0f * bu * q1) * f3;
    g.g[2] = (2.0f * bn * q1 - 2.0f * bu * q0) * f1 + (2.0f * bu * q3) * f2 + (2.0f * bn * q3 - 4.0f * bu * q2) * f3;
    g.g[3] = (2.0f * bn * q0 + 2.0f * bu * q1) * f1 + (-4.0f * bn * q3 + 2.0f * bu * q2) * f2 + (2.0f * bn * q2) * f3;
    return g;
}

void MadgwickFilter::magReference(const Quaternion& q, const Vec3f& m, float& bn, float& bu)
{
    const Vec3f h = quat::rotate(q, m);  // measured field direction in ENU
    bn = std::sqrt(h.x * h.x + h.y * h.y);
    bu = h.z;
}

bool MadgwickFilter::update(const Vec3f& gyro_radps, const Vec3f* accelUnit, const Vec3f* magUnit, float beta_radps,
                            float dt_s)
{
    if (!(dt_s > 0.0f) || !std::isfinite(dt_s) || !vec::isFinite(gyro_radps) || !std::isfinite(beta_radps)) {
        return false;
    }
    const Quaternion& q = q_;

    // 1. Rate of change from the gyroscope: 1/2 q (x) [0, w]
    const Quaternion qdotW = quat::multiply(q, Quaternion{0.0f, gyro_radps.x, gyro_radps.y, gyro_radps.z});
    float qdot[4] = {0.5f * qdotW.w, 0.5f * qdotW.x, 0.5f * qdotW.y, 0.5f * qdotW.z};

    // 2-4. Normalised gradient-descent correction.
    if (accelUnit != nullptr && beta_radps > 0.0f) {
        Gradient grad = accelGradient(q, *accelUnit);
        if (magUnit != nullptr) {
            float bn = 0.0f;
            float bu = 0.0f;
            magReference(q, *magUnit, bn, bu);
            const Gradient gm = magGradient(q, *magUnit, bn, bu);
            for (int i = 0; i < 4; ++i) grad.g[i] += gm.g[i];
        }
        const float n = std::sqrt(grad.g[0] * grad.g[0] + grad.g[1] * grad.g[1] + grad.g[2] * grad.g[2] +
                                  grad.g[3] * grad.g[3]);
        // n == 0 exactly at the optimum: no direction to move in, no correction.
        if (n > 1e-12f && std::isfinite(n)) {
            const float k = beta_radps / n;
            for (int i = 0; i < 4; ++i) qdot[i] -= k * grad.g[i];
        }
    }

    // 5. Integrate (first order, as in Madgwick's formulation) and renormalise.
    const Quaternion next{q.w + qdot[0] * dt_s, q.x + qdot[1] * dt_s, q.y + qdot[2] * dt_s, q.z + qdot[3] * dt_s};
    Quaternion n{};
    if (!quat::normalize(next, n)) {
        return false;  // keep q_ unchanged
    }
    // Canonical sign (w >= 0): q and -q are the same rotation; this only keeps logs readable.
    if (n.w < 0.0f) {
        n = Quaternion{-n.w, -n.x, -n.y, -n.z};
    }
    q_ = n;
    return true;
}

}  // namespace pg
