/**
 ******************************************************************************
 * @file    ConstantAccelerationModel.cpp
 * @brief   See ConstantAccelerationModel.hpp.
 ******************************************************************************
 */
#include "Algorithms/Navigation/ConstantAccelerationModel.hpp"

#include <cmath>

namespace pg {

VerticalStateVec ConstantAccelerationModel::propagate(const VerticalStateVec& x, float dt_s,
                                                      const ProcessModelInput& /*u*/) const
{
    VerticalStateVec out{};
    out[vstate::kAltitude] = x[vstate::kAltitude] + x[vstate::kVelocity] * dt_s +
                             0.5f * x[vstate::kAcceleration] * dt_s * dt_s;
    out[vstate::kVelocity] = x[vstate::kVelocity] + x[vstate::kAcceleration] * dt_s;
    out[vstate::kAcceleration] = x[vstate::kAcceleration];
    return out;
}

void ConstantAccelerationModel::processNoise(float dt_s, const VerticalStateVec& /*x*/,
                                             const ProcessModelInput& /*u*/, VerticalCov& Q) const
{
    const float t1 = dt_s;
    const float t2 = t1 * t1;
    const float t3 = t2 * t1;
    const float t4 = t3 * t1;
    const float t5 = t4 * t1;
    Q.a[0][0] = qj_ * t5 / 20.0f + qa_ * t3 / 3.0f;
    Q.a[0][1] = qj_ * t4 / 8.0f + qa_ * t2 / 2.0f;
    Q.a[0][2] = qj_ * t3 / 6.0f;
    Q.a[1][0] = Q.a[0][1];
    Q.a[1][1] = qj_ * t3 / 3.0f + qa_ * t1;
    Q.a[1][2] = qj_ * t2 / 2.0f;
    Q.a[2][0] = Q.a[0][2];
    Q.a[2][1] = Q.a[1][2];
    Q.a[2][2] = qj_ * t1;
}

bool ConstantAccelerationModel::parametersValid() const
{
    return std::isfinite(qj_) && qj_ > 0.0f && std::isfinite(qa_) && qa_ >= 0.0f;
}

}  // namespace pg
