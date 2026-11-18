/**
 ******************************************************************************
 * @file    AxisAlignment.cpp
 * @brief   See AxisAlignment.hpp.
 ******************************************************************************
 */
#include "Processing/AxisAlignment.hpp"

#include <cmath>

namespace pg {

AxisAlignment::AxisAlignment()
{
    for (size_t i = 0; i < kSensorCount; ++i) {
        // PLACEHOLDER: identity, NOT physically verified (board orientation unknown).
        mountings_[i] = SensorMounting{mat::identity(), false};
    }
}

bool AxisAlignment::supports(SensorId id)
{
    return isImu(id) || id == SensorId::Mag0;
}

bool AxisAlignment::isProperRotation(const Mat3f& r)
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!std::isfinite(r.m[i][j])) {
                return false;
            }
        }
    }
    const Mat3f rrt = mat::mul(r, mat::transpose(r));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const float expected = (i == j) ? 1.0f : 0.0f;
            if (std::fabs(rrt.m[i][j] - expected) > kOrthoTolerance) {
                return false;
            }
        }
    }
    return std::fabs(mat::determinant(r) - 1.0f) <= kDetTolerance;
}

bool AxisAlignment::fromAxisMap(const AxisMapEntry (&map)[3], Mat3f& out)
{
    Mat3f r{};
    for (int k = 0; k < 3; ++k) {
        if (map[k].axis > 2u || (map[k].sign != 1 && map[k].sign != -1)) {
            return false;
        }
        r.m[k][map[k].axis] = static_cast<float>(map[k].sign);
    }
    if (!isProperRotation(r)) {
        return false;  // repeated axis (singular) or mirror (det = -1)
    }
    out = r;
    return true;
}

bool AxisAlignment::setMounting(SensorId id, const Mat3f& bodyFromSensor, bool verified)
{
    if (!supports(id) || !isProperRotation(bodyFromSensor)) {
        if (rejected_ != UINT32_MAX) {
            ++rejected_;
        }
        return false;
    }
    mountings_[sensorIndex(id)] = SensorMounting{bodyFromSensor, verified};
    return true;
}

const SensorMounting& AxisAlignment::mounting(SensorId id) const
{
    return mountings_[(sensorIndex(id) < kSensorCount) ? sensorIndex(id) : 0u];
}

bool AxisAlignment::allVerified() const
{
    for (size_t i = 0; i < kSensorCount; ++i) {
        if (supports(static_cast<SensorId>(i)) && !mountings_[i].verified) {
            return false;
        }
    }
    return true;
}

Vec3f AxisAlignment::toBody(SensorId id, const Vec3f& v) const
{
    if (!supports(id)) {
        return v;
    }
    return mat::mul(mountings_[sensorIndex(id)].bodyFromSensor, v);
}

}  // namespace pg
