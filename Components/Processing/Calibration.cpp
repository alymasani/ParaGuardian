/**
 ******************************************************************************
 * @file    Calibration.cpp
 * @brief   See Calibration.hpp.
 ******************************************************************************
 */
#include "Processing/Calibration.hpp"

#include <cmath>
#include <limits>

namespace pg {

// ---------------------------------------------------------------- gyro ------

void GyroBiasCalibrator::begin()
{
    stats_.reset();
    collecting_ = true;
}

void GyroBiasCalibrator::add(const Vec3f& gyro)
{
    if (collecting_ && vec::isFinite(gyro)) {
        stats_.add(gyro);
    }
}

bool GyroBiasCalibrator::finish(uint32_t nowMs, uint32_t sessionId)
{
    if (!collecting_) {
        return false;
    }
    collecting_ = false;
    attempted_ = true;

    const uint32_t n = stats_.count();
    if (n < cfg_.minSamples) {
        lastFailure_ = CalibrationFailure::InsufficientSamples;
        return false;
    }
    const Vec3f mean = stats_.mean();
    const Vec3f sd = stats_.stddev();
    if (!vec::isFinite(mean) || !vec::isFinite(sd)) {
        lastFailure_ = CalibrationFailure::InvalidValue;
        return false;
    }
    if (stats_.maxStddev() > cfg_.maxStd_radps) {
        lastFailure_ = CalibrationFailure::TooNoisy;
        return false;
    }
    if (std::fabs(mean.x) > cfg_.maxAbsBias_radps || std::fabs(mean.y) > cfg_.maxAbsBias_radps ||
        std::fabs(mean.z) > cfg_.maxAbsBias_radps) {
        lastFailure_ = CalibrationFailure::InvalidValue;
        return false;
    }

    const float invSqrtN = 1.0f / std::sqrt(static_cast<float>(n));
    active_.bias_radps = mean;
    active_.noiseStd_radps = sd;
    active_.biasStdError_radps = vec::scale(sd, invSqrtN);
    active_.samples = n;
    active_.timestampMs = nowMs;
    active_.sessionId = sessionId;
    active_.valid = true;
    lastFailure_ = CalibrationFailure::None;
    return true;
}

CalibrationState GyroBiasCalibrator::state() const
{
    if (collecting_) {
        return CalibrationState::Collecting;
    }
    if (active_.valid) {
        return CalibrationState::Valid;
    }
    return attempted_ ? CalibrationState::Failed : CalibrationState::NotCalibrated;
}

// ---------------------------------------------------------------- baro ------

void BaroReferenceCalibrator::begin(uint32_t startMs)
{
    collecting_ = true;
    restart(startMs);
}

void BaroReferenceCalibrator::restart(uint32_t startMs)
{
    stats_.reset();
    segments_.reset(startMs, cfg_.segmentMs);
}

void BaroReferenceCalibrator::add(uint32_t timestampMs, float pressure_pa)
{
    if (!collecting_ || !std::isfinite(pressure_pa) || pressure_pa <= 0.0f) {
        return;
    }
    stats_.add(pressure_pa);
    segments_.add(timestampMs, pressure_pa);
}

bool BaroReferenceCalibrator::finish(uint32_t nowMs, uint32_t sessionId)
{
    if (!collecting_) {
        return false;
    }
    collecting_ = false;
    attempted_ = true;
    active_.valid = false;  // replaced below on success; never kept from an older session

    const uint32_t n = stats_.count();
    if (n < cfg_.minSamples) {
        lastFailure_ = CalibrationFailure::InsufficientSamples;
        return false;
    }
    const double p0 = stats_.mean();
    if (!std::isfinite(p0) || p0 <= 0.0) {
        lastFailure_ = CalibrationFailure::InvalidValue;
        return false;
    }
    const double sd = stats_.stddev();
    if (sd > static_cast<double>(cfg_.maxStd_pa)) {
        lastFailure_ = CalibrationFailure::TooNoisy;
        return false;
    }
    // Ignore almost-empty segments (the last partial one): at 50 Hz a full 1 s segment has 50 samples.
    const double spread = segments_.spread(5u);
    if (spread > static_cast<double>(cfg_.maxSegmentSpread_pa)) {
        lastFailure_ = CalibrationFailure::Drifting;
        return false;
    }

    active_.referencePressure_pa = p0;
    active_.scaleHeight_m = (cfg_.altitudeModel == BaroAltitudeModel::IsaAtReference) ? baro::isaScaleHeight_m(p0)
                                                                                     : baro::kIsaScaleHeight_m;
    active_.pressureStd_pa = static_cast<float>(sd);
    active_.segmentSpread_pa = static_cast<float>(spread);
    active_.samples = n;
    active_.timestampMs = nowMs;
    active_.sessionId = sessionId;
    active_.valid = true;
    lastFailure_ = CalibrationFailure::None;
    return true;
}

CalibrationState BaroReferenceCalibrator::state() const
{
    if (collecting_) {
        return CalibrationState::Collecting;
    }
    if (active_.valid) {
        return CalibrationState::Valid;
    }
    return attempted_ ? CalibrationState::Failed : CalibrationState::NotCalibrated;
}

namespace baro {

double relativeAltitude_m(double p_pa, double p0_pa)
{
    return relativeAltitude_m(p_pa, p0_pa, kIsaScaleHeight_m);
}

double relativeAltitude_m(double p_pa, double p0_pa, double scaleHeight_m)
{
    if (!std::isfinite(p_pa) || !std::isfinite(p0_pa) || !std::isfinite(scaleHeight_m) || p_pa <= 0.0 ||
        p0_pa <= 0.0 || scaleHeight_m <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return scaleHeight_m * (1.0 - std::pow(p_pa / p0_pa, kIsaExponent));
}

double isaScaleHeight_m(double p0_pa)
{
    if (!std::isfinite(p0_pa) || p0_pa <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return kIsaScaleHeight_m * std::pow(p0_pa / kIsaSeaLevelPressure_pa, kIsaExponent);
}

}  // namespace baro

}  // namespace pg
