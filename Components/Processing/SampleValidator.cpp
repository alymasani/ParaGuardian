/**
 ******************************************************************************
 * @file    SampleValidator.cpp
 * @brief   See SampleValidator.hpp.
 ******************************************************************************
 */
#include "Processing/SampleValidator.hpp"

#include <cmath>

#include "Processing/TimeBase.hpp"
#include "Processing/VectorMath.hpp"

namespace pg {

namespace {

bool finite(float v) { return std::isfinite(v); }

bool anyAbove(const Vec3f& v, float limit)
{
    return std::fabs(v.x) > limit || std::fabs(v.y) > limit || std::fabs(v.z) > limit;
}

void saturatingIncrement(uint32_t& v)
{
    if (v != UINT32_MAX) {
        ++v;
    }
}

}  // namespace

bool SampleValidator::idMatchesType(const SensorMessage& msg, SensorId id)
{
    switch (msg.type) {
        case SensorMessageType::Imu:   return isImu(id);
        case SensorMessageType::Baro:  return isBaro(id);
        case SensorMessageType::Mag:   return id == SensorId::Mag0;
        case SensorMessageType::Gnss:  return id == SensorId::Gnss0;
        case SensorMessageType::Lidar: return id == SensorId::Lidar0;
        case SensorMessageType::None:  break;
    }
    return false;
}

uint32_t SampleValidator::contentReasons(const SensorMessage& msg) const
{
    const SampleHeader* h = headerOf(msg);
    if (h == nullptr) {
        return reject::NoSample;
    }
    uint32_t r = reject::None;
    if (!ok(h->id) || !idMatchesType(msg, h->id)) {
        r |= reject::UnknownSensor;
    }
    if (!h->valid) {
        r |= reject::FlagInvalid;
    }

    switch (msg.type) {
        case SensorMessageType::Imu: {
            const ImuSample& s = msg.imu;
            if (!vec::isFinite(s.accel_mps2) || !vec::isFinite(s.gyro_radps) || !finite(s.temperature_c)) {
                r |= reject::NonFinite;
                break;  // range checks on NaN are meaningless
            }
            if (s.accelSaturated) r |= reject::AccelSaturated;
            if (s.gyroSaturated) r |= reject::GyroSaturated;
            if (anyAbove(s.accel_mps2, cfg_.accelLimit_mps2)) r |= reject::AccelOutOfRange;
            if (anyAbove(s.gyro_radps, cfg_.gyroLimit_radps)) r |= reject::GyroOutOfRange;
            if (!s.accelFresh) r |= reject::AccelNotNew;
            if (!s.gyroFresh) r |= reject::GyroNotNew;
            if (s.temperature_c < cfg_.imuTempMin_c || s.temperature_c > cfg_.imuTempMax_c) {
                r |= reject::TemperatureRange;
            }
            // NOTE: no lower limit on |accel|. Near-zero specific force is free fall,
            // a real (and safety-relevant) condition, not evidence of a dead sensor.
            break;
        }
        case SensorMessageType::Baro: {
            const BaroSample& s = msg.baro;
            if (!finite(s.pressure_pa) || !finite(s.temperature_c)) {
                r |= reject::NonFinite;
                break;
            }
            if (s.pressure_pa <= 0.0f || !s.pressureInRange) r |= reject::PressureRange;
            if (s.temperature_c < cfg_.baroTempMin_c || s.temperature_c > cfg_.baroTempMax_c) {
                r |= reject::TemperatureRange;
            }
            break;
        }
        case SensorMessageType::Mag: {
            const MagSample& s = msg.mag;
            if (!vec::isFinite(s.field_uT)) {
                r |= reject::NonFinite;
                break;
            }
            if (s.saturated) r |= reject::MagSaturated;
            if (anyAbove(s.field_uT, cfg_.magLimit_uT)) r |= reject::MagOutOfRange;
            break;
        }
        case SensorMessageType::Gnss: {
            const GnssSample& s = msg.gnss;
            if (!finite(s.heightMsl_m) || !finite(s.velocityNorth_mps) || !finite(s.velocityEast_mps) ||
                !finite(s.verticalVelocity_mps) || !finite(s.horizontalAccuracy_m) ||
                !finite(s.verticalAccuracy_m) || !finite(s.speedAccuracy_mps)) {
                r |= reject::NonFinite;
                break;
            }
            if (!s.gnssFixOk || !s.positionValid) r |= reject::GnssNoFix;
            if (s.horizontalAccuracy_m < 0.0f || s.verticalAccuracy_m < 0.0f || s.speedAccuracy_mps < 0.0f) {
                r |= reject::GnssBadAccuracy;
            }
            break;
        }
        case SensorMessageType::Lidar: {
            const LidarSample& s = msg.lidar;
            if (!finite(s.distance_m)) {
                r |= reject::NonFinite;
                break;
            }
            if (s.outOfRange || s.distance_m < cfg_.lidarMin_m) r |= reject::LidarOutOfRange;
            if (s.weakSignal) r |= reject::LidarWeakSignal;
            break;
        }
        case SensorMessageType::None:
            r |= reject::NoSample;
            break;
    }
    return r;
}

ValidationResult SampleValidator::validate(const SensorMessage& msg, uint32_t nowMs)
{
    ValidationResult res{};
    res.sequence = SequenceKind::Invalid;
    res.reasons = contentReasons(msg);

    const SampleHeader* h = headerOf(msg);
    if (h == nullptr || (res.reasons & (reject::NoSample | reject::UnknownSensor)) != 0u) {
        saturatingIncrement(totalRejected_);
        for (unsigned b = 0; b < reject::kReasonBits; ++b) {
            if ((res.reasons & (1u << b)) != 0u) saturatingIncrement(reasonCounts_[b]);
        }
        return res;  // cannot attribute to a sensor slot
    }
    const SensorId id = h->id;
    PerSensor& ps = stats_[sensorIndex(id)];

    // ---- time checks (wrap-safe) ----
    const int32_t ahead = timebase::diffMs(h->timestampMs, nowMs);
    if (ahead > static_cast<int32_t>(cfg_.maxFutureMs)) {
        res.reasons |= reject::TimestampFuture;
    }
    res.ageMs = timebase::ageMs(h->timestampMs, nowMs);
    if (res.ageMs > cfg_.maxAgeMs[sensorIndex(id)]) {
        res.reasons |= reject::Stale;
    }

    // ---- sequence checks ----
    const SequenceObservation obs = sequence_.observeHeader(*h);
    res.sequence = obs.kind;
    res.sequenceGap = obs.gap;
    if (obs.kind == SequenceKind::Duplicate) {
        res.reasons |= reject::Duplicate;
    } else if (obs.kind == SequenceKind::OutOfOrder) {
        res.reasons |= reject::OutOfOrder;
    } else if (ps.haveAccepted && timebase::isAfter(ps.lastTimestampMs, h->timestampMs)) {
        // Newer sequence number but an OLDER timestamp than an accepted sample:
        // the clock or the driver is inconsistent; do not let time run backwards.
        res.reasons |= reject::TimestampBackwards;
    }

    res.accepted = (res.reasons & reject::kWholeSampleMask) == 0u;
    if (msg.type == SensorMessageType::Imu) {
        res.accelUsable = res.accepted && (res.reasons & reject::kAccelMask) == 0u;
        res.gyroUsable = res.accepted && (res.reasons & reject::kGyroMask) == 0u;
        // An IMU sample with neither channel usable is rejected as a whole.
        if (!res.accelUsable && !res.gyroUsable) {
            res.accepted = false;
        }
    }

    if (res.accepted) {
        ps.haveAccepted = true;
        ps.lastTimestampMs = h->timestampMs;
        ps.lastSequence = h->sequence;
    }
    record(id, res.reasons, res.accepted);
    return res;
}

void SampleValidator::record(SensorId id, uint32_t reasons, bool accepted)
{
    PerSensor& ps = stats_[sensorIndex(id)];
    if (accepted) {
        saturatingIncrement(ps.accepted);
    } else {
        saturatingIncrement(ps.rejected);
        saturatingIncrement(totalRejected_);
    }
    if (reasons != 0u) {
        ps.lastReasons = reasons;
        for (unsigned b = 0; b < reject::kReasonBits; ++b) {
            if ((reasons & (1u << b)) != 0u) saturatingIncrement(reasonCounts_[b]);
        }
    }
}

}  // namespace pg
