/**
 ******************************************************************************
 * @file    GroundClearanceEstimator.cpp
 * @brief   See GroundClearanceEstimator.hpp and FLIGHT_LANDING_DESIGN.md section 6.
 ******************************************************************************
 */
#include "Algorithms/Landing/GroundClearanceEstimator.hpp"

#include <cmath>
#include <limits>

#include "Algorithms/Attitude/Quaternion.hpp"
#include "Processing/ProcessingTypes.hpp"
#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;

void satInc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}
}  // namespace

GroundClearanceEstimator::GroundClearanceEstimator(const GroundClearanceConfig& cfg) : cfg_(cfg), configValid_(false)
{
    configValid_ = cfg_.isValid();
    const Vec3f b = cfg_.beamBody;
    const float n = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
    if (configValid_ && n > 0.0f) beamBodyUnit_ = Vec3f{b.x / n, b.y / n, b.z / n};
    cosMaxTilt_ = std::cos(cfg_.maxBeamTiltDeg * kDegToRad);
    cosHighTilt_ = std::cos(cfg_.highTiltDeg * kDegToRad);
    reset();
}

void GroundClearanceEstimator::reset()
{
    haveSample_ = false;
    lastSampleTs_ = 0u;
    havePrevClearance_ = false;
    prevClearance_ = 0.0f;
    prevClearanceTs_ = 0u;
    rateCount_ = 0u;
    out_ = GroundClearance{};
    setUnavailable(ClearanceStatus::NoLidar);
}

Vec3f GroundClearanceEstimator::beamEnu(const Quaternion& q) const { return quat::rotate(q, beamBodyUnit_); }

void GroundClearanceEstimator::setUnavailable(ClearanceStatus s)
{
    out_.status = s;
    out_.clearanceValid = false;
    out_.clearance_m = kNaN;
    out_.clearanceSigma_m = kNaN;
    out_.lowerBound_m = kNaN;
    out_.closureRateValid = false;
    out_.closureRate_mps = kNaN;
    out_.closureRateSigma_mps = kNaN;
    out_.rateSamples = 0u;
    out_.rateLatencyMs = 0u;
    out_.ttcValid = false;
    out_.timeToContact_s = kNaN;
}

void GroundClearanceEstimator::resetRate()
{
    if (rateCount_ != 0u) out_.flags |= clearflag::RateReset;
    rateCount_ = 0u;
}

void GroundClearanceEstimator::addRateSample(uint32_t ts, float c)
{
    // Drop samples that left the window (time-based, wrap-safe), then append.
    uint32_t keepFrom = 0u;
    while (keepFrom < rateCount_ && timebase::diffMs(ts, rateTs_[keepFrom]) > static_cast<int32_t>(cfg_.rateWindowMs))
        ++keepFrom;
    if (rateCount_ == GroundClearanceConfig::kRateCapacity && keepFrom == 0u) keepFrom = 1u;
    if (keepFrom > 0u) {
        for (uint32_t i = keepFrom; i < rateCount_; ++i) {
            rateTs_[i - keepFrom] = rateTs_[i];
            rateC_[i - keepFrom] = rateC_[i];
        }
        rateCount_ -= keepFrom;
    }
    rateTs_[rateCount_] = ts;
    rateC_[rateCount_] = c;
    ++rateCount_;
}

void GroundClearanceEstimator::fitRate()
{
    out_.closureRateValid = false;
    out_.closureRate_mps = kNaN;
    out_.closureRateSigma_mps = kNaN;
    out_.rateSamples = static_cast<uint8_t>(rateCount_);
    out_.rateLatencyMs = 0u;
    const uint32_t n = rateCount_;
    if (n < cfg_.rateMinSamples) return;
    const uint32_t last = n - 1u;
    const int32_t span = timebase::diffMs(rateTs_[last], rateTs_[0]);
    if (span < static_cast<int32_t>(cfg_.rateMinSpanMs)) return;

    // Least squares c(t) = a + s t; times relative to the newest sample, values relative to it (float precision).
    float mt = 0.0f, mc = 0.0f;
    for (uint32_t i = 0; i < n; ++i) {
        mt += static_cast<float>(timebase::diffMs(rateTs_[i], rateTs_[last])) * 1e-3f;
        mc += rateC_[i] - rateC_[last];
    }
    const float fn = static_cast<float>(n);
    mt /= fn;
    mc /= fn;
    float stt = 0.0f, stc = 0.0f;
    for (uint32_t i = 0; i < n; ++i) {
        const float t = static_cast<float>(timebase::diffMs(rateTs_[i], rateTs_[last])) * 1e-3f - mt;
        stt += t * t;
        stc += t * ((rateC_[i] - rateC_[last]) - mc);
    }
    if (!(stt > 0.0f)) return;
    const float slope = stc / stt;
    float sse = 0.0f;
    for (uint32_t i = 0; i < n; ++i) {
        const float t = static_cast<float>(timebase::diffMs(rateTs_[i], rateTs_[last])) * 1e-3f;
        const float r = (rateC_[i] - rateC_[last]) - (mc + slope * (t - mt));
        sse += r * r;
    }
    const float rms = std::sqrt(sse / static_cast<float>(n - 2u));
    if (!std::isfinite(slope) || !(rms <= cfg_.rateMaxResidual_m)) return;   // spiky or strongly curved: no rate
    // Never claim more precision than the per-sample clearance noise allows.
    const float s = std::fmax(rms, std::isfinite(out_.clearanceSigma_m) ? out_.clearanceSigma_m : rms);
    out_.closureRate_mps = -slope;
    out_.closureRateSigma_mps = s / std::sqrt(stt);
    out_.closureRateValid = true;
    out_.rateLatencyMs = static_cast<uint32_t>(span) / 2u;
}

const GroundClearance& GroundClearanceEstimator::update(const SafetyInput& in)
{
    const uint32_t now = in.nowMs;
    const SafetyLidarInput& L = in.lidar;
    const SafetyAttitudeInput& A = in.attitude;
    out_.flags = 0u;
    if (!cfg_.mountingVerified) out_.flags |= clearflag::MountingUnverified;
    if ((in.frameQualityFlags & quality::MountingUnverified) != 0u) out_.flags |= clearflag::ImuMountingUnverified;

    if (!configValid_) {
        setUnavailable(ClearanceStatus::NoLidar);
        return out_;
    }

    const bool newSample = L.valid && (!haveSample_ || L.timestampMs != lastSampleTs_);

    if (!newSample) {
        if (L.valid && haveSample_) {
            // Same sample as before: carried, never re-processed.
            out_.flags |= clearflag::NotNew;
            satInc(out_.duplicates);
            out_.ageMs = timebase::ageMs(out_.timestampMs, now);
            if (out_.ageMs > cfg_.maxAgeMs) {
                setUnavailable(ClearanceStatus::Stale);
                resetRate();
                satInc(out_.staleUpdates);
            }
            return out_;
        }
        // LiDAR invalid: explain why from the newest rejection, if recent and newer than the last good sample.
        const bool recentReject = L.lastRejectReasons != 0u &&
                                  (!haveSample_ || timebase::isAfter(L.lastRejectTimestampMs, lastSampleTs_)) &&
                                  timebase::ageMs(L.lastRejectTimestampMs, now) <= cfg_.rejectHoldMs;
        resetRate();
        if (recentReject && (L.lastRejectReasons & reject::LidarOutOfRange) != 0u) {
            setUnavailable(ClearanceStatus::BeyondRange);
            satInc(out_.beyondRangeUpdates);
            out_.timestampMs = L.lastRejectTimestampMs;
            out_.ageMs = timebase::ageMs(L.lastRejectTimestampMs, now);
            // Lower bound only with a usable, synchronised beam geometry.
            if (A.orientationValid && (A.qualityFlags & (attitude::GapDetected | attitude::NumericalFault)) == 0u &&
                timebase::ageMs(A.timestampMs, L.lastRejectTimestampMs) + timebase::ageMs(L.lastRejectTimestampMs, A.timestampMs) <=
                    cfg_.maxAttitudeSkewMs) {
                const Vec3f d = beamEnu(A.q);
                const float proj = -d.z;
                out_.beamEnu = d;
                out_.projectionFactor = proj;
                out_.tiltDeg = std::acos(std::fmin(1.0f, std::fmax(-1.0f, proj))) * kRadToDeg;
                if (proj >= cosMaxTilt_) out_.lowerBound_m = cfg_.maxRange_m * proj;
            }
        } else if (recentReject && (L.lastRejectReasons & reject::LidarWeakSignal) != 0u) {
            setUnavailable(ClearanceStatus::WeakSignal);
            satInc(out_.weakSignalUpdates);
        } else if (recentReject) {
            setUnavailable(ClearanceStatus::InvalidMeasurement);
        } else if (haveSample_ || L.lastRejectReasons != 0u) {
            setUnavailable(ClearanceStatus::Stale);
            satInc(out_.staleUpdates);
        } else {
            setUnavailable(ClearanceStatus::NoLidar);
        }
        if (out_.status != ClearanceStatus::BeyondRange) {
            out_.ageMs = haveSample_ ? timebase::ageMs(lastSampleTs_, now) : UINT32_MAX;
        }
        return out_;
    }

    // ---- a new accepted sample ----
    haveSample_ = true;
    lastSampleTs_ = L.timestampMs;
    satInc(out_.samples);
    out_.timestampMs = L.timestampMs;
    out_.ageMs = timebase::ageMs(L.timestampMs, now);
    out_.slantRange_m = L.distance_m;
    out_.beamEnu = Vec3f{kNaN, kNaN, kNaN};
    out_.projectionFactor = kNaN;
    out_.tiltDeg = kNaN;

    if (out_.ageMs > cfg_.maxAgeMs) {
        setUnavailable(ClearanceStatus::Stale);
        resetRate();
        satInc(out_.staleUpdates);
        return out_;
    }
    if (!(L.distance_m >= cfg_.minRange_m) || L.distance_m > cfg_.maxRange_m || !std::isfinite(L.distance_m)) {
        setUnavailable(ClearanceStatus::InvalidMeasurement);
        resetRate();
        return out_;
    }
    if (!A.orientationValid || (A.qualityFlags & (attitude::GapDetected | attitude::NumericalFault)) != 0u) {
        setUnavailable(ClearanceStatus::AttitudeInvalid);
        resetRate();
        satInc(out_.rejectedAttitude);
        return out_;
    }
    const uint32_t skew = timebase::ageMs(A.timestampMs, L.timestampMs) + timebase::ageMs(L.timestampMs, A.timestampMs);
    if (skew > cfg_.maxAttitudeSkewMs) {
        setUnavailable(ClearanceStatus::AttitudeStale);
        resetRate();
        satInc(out_.rejectedAttitude);
        return out_;
    }
    const bool attDegraded = A.state == AttitudeState::Degraded || (A.qualityFlags & attitude::Recovering) != 0u;
    if (attDegraded) out_.flags |= clearflag::AttitudeDegraded;

    const Vec3f d = beamEnu(A.q);
    const float proj = -d.z;   // cos(angle between the beam and straight down)
    out_.beamEnu = d;
    out_.projectionFactor = proj;
    out_.tiltDeg = std::acos(std::fmin(1.0f, std::fmax(-1.0f, proj))) * kRadToDeg;
    if (!(proj >= cosMaxTilt_)) {
        // Beam too far from vertical (or horizontal / upward): no clearance is manufactured.
        setUnavailable(ClearanceStatus::BeamGeometry);
        out_.beamEnu = d;
        out_.projectionFactor = proj;
        resetRate();
        satInc(out_.rejectedGeometry);
        return out_;
    }
    if (proj < cosHighTilt_) out_.flags |= clearflag::HighTilt;

    // Clearance and its 1-sigma: range error projected + tilt error (first and second order).
    const float r = L.distance_m;
    const float c = r * proj;
    const float sinT = std::sqrt(std::fmax(0.0f, 1.0f - proj * proj));
    float sigTiltDeg = attDegraded ? cfg_.degradedAttitudeSigmaDeg : cfg_.attitudeSigmaDeg;
    if (!cfg_.mountingVerified) sigTiltDeg = std::sqrt(sigTiltDeg * sigTiltDeg + cfg_.mountingSigmaDeg * cfg_.mountingSigmaDeg);
    const float sigT = sigTiltDeg * kDegToRad;
    const float sigR = std::fmax(cfg_.rangeSigmaNear_m, cfg_.rangeSigmaFraction * r);
    const float e1 = sigR * proj;
    const float e2 = r * sinT * sigT;
    const float e3 = 0.5f * r * proj * sigT * sigT;
    out_.status = ClearanceStatus::Valid;
    out_.clearanceValid = true;
    out_.clearance_m = c;
    out_.clearanceSigma_m = std::sqrt(e1 * e1 + e2 * e2 + e3 * e3);
    out_.lowerBound_m = kNaN;
    out_.flags |= clearflag::FlatTerrainAssumed;
    satInc(out_.validSamples);

    // Step / discontinuity: compare with the prediction from the previous clearance and rate.
    if (havePrevClearance_) {
        float predicted = prevClearance_;
        if (out_.closureRateValid) {
            predicted -= out_.closureRate_mps * static_cast<float>(timebase::diffMs(L.timestampMs, prevClearanceTs_)) * 1e-3f;
        }
        const float jump = std::fabs(c - predicted);
        if (jump > std::fmax(cfg_.stepThreshold_m, cfg_.stepFraction * std::fmax(c, prevClearance_))) {
            out_.flags |= clearflag::StepChange;
            satInc(out_.stepChanges);
            resetRate();   // a slope across a step would be a fake closure rate
        }
    }
    havePrevClearance_ = true;
    prevClearance_ = c;
    prevClearanceTs_ = L.timestampMs;

    addRateSample(L.timestampMs, c);
    fitRate();

    // Time to contact: constant-closure approximation, only for a clear approach.
    out_.ttcValid = false;
    out_.timeToContact_s = kNaN;
    if (out_.closureRateValid && out_.closureRate_mps > cfg_.ttcMinClosure_mps &&
        out_.closureRate_mps - 2.0f * out_.closureRateSigma_mps > 0.0f) {
        const float ttc = c / out_.closureRate_mps;
        if (ttc <= cfg_.ttcMax_s) {
            out_.ttcValid = true;
            out_.timeToContact_s = ttc;
        }
    }
    return out_;
}

}  // namespace pg
