/**
 ******************************************************************************
 * @file    AttitudeEstimator.cpp
 * @brief   See AttitudeEstimator.hpp and MADGWICK_ATTITUDE_DESIGN.md.
 ******************************************************************************
 */
#include "Algorithms/Attitude/AttitudeEstimator.hpp"

#include <cmath>

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {

void inc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}

uint32_t ageOrMax(bool present, uint32_t ts, uint32_t now) { return present ? timebase::ageMs(ts, now) : UINT32_MAX; }

bool fresh(bool valid, const Vec3f& v, uint32_t ts, uint32_t now, uint32_t maxAge)
{
    return valid && vec::isFinite(v) && timebase::ageMs(ts, now) <= maxAge;
}

bool isRejection(CorrectionStatus s)
{
    return s == CorrectionStatus::RejectedMagnitude || s == CorrectionStatus::RejectedDirection ||
           s == CorrectionStatus::RejectedInclination || s == CorrectionStatus::RejectedRotation;
}

}  // namespace

AttitudeEstimator::AttitudeEstimator(const AttitudeConfig& cfg)
    : cfg_(cfg), cfgValid_(isValid(cfg)), gravity_(cfg.gravity_mps2)
{
    reset();
}

void AttitudeEstimator::reset()
{
    filter_ = MadgwickFilter{};
    est_ = AttitudeEstimate{};
    state_ = AttitudeState::Uninitialized;
    lastGyroTs_ = 0u;
    initActive_ = false;
    initCount_ = 0u;
    hadOrientation_ = false;
    previousYaw_ = 0.0f;
    recovering_ = false;
    gapPending_ = false;
    quietActive_ = false;
    magRefValid_ = false;
    magAcquireCount_ = 0u;
    everMagAccepted_ = false;
    magDisagreeActive_ = false;
}

bool AttitudeEstimator::tracking() const
{
    return state_ == AttitudeState::Tracking6Axis || state_ == AttitudeState::Tracking9Axis ||
           state_ == AttitudeState::Degraded;
}

void AttitudeEstimator::update(const AttitudeInput& in)
{
    inc(est_.sequence);
    est_.computedAtMs = in.nowMs;
    est_.dtMs = 0u;
    est_.gyroIntegration = GyroIntegration::None;
    est_.accelCorrection = CorrectionStatus::NotAttempted;
    est_.magCorrection = CorrectionStatus::NotAttempted;
    est_.accelErrorDeg = -1.0f;
    est_.magHeadingErrorDeg = 0.0f;
    est_.betaUsed_radps = 0.0f;
    est_.magUsed = false;
    est_.qualityFlags = 0u;

    const bool gyroCalOk = in.gyroCalibrated || !cfg_.requireCalibratedGyro;
    const bool gyroOk = cfgValid_ && gyroCalOk &&
                        fresh(in.gyroValid, in.gyro_radps, in.gyroTimestampMs, in.nowMs, cfg_.gyroMaxAgeMs);
    const bool accelOk = fresh(in.accelValid, in.accel_mps2, in.accelTimestampMs, in.nowMs, cfg_.accelMaxAgeMs);
    const bool magFresh = fresh(in.magValid, in.mag_uT, in.magTimestampMs, in.nowMs, cfg_.magMaxAgeMs);

    if (cfgValid_) {
        if (tracking()) {
            stepTracking(in, gyroOk, accelOk, magFresh);
        } else {
            stepInit(in, gyroOk, accelOk);
        }
    } else {
        state_ = AttitudeState::Invalid;
        est_.qualityFlags |= attitude::InvalidConfig;
    }
    finalize(in, accelOk);
}

// ---------------------------------------------------------------------------
// Initialisation: level from a quasi-static gravity window
// ---------------------------------------------------------------------------
void AttitudeEstimator::stepInit(const AttitudeInput& in, bool gyroOk, bool accelOk)
{
    if (!gyroOk || !accelOk) {
        est_.gyroIntegration = gyroOk ? GyroIntegration::None : GyroIntegration::Unavailable;
        initActive_ = false;  // a continuous window is required
        if (state_ != AttitudeState::Invalid) state_ = AttitudeState::Uninitialized;
        return;
    }
    const uint32_t ts = in.gyroTimestampMs;
    if (initActive_) {
        const int32_t d = timebase::diffMs(ts, initLastMs_);
        if (d == 0) {
            est_.gyroIntegration = GyroIntegration::NotNew;
            inc(est_.notNewSamples);
            return;
        }
        if (d < 0) {
            est_.gyroIntegration = GyroIntegration::OutOfOrder;
            inc(est_.outOfOrderSamples);
            return;
        }
        if (static_cast<uint32_t>(d) > cfg_.maxIntegrationDtMs) {
            initActive_ = false;  // hole in the window: restart
        }
    }

    const float fNorm = vec::norm(in.accel_mps2);
    const bool quiet = vec::norm(in.gyro_radps) <= cfg_.initMaxRate_radps &&
                       std::fabs(fNorm - cfg_.gravity_mps2) <= cfg_.initMaxAccelError_mps2;
    if (state_ != AttitudeState::Invalid) state_ = AttitudeState::Initializing;
    if (!quiet) {
        initActive_ = false;  // moving: levelling from this accelerometer reading would be wrong
        return;
    }
    if (!initActive_) {
        initActive_ = true;
        initStartMs_ = ts;
        initCount_ = 0u;
        initSum_[0] = initSum_[1] = initSum_[2] = 0.0;
    }
    initSum_[0] += static_cast<double>(in.accel_mps2.x);
    initSum_[1] += static_cast<double>(in.accel_mps2.y);
    initSum_[2] += static_cast<double>(in.accel_mps2.z);
    ++initCount_;
    initLastMs_ = ts;

    if (initCount_ >= cfg_.initMinSamples &&
        static_cast<uint32_t>(timebase::diffMs(ts, initStartMs_)) >= cfg_.initWindowMs) {
        level(ts);
    }
}

void AttitudeEstimator::level(uint32_t gyroTs)
{
    const double n = static_cast<double>(initCount_);
    const float fx = static_cast<float>(initSum_[0] / n);
    const float fy = static_cast<float>(initSum_[1] / n);
    const float fz = static_cast<float>(initSum_[2] / n);
    // FRD body, specific force at rest f = R_nb^T (0,0,-g)_NED = g * (sin th, -sin ph cos th, -cos ph cos th):
    const float roll = std::atan2(-fy, -fz);
    const float pitch = std::atan2(fx, std::sqrt(fy * fy + fz * fz));
    // Yaw is NOT observable from gravity: 0 (pseudo-north) the first time, else keep the last known yaw.
    const float yaw = hadOrientation_ ? previousYaw_ : 0.0f;
    initActive_ = false;
    if (!filter_.reset(quat::fromEulerNedZyx(roll, pitch, yaw))) {
        inc(est_.numericalFaults);
        est_.qualityFlags |= attitude::NumericalFault;
        return;
    }
    state_ = AttitudeState::Tracking6Axis;
    lastGyroTs_ = gyroTs;
    lastAccelAcceptedMs_ = gyroTs;
    recovering_ = false;
    gapPending_ = false;
    quietActive_ = false;
    magRefValid_ = false;  // a new yaw origin: any heading reference must be re-acquired
    magAcquireCount_ = 0u;
    everMagAccepted_ = false;
    magDisagreeActive_ = false;
    inc(est_.initializations);
}

void AttitudeEstimator::invalidate()
{
    if (tracking()) {
        hadOrientation_ = true;
        previousYaw_ = quat::toEulerNedZyx(filter_.orientation()).yaw;
    }
    state_ = AttitudeState::Invalid;
    inc(est_.invalidations);
    initActive_ = false;
    recovering_ = false;
    gapPending_ = false;
    quietActive_ = false;
    magRefValid_ = false;
    magAcquireCount_ = 0u;
    everMagAccepted_ = false;
    magDisagreeActive_ = false;
}

// ---------------------------------------------------------------------------
// Tracking
// ---------------------------------------------------------------------------
void AttitudeEstimator::updateQuiet(const AttitudeInput& in, bool accelOk)
{
    const bool quietNow =
        accelOk && vec::norm(in.gyro_radps) <= cfg_.recoveryMaxRate_radps &&
        std::fabs(vec::norm(in.accel_mps2) - cfg_.gravity_mps2) <= cfg_.recoveryMaxAccelError_mps2;
    if (quietNow) {
        if (!quietActive_) {
            quietActive_ = true;
            quietSinceMs_ = in.gyroTimestampMs;
        }
    } else {
        quietActive_ = false;
        recovering_ = false;  // recovery only while the motion is quasi-static
    }
}

const Vec3f* AttitudeEstimator::decideAccel(const AttitudeInput& in, bool accelOk, Vec3f& aUnit, bool& recoveryGain)
{
    recoveryGain = false;
    if (!accelOk) {
        est_.accelCorrection = CorrectionStatus::Unavailable;
        return nullptr;
    }
    const uint32_t ts = in.gyroTimestampMs;
    const float fNorm = vec::norm(in.accel_mps2);
    if (!(std::fabs(fNorm - cfg_.gravity_mps2) <= cfg_.accelMagnitudeTolerance_mps2)) {
        est_.accelCorrection = CorrectionStatus::RejectedMagnitude;  // includes free fall (|f| ~ 0)
        inc(est_.accelRejections);
        return nullptr;
    }
    aUnit = vec::scale(in.accel_mps2, 1.0f / fNorm);
    const Vec3f upBody = quat::rotateInverse(filter_.orientation(), Vec3f{0.0f, 0.0f, 1.0f});
    const float errDeg = quat::angleBetween(aUnit, upBody) * quat::kDegPerRad;
    est_.accelErrorDeg = errDeg;
    const bool turning = vec::norm(in.gyro_radps) > cfg_.accelRejectionRate_radps;

    if (turning) {
        // Turn gate (see AttitudeConfig::accelRejectionRate_radps). The recovery rule
        // cannot apply either: it requires quasi-static motion (|w| small).
        est_.accelCorrection = CorrectionStatus::RejectedRotation;
        inc(est_.accelRejections);
        return nullptr;
    }
    if (errDeg <= cfg_.accelRejectionDeg) {
        est_.accelCorrection = CorrectionStatus::Accepted;
        lastAccelAcceptedMs_ = ts;
        gapPending_ = false;  // the attitude agrees with gravity again
        if (recovering_ && errDeg <= 0.5f * cfg_.accelRejectionDeg) {
            recovering_ = false;  // converged
        }
        recoveryGain = recovering_;
        return &aUnit;
    }

    // Outside the gate. Forced acceptance only with evidence the motion is quasi-static.
    const bool quietLongEnough =
        quietActive_ && static_cast<uint32_t>(timebase::diffMs(ts, quietSinceMs_)) >= cfg_.recoveryQuietMs;
    const bool rejectedLong = static_cast<uint32_t>(timebase::diffMs(ts, lastAccelAcceptedMs_)) >= cfg_.accelRecoveryMs;
    if (cfg_.accelRecoveryMs > 0u && quietLongEnough && (gapPending_ || recovering_ || rejectedLong)) {
        if (!recovering_) {
            recovering_ = true;
            inc(est_.recoveries);
        }
        est_.accelCorrection = CorrectionStatus::AcceptedRecovery;
        lastAccelAcceptedMs_ = ts;
        recoveryGain = true;
        return &aUnit;
    }
    est_.accelCorrection = CorrectionStatus::RejectedDirection;
    inc(est_.accelRejections);
    return nullptr;
}

const Vec3f* AttitudeEstimator::decideMag(const AttitudeInput& in, bool magFresh, bool accelAccepted, Vec3f& mUnit,
                                          bool& recoveryGain)
{
    recoveryGain = false;
    CorrectionStatus& st = est_.magCorrection;
    if (cfg_.magPolicy == MagPolicy::Disabled) {
        st = CorrectionStatus::Disabled;
        return nullptr;
    }
    if (!magFresh) {
        st = CorrectionStatus::Unavailable;
        return nullptr;
    }
    if (cfg_.magPolicy == MagPolicy::CalibratedOnly && !in.magCalibrated) {
        st = CorrectionStatus::Disabled;
        return nullptr;
    }
    if (!accelAccepted) {
        // The Madgwick magnetometer gradient also tilts roll/pitch when the field
        // has a dip angle; without a gravity correction holding the tilt, it is skipped.
        st = CorrectionStatus::NoGravityReference;
        return nullptr;
    }
    const uint32_t ts = in.gyroTimestampMs;
    const float mNorm = vec::norm(in.mag_uT);
    if (!(mNorm >= cfg_.magMinField_uT && mNorm <= cfg_.magMaxField_uT)) {
        st = CorrectionStatus::RejectedMagnitude;
        inc(est_.magRejections);
        magAcquireCount_ = 0u;
        magDisagreeActive_ = false;
        return nullptr;
    }
    mUnit = vec::scale(in.mag_uT, 1.0f / mNorm);
    const Vec3f h = quat::rotate(filter_.orientation(), mUnit);         // field direction in ENU
    const float horiz = std::sqrt(h.x * h.x + h.y * h.y);
    const float inclination = std::atan2(h.z, horiz);
    const float headingErr = std::atan2(h.x, h.y);                     // field azimuth east of +Y (north)
    est_.magHeadingErrorDeg = headingErr * quat::kDegPerRad;

    if (!magRefValid_) {
        // Heading acquisition: a run of consistent samples, then ONE yaw-only rotation
        // (documented discontinuity: the heading was undefined before).
        st = CorrectionStatus::Acquiring;
        if (magAcquireCount_ > 0u && in.magTimestampMs == magAcquireLastTs_) {
            return nullptr;  // same magnetometer sample as the previous gyro step: not new evidence
        }
        if (magAcquireCount_ == 0u ||
            std::fabs(mNorm - magAcquireField_uT_) > cfg_.magFieldTolerance * magAcquireField_uT_) {
            magAcquireCount_ = 0u;
            magAcquireField_uT_ = mNorm;
        }
        ++magAcquireCount_;
        magAcquireLastTs_ = in.magTimestampMs;
        if (magAcquireCount_ >= cfg_.magAcquireSamples) {
            const Quaternion yawFix = quat::fromAxisAngle(Vec3f{0.0f, 0.0f, 1.0f}, headingErr);
            if (filter_.reset(quat::multiply(yawFix, filter_.orientation()))) {
                magRefValid_ = true;
                magRefField_uT_ = mNorm;
                magRefInclination_ = inclination;
                inc(est_.headingAcquisitions);
                everMagAccepted_ = true;
                lastMagAcceptedMs_ = ts;
                lastMagCalibrated_ = in.magCalibrated;
                st = CorrectionStatus::Accepted;
                est_.magUsed = true;
            }
            magAcquireCount_ = 0u;
        }
        return nullptr;  // no gradient step on the acquisition sample
    }

    if (std::fabs(mNorm - magRefField_uT_) > cfg_.magFieldTolerance * magRefField_uT_) {
        st = CorrectionStatus::RejectedMagnitude;
        magDisagreeActive_ = false;
    } else if (std::fabs(inclination - magRefInclination_) > cfg_.magInclinationToleranceDeg * quat::kRadPerDeg) {
        st = CorrectionStatus::RejectedInclination;
        magDisagreeActive_ = false;
    } else if (std::fabs(headingErr) > cfg_.magRejectionDeg * quat::kRadPerDeg) {
        // Self-consistent (strength and dip unchanged) but pointing elsewhere: either a
        // disturbance or gyro heading drift. Accept only if it persists.
        if (!magDisagreeActive_) {
            magDisagreeActive_ = true;
            magDisagreeSinceMs_ = ts;
        }
        if (cfg_.magRecoveryMs > 0u &&
            static_cast<uint32_t>(timebase::diffMs(ts, magDisagreeSinceMs_)) >= cfg_.magRecoveryMs) {
            st = CorrectionStatus::AcceptedRecovery;
            recoveryGain = true;
        } else {
            st = CorrectionStatus::RejectedDirection;
        }
    } else {
        st = CorrectionStatus::Accepted;
        magDisagreeActive_ = false;
    }

    if (isRejection(st)) {
        inc(est_.magRejections);
        return nullptr;
    }
    everMagAccepted_ = true;
    lastMagAcceptedMs_ = ts;
    lastMagCalibrated_ = in.magCalibrated;
    est_.magUsed = true;
    return &mUnit;
}

void AttitudeEstimator::stepTracking(const AttitudeInput& in, bool gyroOk, bool accelOk, bool magFresh)
{
    const uint32_t silenceMs = timebase::ageMs(lastGyroTs_, in.nowMs);
    const int32_t d = gyroOk ? timebase::diffMs(in.gyroTimestampMs, lastGyroTs_) : 0;

    if (!gyroOk || d <= 0) {
        if (!gyroOk) {
            est_.gyroIntegration = GyroIntegration::Unavailable;
        } else if (d == 0) {
            est_.gyroIntegration = GyroIntegration::NotNew;
            inc(est_.notNewSamples);
        } else {
            est_.gyroIntegration = GyroIntegration::OutOfOrder;
            inc(est_.outOfOrderSamples);
        }
        if (silenceMs > cfg_.maxGapMs) {
            inc(est_.gaps);
            invalidate();
        } else if (silenceMs > cfg_.maxIntegrationDtMs) {
            gapPending_ = true;
        }
        return;
    }

    const uint32_t dMs = static_cast<uint32_t>(d);
    if (dMs > cfg_.maxGapMs) {
        inc(est_.gaps);
        invalidate();
        stepInit(in, gyroOk, accelOk);  // this sample may start the new levelling window
        return;
    }
    if (dMs > cfg_.maxIntegrationDtMs) {
        inc(est_.gaps);
        gapPending_ = true;
        lastGyroTs_ = in.gyroTimestampMs;  // resume from here; the hole is not integrated
        est_.gyroIntegration = GyroIntegration::GapSkipped;
        est_.dtMs = dMs;
        quietActive_ = false;
        return;
    }
    if (dMs < cfg_.minDtMs) {
        est_.gyroIntegration = GyroIntegration::NotNew;
        inc(est_.notNewSamples);
        return;
    }

    updateQuiet(in, accelOk);
    Vec3f aUnit{};
    Vec3f mUnit{};
    bool accelRecoveryGain = false;
    bool magRecoveryGain = false;
    const Vec3f* aPtr = decideAccel(in, accelOk, aUnit, accelRecoveryGain);
    const Vec3f* mPtr = decideMag(in, magFresh, aPtr != nullptr, mUnit, magRecoveryGain);

    const float beta = (aPtr == nullptr) ? 0.0f
                       : (accelRecoveryGain || magRecoveryGain) ? cfg_.recoveryBeta_radps
                                                                : cfg_.beta_radps;
    const float dt = static_cast<float>(dMs) * 1e-3f;
    if (!filter_.update(in.gyro_radps, aPtr, mPtr, beta, dt)) {
        inc(est_.numericalFaults);
        est_.qualityFlags |= attitude::NumericalFault;
        invalidate();
        return;
    }
    lastGyroTs_ = in.gyroTimestampMs;
    est_.gyroIntegration = GyroIntegration::Integrated;
    est_.dtMs = dMs;
    est_.betaUsed_radps = beta;
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------
void AttitudeEstimator::finalize(const AttitudeInput& in, bool accelOk)
{
    const uint32_t now = in.nowMs;
    HeadingReference heading = HeadingReference::Unreferenced;
    if (tracking()) {
        const bool prolonged = timebase::ageMs(lastAccelAcceptedMs_, lastGyroTs_) > cfg_.degradedAfterMs ||
                               timebase::ageMs(lastAccelAcceptedMs_, now) > cfg_.degradedAfterMs;
        const bool magActive = everMagAccepted_ && timebase::ageMs(lastMagAcceptedMs_, now) <= cfg_.headingHoldMs;
        if (magRefValid_) {
            heading = magActive ? (lastMagCalibrated_ ? HeadingReference::MagneticReferenced
                                                      : HeadingReference::MagneticUnvalidated)
                                : HeadingReference::MagneticLost;
        }
        if (prolonged) est_.qualityFlags |= attitude::AccelRejectedProlonged;
        state_ = (prolonged || recovering_ || gapPending_)
                     ? AttitudeState::Degraded
                     : ((magActive && magRefValid_) ? AttitudeState::Tracking9Axis : AttitudeState::Tracking6Axis);
    }

    AttitudeEstimate& e = est_;
    e.state = state_;
    e.heading = heading;
    e.convention = RotationConvention::EnuFromFrdBody;
    e.orientationValid = tracking();
    e.timestampMs = lastGyroTs_;
    const float nan = std::nanf("");
    if (e.orientationValid) {
        e.q = filter_.orientation();
        const EulerAngles eu = quat::toEulerNedZyx(e.q);
        e.eulerDeg = EulerAngles{eu.roll * quat::kDegPerRad, eu.pitch * quat::kDegPerRad, eu.yaw * quat::kDegPerRad};
    } else {
        e.q = Quaternion{nan, nan, nan, nan};
        e.eulerDeg = EulerAngles{nan, nan, nan};
    }
    e.yawReferenced = heading == HeadingReference::MagneticReferenced;

    const bool trusted = state_ == AttitudeState::Tracking6Axis || state_ == AttitudeState::Tracking9Axis;
    e.linearAccel = gravity_.compute(e.q, e.orientationValid, trusted, in.accel_mps2, accelOk, in.accelTimestampMs);

    e.gyroAgeMs = ageOrMax(in.gyroValid, in.gyroTimestampMs, now);
    e.accelAgeMs = ageOrMax(in.accelValid, in.accelTimestampMs, now);
    e.magAgeMs = ageOrMax(in.magValid, in.magTimestampMs, now);
    e.sinceAccelAcceptedMs = tracking() ? timebase::ageMs(lastAccelAcceptedMs_, now) : UINT32_MAX;
    e.sinceMagAcceptedMs = (tracking() && everMagAccepted_) ? timebase::ageMs(lastMagAcceptedMs_, now) : UINT32_MAX;
    e.gyroSourceMask = in.gyroSourceMask;
    e.accelSourceMask = in.accelSourceMask;

    uint32_t f = e.qualityFlags;
    if (!e.orientationValid) f |= attitude::NotInitialized;
    if (!in.gyroValid || e.gyroIntegration == GyroIntegration::Unavailable) f |= attitude::GyroUnavailable;
    if (!in.gyroCalibrated) f |= attitude::GyroNotCalibrated;
    if (!accelOk) f |= attitude::AccelUnavailable;
    if (isRejection(e.accelCorrection)) f |= attitude::AccelRejected;
    if (recovering_) f |= attitude::Recovering;
    if (gapPending_) f |= attitude::GapDetected;
    if (heading != HeadingReference::MagneticReferenced) f |= attitude::HeadingUnreferenced;
    if (heading == HeadingReference::MagneticUnvalidated || (e.magUsed && !in.magCalibrated)) {
        f |= attitude::MagUncalibrated;
    }
    if (isRejection(e.magCorrection)) f |= attitude::MagRejected;
    if (!in.mountingVerified) f |= attitude::MountingUnverified;
    if (!in.accelCalibrated) f |= attitude::AccelNotCalibrated;
    if (in.gyroStatus == RedundancyStatus::Degraded || in.accelStatus == RedundancyStatus::Degraded) {
        f |= attitude::ImuRedundancyDegraded;
    }
    if (e.linearAccel.valid && !e.linearAccel.reliable) f |= attitude::LinearAccelUnreliable;
    e.qualityFlags = f;
}

}  // namespace pg
