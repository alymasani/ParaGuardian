/**
 ******************************************************************************
 * @file    VerticalNavigationEstimator.cpp
 * @brief   See VerticalNavigationEstimator.hpp and VERTICAL_UKF_DESIGN.md.
 ******************************************************************************
 */
#include "Algorithms/Navigation/VerticalNavigationEstimator.hpp"

#include <cmath>
#include <limits>

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
/// Variance added to altitude when the barometer reference changes: (100 m)^2,
/// i.e. "altitude unknown", so the next barometer sample re-anchors it.
constexpr float kReanchorVariance_m2 = 1.0e4f;

void satInc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}

uint32_t since(bool have, uint32_t t, uint32_t now) { return have ? timebase::ageMs(t, now) : UINT32_MAX; }

void clearPerUpdate(MeasurementDiagnostics& d)
{
    d.last = MeasurementOutcome::NoData;
    d.innovation = kNaN;
    d.innovationSigma = kNaN;
    d.nis = kNaN;
    d.measurementSigma = kNaN;
    d.sampleAgeMs = UINT32_MAX;
}

}  // namespace

VerticalNavigationEstimator::VerticalNavigationEstimator(const NavigationConfig& cfg, const IVerticalProcessModel& model)
    : cfg_(cfg), model_(model), configValid_(false)
{
    configValid_ = cfg_.isValid() && ukf_.configure(cfg_.ukf) && model_.parametersValid();
    reset();
}

void VerticalNavigationEstimator::reset()
{
    state_ = NavState::Uninitialized;
    ukf_.clear();
    tf_ = 0u;
    lastAnyUpdateAt_ = 0u;
    haveAnyUpdate_ = false;
    assimilated_ = false;
    recovering_ = false;
    recoveryConfirm_ = 0u;
    referenceChangedPending_ = false;
    for (SourceTrack& t : tracks_) t = SourceTrack{};
    baroTracker_.reset();
    for (uint32_t j = 0; j < kBaroCount; ++j) {
        baroRef_[j] = 0.0f;
        baroRefKnown_[j] = false;
    }
    lastGnssTow_ = 0u;
    haveGnssTow_ = false;
    initCount_ = 0u;
    est_ = NavigationEstimate{};
    est_.estimator = NavEstimatorId::VerticalUkf;
    est_.processModel = model_.info();
    clearPerUpdate(est_.baro);
    clearPerUpdate(est_.accel);
    clearPerUpdate(est_.gnss);
    est_.baro.sinceAcceptedMs = est_.accel.sinceAcceptedMs = est_.gnss.sinceAcceptedMs = UINT32_MAX;
    est_.outageMs = UINT32_MAX;
    est_.sinceAnyUpdateMs = UINT32_MAX;
    est_.altitude_m = est_.verticalVelocity_mps = est_.verticalAccel_mps2 = kNaN;
    est_.altitudeSigma_m = est_.velocitySigma_mps = est_.accelSigma_mps2 = kNaN;
    for (auto& row : est_.covariance)
        for (float& v : row) v = kNaN;
    est_.state = state_;
    est_.qualityFlags = navflag::NotInitialized;
}

bool VerticalNavigationEstimator::tracking() const
{
    return state_ == NavState::Tracking || state_ == NavState::Degraded || state_ == NavState::Recovering;
}

MeasurementDiagnostics& VerticalNavigationEstimator::diag(Source s)
{
    switch (s) {
        case Source::Baro:  return est_.baro;
        case Source::Accel: return est_.accel;
        default:            return est_.gnss;
    }
}

void VerticalNavigationEstimator::restartInit() { initCount_ = 0u; }

// ---------------------------------------------------------------------------
// update
// ---------------------------------------------------------------------------

const NavigationEstimate& VerticalNavigationEstimator::update(const NavigationInput& in)
{
    beginUpdate(in);
    if (!configValid_) {
        state_ = NavState::Unavailable;
        est_.qualityFlags |= navflag::InvalidConfig;
        finalize(in);
        return est_;
    }

    Candidate c[3];
    uint32_t n = 0u;
    classifyBaro(in, c, n);
    classifyAccel(in, c, n);
    classifyGnss(in, c, n);

    // Process in time order (insertion sort of <= 3 items, wrap-safe comparison).
    for (uint32_t i = 1; i < n; ++i) {
        const Candidate key = c[i];
        uint32_t j = i;
        while (j > 0u && timebase::diffMs(c[j - 1u].ts, key.ts) > 0) {
            c[j] = c[j - 1u];
            --j;
        }
        c[j] = key;
    }

    if (!tracking()) {
        initializationStep(in, c, n);
    } else {
        if (referenceChangedPending_) reanchorAltitude();
        for (uint32_t i = 0; i < n && tracking(); ++i) processCandidate(in, c[i]);
    }

    // Outage / idle: nothing new assimilated for idlePredictMs -> predict to now so the
    // covariance reflects the elapsed time (bounded by maxPredictionMs).
    if (tracking() && !assimilated_ && timebase::diffMs(in.nowMs, tf_) >= static_cast<int32_t>(cfg_.idlePredictMs)) {
        predictTo(in.nowMs, in);
    }
    if (tracking() && !assimilated_) est_.qualityFlags |= navflag::PredictionOnly;

    updateState(in.nowMs);
    finalize(in);
    return est_;
}

void VerticalNavigationEstimator::beginUpdate(const NavigationInput& in)
{
    satInc(est_.sequence);
    est_.computedAtMs = in.nowMs;
    est_.qualityFlags = 0u;
    assimilated_ = false;
    clearPerUpdate(est_.baro);
    clearPerUpdate(est_.accel);
    clearPerUpdate(est_.gnss);
    if (state_ == NavState::NumericalFault) {
        state_ = NavState::Initializing;   // the fault was reported in the previous output
    }
}

// ---------------------------------------------------------------------------
// classification: which samples are new, usable, and with what noise
// ---------------------------------------------------------------------------

void VerticalNavigationEstimator::classifyBaro(const NavigationInput& in, Candidate* c, uint32_t& n)
{
    MeasurementDiagnostics& d = est_.baro;
    if (!in.baroValid) {
        d.last = (in.baroStatus == RedundancyStatus::Ambiguous) ? MeasurementOutcome::Ambiguous
                                                                 : MeasurementOutcome::NoData;
        return;
    }
    d.sampleAgeMs = timebase::ageMs(in.baroTimestampMs, in.nowMs);
    if (timebase::isAfter(in.baroTimestampMs, in.nowMs)) {
        d.last = MeasurementOutcome::FutureTimestamp;
        return;
    }

    // Reference pressure of a contributing barometer changed (new calibration
    // session): altitudes before and after are relative to different zeros.
    bool refChanged = false;
    for (uint32_t j = 0; j < kBaroCount; ++j) {
        if ((in.baroContributingMask & (1u << j)) == 0u) continue;
        if (baroRefKnown_[j] && in.baroReferencePressure_pa[j] != baroRef_[j]) refChanged = true;
        baroRef_[j] = in.baroReferencePressure_pa[j];
        baroRefKnown_[j] = true;
    }
    if (refChanged) {
        est_.qualityFlags |= navflag::ReferenceChanged;
        if (tracking()) {
            referenceChangedPending_ = true;
        } else {
            restartInit();   // a half-collected window mixes two references
        }
    }
    if (in.baroSourceChanged) est_.qualityFlags |= navflag::BaroSourceChanged;
    if (in.baroStatus == RedundancyStatus::Degraded) est_.qualityFlags |= navflag::BaroRedundancyDegraded;

    switch (baroTracker_.classify(in)) {
        case BaroSampleTracker::Verdict::NotNew:
            d.last = MeasurementOutcome::NotNew;
            satInc(d.notNew);
            return;
        case BaroSampleTracker::Verdict::OutOfOrder:
            d.last = MeasurementOutcome::OutOfOrder;
            satInc(d.outOfOrder);
            return;
        case BaroSampleTracker::Verdict::New:
            break;
    }
    baroTracker_.markUsed(in);
    if (d.sampleAgeMs > cfg_.baroMaxAgeMs) {
        d.last = MeasurementOutcome::Stale;
        return;
    }
    float sigma = cfg_.baroSigma_m;
    if (in.baroStatus == RedundancyStatus::Degraded) sigma *= cfg_.baroSingleSensorFactor;
    if (in.baroSourceChanged) sigma *= cfg_.baroSourceChangeFactor;
    c[n++] = Candidate{Source::Baro, BaroSampleTracker::observationTime(in), in.baroAltitude_m, sigma};
}

void VerticalNavigationEstimator::classifyAccel(const NavigationInput& in, Candidate* c, uint32_t& n)
{
    MeasurementDiagnostics& d = est_.accel;
    if (!in.accelValid || !std::isfinite(in.accelUp_mps2)) {
        d.last = MeasurementOutcome::NoData;
        return;
    }
    d.sampleAgeMs = timebase::ageMs(in.accelTimestampMs, in.nowMs);
    if (timebase::isAfter(in.accelTimestampMs, in.nowMs)) {
        d.last = MeasurementOutcome::FutureTimestamp;
        return;
    }
    SourceTrack& t = track(Source::Accel);
    if (t.haveLast) {
        const int32_t dd = timebase::diffMs(in.accelTimestampMs, t.lastTs);
        if (dd == 0) {
            d.last = MeasurementOutcome::NotNew;
            satInc(d.notNew);
            return;
        }
        if (dd < 0) {
            d.last = MeasurementOutcome::OutOfOrder;
            satInc(d.outOfOrder);
            return;
        }
    }
    t.lastTs = in.accelTimestampMs;
    t.haveLast = true;
    if (d.sampleAgeMs > cfg_.accelMaxAgeMs) {
        d.last = MeasurementOutcome::Stale;
        return;
    }
    if ((in.attitudeFlags & (attitude::GapDetected | attitude::NumericalFault)) != 0u) {
        // Orientation may be wrong by tens of degrees after a gyro gap: the
        // "vertical" acceleration would be partly horizontal and partly gravity.
        d.last = MeasurementOutcome::RejectedQuality;
        satInc(d.rejectedQuality);
        return;
    }
    // Conservative, NOT statistically calibrated inflation for attitude quality.
    float factor = 1.0f;
    if (!in.accelReliable || (in.attitudeFlags & attitude::Recovering) != 0u) {
        factor *= cfg_.accelUnreliableFactor;
        est_.qualityFlags |= navflag::AttitudeDegraded;
    }
    const CorrectionStatus cs = in.attitudeAccelCorrection;
    if (cs == CorrectionStatus::RejectedRotation || cs == CorrectionStatus::RejectedDirection ||
        cs == CorrectionStatus::RejectedMagnitude) {
        factor *= cfg_.accelTurnGateFactor;
    }
    if ((in.attitudeFlags & attitude::AccelRejectedProlonged) != 0u) factor *= cfg_.accelProlongedRejectFactor;
    if (in.imuAccelStatus == RedundancyStatus::Degraded || in.imuGyroStatus == RedundancyStatus::Degraded) {
        factor *= cfg_.accelImuDegradedFactor;
    }
    if ((in.attitudeFlags & attitude::AccelNotCalibrated) != 0u) est_.qualityFlags |= navflag::AccelNotCalibrated;
    if (factor > cfg_.accelMaxInflation) {
        d.last = MeasurementOutcome::RejectedQuality;
        satInc(d.rejectedQuality);
        return;
    }
    if (factor > 1.0f) est_.qualityFlags |= navflag::AccelInflated;
    c[n++] = Candidate{Source::Accel, in.accelTimestampMs, in.accelUp_mps2, cfg_.accelSigma_mps2 * factor};
}

void VerticalNavigationEstimator::classifyGnss(const NavigationInput& in, Candidate* c, uint32_t& n)
{
    MeasurementDiagnostics& d = est_.gnss;
    if (!in.gnssValid) {
        d.last = MeasurementOutcome::NoData;
        return;
    }
    d.sampleAgeMs = timebase::ageMs(in.gnssTimestampMs, in.nowMs);
    if (timebase::isAfter(in.gnssTimestampMs, in.nowMs)) {
        d.last = MeasurementOutcome::FutureTimestamp;
        return;
    }
    if (!cfg_.useGnssVelocity) {
        d.last = MeasurementOutcome::Disabled;
        return;
    }
    SourceTrack& t = track(Source::Gnss);
    if ((haveGnssTow_ && in.gnssTimeOfWeekMs == lastGnssTow_) ||
        (t.haveLast && timebase::diffMs(in.gnssTimestampMs, t.lastTs) == 0)) {
        d.last = MeasurementOutcome::NotNew;
        satInc(d.notNew);
        return;
    }
    if (t.haveLast && timebase::diffMs(in.gnssTimestampMs, t.lastTs) < 0) {
        d.last = MeasurementOutcome::OutOfOrder;
        satInc(d.outOfOrder);
        return;
    }
    t.lastTs = in.gnssTimestampMs;
    t.haveLast = true;
    lastGnssTow_ = in.gnssTimeOfWeekMs;
    haveGnssTow_ = true;
    if (d.sampleAgeMs > cfg_.gnssMaxAgeMs) {
        d.last = MeasurementOutcome::Stale;
        return;
    }
    const bool fix3d = in.gnssFixType == GnssFixType::Fix3D || in.gnssFixType == GnssFixType::GnssPlusDeadReckoning;
    const float sAcc = in.gnssSpeedAccuracy_mps;
    if (!fix3d || in.gnssNumSatellites < cfg_.gnssMinSatellites || !std::isfinite(sAcc) || !(sAcc > 0.0f) ||
        sAcc > cfg_.gnssMaxSpeedAccuracy_mps || !std::isfinite(in.gnssVerticalVelocity_mps)) {
        d.last = MeasurementOutcome::RejectedQuality;
        satInc(d.rejectedQuality);
        return;
    }
    const float sigma = std::fmax(sAcc * cfg_.gnssSpeedAccuracyScale, cfg_.gnssMinSigma_mps);
    c[n++] = Candidate{Source::Gnss, in.gnssTimestampMs, in.gnssVerticalVelocity_mps, sigma};
}

// ---------------------------------------------------------------------------
// initialisation
// ---------------------------------------------------------------------------

void VerticalNavigationEstimator::initializationStep(const NavigationInput& in, Candidate* c, uint32_t n)
{
    const Candidate* baro = nullptr;
    const Candidate* accel = nullptr;
    for (uint32_t i = 0; i < n; ++i) {
        if (c[i].src == Source::Baro) baro = &c[i];
        if (c[i].src == Source::Accel) accel = &c[i];
    }
    if (baro == nullptr) return;   // Uninitialized / Initializing / Unavailable: wait for the barometer

    if (state_ != NavState::Initializing) restartInit();
    state_ = NavState::Initializing;
    if (initCount_ > 0u) {
        const int32_t gap = timebase::diffMs(baro->ts, initTs_[initCount_ - 1u]);
        if (gap <= 0) return;
        if (static_cast<uint32_t>(gap) > cfg_.initMaxGapMs) restartInit();
    }
    if (initCount_ == NavigationConfig::kInitCapacity) {
        for (uint32_t i = 1; i < initCount_; ++i) {
            initTs_[i - 1u] = initTs_[i];
            initH_[i - 1u] = initH_[i];
        }
        --initCount_;
    }
    initTs_[initCount_] = baro->ts;
    initH_[initCount_] = baro->z;
    ++initCount_;
    est_.baro.last = MeasurementOutcome::UsedForInit;

    if (!tryFitAndStart(in, accel)) return;

    // Started: GNSS of this update (if any) is assimilated normally.
    if (accel != nullptr) est_.accel.last = MeasurementOutcome::UsedForInit;
    for (uint32_t i = 0; i < n && tracking(); ++i) {
        if (c[i].src == Source::Gnss) processCandidate(in, c[i]);
    }
}

bool VerticalNavigationEstimator::tryFitAndStart(const NavigationInput& in, const Candidate* accel)
{
    const uint32_t n = initCount_;
    if (n < cfg_.initMinSamples) return false;
    const uint32_t last = n - 1u;
    if (timebase::diffMs(initTs_[last], initTs_[0]) < static_cast<int32_t>(cfg_.initWindowMs)) return false;

    // Least-squares line h(t) = b + v t, t relative to the newest sample (t <= 0),
    // altitudes relative to the newest sample (float precision).
    float mt = 0.0f;
    float my = 0.0f;
    for (uint32_t i = 0; i < n; ++i) {
        mt += static_cast<float>(timebase::diffMs(initTs_[i], initTs_[last])) * 1e-3f;
        my += initH_[i] - initH_[last];
    }
    const float fn = static_cast<float>(n);
    mt /= fn;
    my /= fn;
    float stt = 0.0f;
    float sty = 0.0f;
    for (uint32_t i = 0; i < n; ++i) {
        const float t = static_cast<float>(timebase::diffMs(initTs_[i], initTs_[last])) * 1e-3f - mt;
        const float y = (initH_[i] - initH_[last]) - my;
        stt += t * t;
        sty += t * y;
    }
    if (!(stt > 0.0f)) return false;
    const float slope = sty / stt;
    const float intercept = my - slope * mt;   // relative altitude at t = 0 (newest sample)
    float sse = 0.0f;
    for (uint32_t i = 0; i < n; ++i) {
        const float t = static_cast<float>(timebase::diffMs(initTs_[i], initTs_[last])) * 1e-3f;
        const float r = (initH_[i] - initH_[last]) - (intercept + slope * t);
        sse += r * r;
    }
    const float s2 = sse / static_cast<float>(n - 2u);
    if (!std::isfinite(s2) || std::sqrt(s2) > cfg_.initMaxResidual_m) {
        // Not a straight line (strong acceleration / outliers): keep the newer half and continue.
        const uint32_t keep = n / 2u;
        for (uint32_t i = 0; i < keep; ++i) {
            initTs_[i] = initTs_[n - keep + i];
            initH_[i] = initH_[n - keep + i];
        }
        initCount_ = keep;
        return false;
    }
    // Never more confident than the barometer noise allows.
    const float sig2 = std::fmax(s2, cfg_.baroSigma_m * cfg_.baroSigma_m);
    float varH = sig2 * (1.0f / fn + mt * mt / stt);
    float varV = sig2 / stt;
    const float covHV = -sig2 * mt / stt;
    varH = std::fmax(varH, cfg_.initMinAltitudeSigma_m * cfg_.initMinAltitudeSigma_m);
    varV = std::fmax(varV, cfg_.initMinVelocitySigma_mps * cfg_.initMinVelocitySigma_mps);

    float a0 = 0.0f;
    float varA = cfg_.initAccelSigma_mps2 * cfg_.initAccelSigma_mps2;
    if (accel != nullptr) {
        a0 = accel->z;
        varA = accel->sigma * accel->sigma;
    }
    VerticalStateVec x{};
    x[vstate::kAltitude] = initH_[last] + intercept;
    x[vstate::kVelocity] = slope;
    x[vstate::kAcceleration] = a0;
    VerticalCov P{};
    P.a[0][0] = varH;
    P.a[0][1] = P.a[1][0] = covHV;
    P.a[1][1] = varV;
    P.a[2][2] = varA;
    tf_ = initTs_[last];
    initCount_ = 0u;
    if (!ukf_.initialize(x, P)) {
        numericalFault(nav::UkfStatus::CovarianceNotPositiveDefinite, in.nowMs);
        return false;
    }
    state_ = NavState::Tracking;
    recovering_ = false;
    referenceChangedPending_ = false;
    haveAnyUpdate_ = true;
    lastAnyUpdateAt_ = tf_;
    SourceTrack& b = track(Source::Baro);
    b.lastAcceptedAt = tf_;
    b.everAccepted = true;
    b.rejectRun = 0u;
    b.lastCandidateTs = tf_;
    b.haveCandidate = true;
    if (accel != nullptr) {
        SourceTrack& a = track(Source::Accel);
        a.lastAcceptedAt = accel->ts;
        a.everAccepted = true;
        a.rejectRun = 0u;
    }
    satInc(est_.initializations);
    if (est_.initializations > 1u) satInc(est_.reinitializations);
    assimilated_ = true;
    return true;
}

// ---------------------------------------------------------------------------
// prediction and measurement update
// ---------------------------------------------------------------------------

bool VerticalNavigationEstimator::predictTo(uint32_t targetMs, const NavigationInput& in)
{
    const int32_t total = timebase::diffMs(targetMs, tf_);
    if (total <= 0) return true;
    if (static_cast<uint32_t>(total) > cfg_.maxPredictionMs) {
        // Too long to predict meaningfully (and unbounded work otherwise): give up the estimate.
        state_ = NavState::Unavailable;
        satInc(est_.unavailableEpisodes);
        ukf_.clear();
        restartInit();
        recovering_ = false;
        return false;
    }
    uint32_t remaining = static_cast<uint32_t>(total);
    while (remaining > 0u) {
        const uint32_t step = (remaining < cfg_.maxPredictStepMs) ? remaining : cfg_.maxPredictStepMs;
        const nav::UkfStatus s = ukf_.predict(model_, static_cast<float>(step) * 1e-3f, in.model);
        if (s != nav::UkfStatus::Ok) {
            numericalFault(s, in.nowMs);
            return false;
        }
        tf_ += step;
        remaining -= step;
        satInc(est_.predictions);
    }
    return true;
}

float VerticalNavigationEstimator::lateVariance(Source s, uint32_t lateMs, const ProcessModelInput& u) const
{
    const float dt = static_cast<float>(lateMs) * 1e-3f;
    VerticalCov Q{};
    model_.processNoise(dt, ukf_.state(), u, Q);
    const VerticalStateVec& x = ukf_.state();
    const VerticalCov& P = ukf_.covariance();
    switch (s) {
        case Source::Baro: {
            const float rate = std::fabs(x[vstate::kVelocity]) + 2.0f * std::sqrt(P.a[1][1]);
            return Q.a[0][0] + (rate * dt) * (rate * dt);
        }
        case Source::Gnss: {
            const float rate = std::fabs(x[vstate::kAcceleration]) + 2.0f * std::sqrt(P.a[2][2]);
            return Q.a[1][1] + (rate * dt) * (rate * dt);
        }
        default:
            return Q.a[2][2];
    }
}

void VerticalNavigationEstimator::processCandidate(const NavigationInput& in, const Candidate& c)
{
    MeasurementDiagnostics& d = diag(c.src);
    SourceTrack& tr = track(c.src);

    const int32_t dt = timebase::diffMs(c.ts, tf_);
    if (dt > 0 && !predictTo(c.ts, in)) return;
    const uint32_t lateMs = (dt < 0) ? static_cast<uint32_t>(-dt) : 0u;
    const uint32_t maxLate = (c.src == Source::Gnss) ? cfg_.gnssMaxLateMs : cfg_.maxLateMs;
    if (lateMs > maxLate) {
        d.last = MeasurementOutcome::TooLate;
        satInc(d.tooLate);
        return;
    }
    const uint32_t gapSincePrevious = tr.haveCandidate ? timebase::ageMs(tr.lastCandidateTs, c.ts) : UINT32_MAX;
    tr.lastCandidateTs = c.ts;
    tr.haveCandidate = true;

    float R = c.sigma * c.sigma;
    bool inflated = false;
    if (lateMs > 0u) {
        R += lateVariance(c.src, lateMs, in.model);
        inflated = true;
        est_.qualityFlags |= navflag::LateMeasurement;
    }
    d.measurementSigma = std::sqrt(R);

    const unsigned idx = (c.src == Source::Baro) ? vstate::kAltitude
                         : (c.src == Source::Gnss) ? vstate::kVelocity
                                                   : vstate::kAcceleration;
    const auto h = [idx](const VerticalStateVec& x) {
        nav::Vec<1> z{};
        z[0] = x[idx];
        return z;
    };
    nav::Vec<1> z{};
    z[0] = c.z;
    nav::Mat<1, 1> Rm{};
    Rm.a[0][0] = R;
    nav::Innovation<vstate::kDim, 1> inn{};
    nav::UkfStatus s = ukf_.evaluate(z, Rm, h, inn);
    if (s != nav::UkfStatus::Ok) {
        d.last = MeasurementOutcome::NumericalFault;
        numericalFault(s, in.nowMs);
        return;
    }
    d.innovation = inn.residual[0];
    d.innovationSigma = std::sqrt(inn.S.a[0][0]);
    d.nis = inn.nis;

    const float gate = (c.src == Source::Baro) ? cfg_.baroGateSigma
                       : (c.src == Source::Gnss) ? cfg_.gnssGateSigma
                                                 : cfg_.accelGateSigma;
    if (inn.nis > gate * gate) {
        if (tr.rejectRun == 0u) tr.rejectRunStart = c.ts;
        satInc(tr.rejectRun);
        bool recover = false;
        if (c.src == Source::Baro) {
            const uint32_t sinceAccepted = since(tr.everAccepted, tr.lastAcceptedAt, c.ts);
            const bool afterOutage = gapSincePrevious >= cfg_.baroOutageReanchorMs &&
                                     sinceAccepted >= cfg_.baroOutageReanchorMs;
            const bool persistent = tr.rejectRun >= cfg_.baroRejectRecoveryCount &&
                                    timebase::ageMs(tr.rejectRunStart, c.ts) >= cfg_.baroRejectRecoveryMs;
            recover = afterOutage || persistent;
        } else if (c.src == Source::Accel) {
            recover = tr.rejectRun >= cfg_.accelRejectRecoveryCount;
        }
        if (!recover) {
            d.last = MeasurementOutcome::RejectedGate;
            satInc(d.rejectedGate);
            est_.qualityFlags |= (c.src == Source::Baro)    ? navflag::BaroRejected
                                 : (c.src == Source::Accel) ? navflag::AccelRejected
                                                            : navflag::GnssRejected;
            return;
        }
        // Explicit, counted recovery: the state is persistently inconsistent with
        // this source. Inflate that state's variance by nu^2 so the sample is
        // accepted and the state moves to it smoothly through the filter.
        const float nu2 = inn.residual[0] * inn.residual[0];
        ukf_.inflateVariance(idx, nu2);
        if (c.src == Source::Baro) {
            // An altitude error of nu that built up over T seconds implies a velocity
            // error of about nu / T: re-open the velocity too, otherwise the old
            // (wrong) velocity immediately rebuilds the same disagreement.
            const uint32_t sinceOk = since(tr.everAccepted, tr.lastAcceptedAt, c.ts);
            const float T = (sinceOk == UINT32_MAX) ? 1.0f : std::fmax(1.0f, static_cast<float>(sinceOk) * 1e-3f);
            ukf_.inflateVariance(vstate::kVelocity, nu2 / (T * T));
        }
        s = ukf_.evaluate(z, Rm, h, inn);
        if (s != nav::UkfStatus::Ok) {
            d.last = MeasurementOutcome::NumericalFault;
            numericalFault(s, in.nowMs);
            return;
        }
        satInc(d.recoveries);
        inflated = true;
        if (c.src == Source::Baro) {
            recovering_ = true;
            recoveryConfirm_ = 0u;
        }
    }

    const uint32_t baroSilence = since(tr.everAccepted, tr.lastAcceptedAt, c.ts);
    s = ukf_.apply(inn);
    if (s != nav::UkfStatus::Ok) {
        d.last = MeasurementOutcome::NumericalFault;
        numericalFault(s, in.nowMs);
        return;
    }
    d.last = inflated ? MeasurementOutcome::AcceptedInflated : MeasurementOutcome::Accepted;
    satInc(d.accepted);
    if (inflated) satInc(d.inflated);
    tr.rejectRun = 0u;
    tr.lastAcceptedAt = c.ts;
    tr.everAccepted = true;
    haveAnyUpdate_ = true;
    lastAnyUpdateAt_ = tf_;
    assimilated_ = true;

    if (c.src == Source::Baro) {
        if (baroSilence != UINT32_MAX && baroSilence > cfg_.baroDegradedMs && !recovering_) {
            recovering_ = true;          // barometer back after an outage: confirm before "Tracking"
            recoveryConfirm_ = 0u;
        }
        if (recovering_) {
            ++recoveryConfirm_;
            if (recoveryConfirm_ >= cfg_.recoveryConfirmCount) recovering_ = false;
        }
    }
}

void VerticalNavigationEstimator::reanchorAltitude()
{
    ukf_.inflateVariance(vstate::kAltitude, kReanchorVariance_m2);
    satInc(est_.referenceResets);
    referenceChangedPending_ = false;
}

void VerticalNavigationEstimator::numericalFault(nav::UkfStatus s, uint32_t nowMs)
{
    state_ = NavState::NumericalFault;
    satInc(est_.numericalFaults);
    est_.lastFaultCode = static_cast<uint32_t>(s);
    est_.lastFaultAtMs = nowMs;
    est_.qualityFlags |= navflag::NumericalFault;
    ukf_.clear();
    restartInit();
    recovering_ = false;
}

// ---------------------------------------------------------------------------
// state machine and output
// ---------------------------------------------------------------------------

void VerticalNavigationEstimator::updateState(uint32_t nowMs)
{
    if (!tracking()) return;
    const uint32_t sinceAny = since(haveAnyUpdate_, lastAnyUpdateAt_, nowMs);
    const SourceTrack& b = track(Source::Baro);
    const SourceTrack& a = track(Source::Accel);
    const uint32_t sinceBaro = since(b.everAccepted, b.lastAcceptedAt, nowMs);
    const uint32_t sinceAccel = since(a.everAccepted, a.lastAcceptedAt, nowMs);
    const float sigH = std::sqrt(ukf_.covariance().a[0][0]);

    if (sinceAny > cfg_.maxPredictionMs || sinceBaro > cfg_.maxBaroOutageMs ||
        !(sigH <= cfg_.unavailableAltitudeSigma_m)) {
        state_ = NavState::Unavailable;
        satInc(est_.unavailableEpisodes);
        ukf_.clear();
        restartInit();
        recovering_ = false;
        return;
    }
    if (sinceBaro > cfg_.baroDegradedMs) est_.qualityFlags |= navflag::BaroUnavailable;
    if (sinceAccel > cfg_.accelDegradedMs) est_.qualityFlags |= navflag::AccelUnavailable;

    if (sinceBaro > cfg_.baroDegradedMs || sinceAccel > cfg_.accelDegradedMs) {
        state_ = NavState::Degraded;
    } else if (recovering_) {
        state_ = NavState::Recovering;
    } else {
        state_ = NavState::Tracking;
    }
}

void VerticalNavigationEstimator::finalize(const NavigationInput& in)
{
    const uint32_t now = in.nowMs;
    est_.state = state_;
    est_.estimator = NavEstimatorId::VerticalUkf;
    est_.processModel = model_.info();
    est_.timestampMs = tf_;
    est_.predictionAgeMs = timebase::ageMs(tf_, now);

    const SourceTrack& b = track(Source::Baro);
    const SourceTrack& a = track(Source::Accel);
    const SourceTrack& g = track(Source::Gnss);
    est_.baro.sinceAcceptedMs = since(b.everAccepted, b.lastAcceptedAt, now);
    est_.accel.sinceAcceptedMs = since(a.everAccepted, a.lastAcceptedAt, now);
    est_.gnss.sinceAcceptedMs = since(g.everAccepted, g.lastAcceptedAt, now);
    est_.outageMs = est_.baro.sinceAcceptedMs;
    est_.sinceAnyUpdateMs = since(haveAnyUpdate_, lastAnyUpdateAt_, now);
    if (est_.gnss.sinceAcceptedMs > cfg_.gnssUnusedMs) est_.qualityFlags |= navflag::GnssUnused;
    if (est_.processModel.id == ProcessModelId::ConstantAcceleration) est_.qualityFlags |= navflag::ModelPlaceholder;
    if (recovering_) est_.qualityFlags |= navflag::Recovering;

    if (tracking() && ukf_.initialized()) {
        const VerticalStateVec& x = ukf_.state();
        const VerticalCov& P = ukf_.covariance();
        est_.altitude_m = x[vstate::kAltitude];
        est_.verticalVelocity_mps = x[vstate::kVelocity];
        est_.verticalAccel_mps2 = x[vstate::kAcceleration];
        for (unsigned r = 0; r < vstate::kDim; ++r)
            for (unsigned cc = 0; cc < vstate::kDim; ++cc) est_.covariance[r][cc] = P.a[r][cc];
        est_.altitudeSigma_m = std::sqrt(P.a[0][0]);
        est_.velocitySigma_mps = std::sqrt(P.a[1][1]);
        est_.accelSigma_mps2 = std::sqrt(P.a[2][2]);
        est_.altitudeValid = est_.altitudeSigma_m <= cfg_.maxAltitudeSigma_m;
        est_.velocityValid = est_.velocitySigma_mps <= cfg_.maxVelocitySigma_mps;
        est_.accelerationValid = est_.accelSigma_mps2 <= cfg_.maxAccelSigma_mps2 &&
                                 est_.accel.sinceAcceptedMs <= cfg_.accelDegradedMs;
        if (!est_.altitudeValid || !est_.velocityValid || !est_.accelerationValid) {
            est_.qualityFlags |= navflag::UncertaintyLarge;
        }
    } else {
        est_.altitude_m = est_.verticalVelocity_mps = est_.verticalAccel_mps2 = kNaN;
        est_.altitudeSigma_m = est_.velocitySigma_mps = est_.accelSigma_mps2 = kNaN;
        for (auto& row : est_.covariance)
            for (float& v : row) v = kNaN;
        est_.altitudeValid = est_.velocityValid = est_.accelerationValid = false;
        est_.qualityFlags |= navflag::NotInitialized;
    }
}

}  // namespace pg
