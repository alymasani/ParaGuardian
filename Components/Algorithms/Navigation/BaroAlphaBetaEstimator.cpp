/**
 ******************************************************************************
 * @file    BaroAlphaBetaEstimator.cpp
 * @brief   See BaroAlphaBetaEstimator.hpp.
 ******************************************************************************
 */
#include "Algorithms/Navigation/BaroAlphaBetaEstimator.hpp"

#include <cmath>
#include <limits>

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
void satInc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}
}  // namespace

BaroAlphaBetaEstimator::BaroAlphaBetaEstimator(const BaroAlphaBetaConfig& cfg) : cfg_(cfg) { reset(); }

void BaroAlphaBetaEstimator::reset()
{
    tracker_.reset();
    running_ = false;
    h_ = v_ = 0.0f;
    t_ = 0u;
    samples_ = 0u;
    rejectRun_ = 0u;
    lastAcceptedAt_ = 0u;
    haveAccepted_ = false;
    est_ = NavigationEstimate{};
    est_.estimator = NavEstimatorId::BaroAlphaBeta;
    est_.processModel = ProcessModelInfo{ProcessModelId::None, 0u};
    est_.state = NavState::Uninitialized;
    est_.altitude_m = est_.verticalVelocity_mps = est_.verticalAccel_mps2 = kNaN;
    est_.altitudeSigma_m = est_.velocitySigma_mps = est_.accelSigma_mps2 = kNaN;
    for (auto& row : est_.covariance)
        for (float& x : row) x = kNaN;
    est_.qualityFlags = navflag::NotInitialized;
}

void BaroAlphaBetaEstimator::gains(float sigmaA, float sigmaZ, float dt_s, float& alpha, float& beta)
{
    const float lambda = sigmaA * dt_s * dt_s / sigmaZ;
    const float r = (4.0f + lambda - std::sqrt(8.0f * lambda + lambda * lambda)) / 4.0f;
    alpha = 1.0f - r * r;
    beta = 2.0f * (2.0f - alpha) - 4.0f * std::sqrt(1.0f - alpha);
}

void BaroAlphaBetaEstimator::start(float z, uint32_t ts)
{
    h_ = z;
    v_ = 0.0f;   // unknown: velocity is reported INVALID until settleSamples
    t_ = ts;
    samples_ = 1u;
    rejectRun_ = 0u;
    running_ = true;
    satInc(est_.initializations);
    if (est_.initializations > 1u) satInc(est_.reinitializations);
}

const NavigationEstimate& BaroAlphaBetaEstimator::update(const NavigationInput& in)
{
    satInc(est_.sequence);
    est_.computedAtMs = in.nowMs;
    est_.qualityFlags = 0u;
    est_.baro.last = MeasurementOutcome::NoData;
    float dt = lastDt_;

    if (cfg_.isValid() && in.baroValid) {
        const BaroSampleTracker::Verdict v = tracker_.classify(in);
        if (v == BaroSampleTracker::Verdict::New && timebase::ageMs(in.baroTimestampMs, in.nowMs) <= cfg_.maxAgeMs) {
            tracker_.markUsed(in);
            const uint32_t ts = BaroSampleTracker::observationTime(in);
            const float z = in.baroAltitude_m;
            const int32_t dtMs = running_ ? timebase::diffMs(ts, t_) : 0;
            if (!running_ || dtMs > static_cast<int32_t>(cfg_.maxGapMs)) {
                start(z, ts);
                est_.baro.last = MeasurementOutcome::UsedForInit;
            } else if (dtMs > 0) {
                dt = static_cast<float>(dtMs) * 1e-3f;
                float a = 0.0f;
                float b = 0.0f;
                gains(cfg_.maneuverSigma_mps2, cfg_.baroSigma_m, dt, a, b);
                const float hp = h_ + v_ * dt;
                const float r = z - hp;
                est_.baro.innovation = r;
                t_ = ts;
                if (std::fabs(r) > cfg_.gate_m && ++rejectRun_ < cfg_.rejectResetCount) {
                    h_ = hp;   // prediction only
                    est_.baro.last = MeasurementOutcome::RejectedGate;
                    satInc(est_.baro.rejectedGate);
                    est_.qualityFlags |= navflag::BaroRejected;
                } else {
                    if (rejectRun_ >= cfg_.rejectResetCount) {
                        h_ = z;   // persistent disagreement: re-anchor (counted)
                        satInc(est_.baro.recoveries);
                        est_.baro.last = MeasurementOutcome::AcceptedInflated;
                    } else {
                        h_ = hp + a * r;
                        v_ = v_ + (b / dt) * r;
                        est_.baro.last = MeasurementOutcome::Accepted;
                    }
                    rejectRun_ = 0u;
                    satInc(est_.baro.accepted);
                    satInc(samples_);
                    lastAlpha_ = a;
                    lastBeta_ = b;
                    lastDt_ = dt;
                    lastAcceptedAt_ = ts;
                    haveAccepted_ = true;
                }
            }
        } else if (v == BaroSampleTracker::Verdict::NotNew) {
            est_.baro.last = MeasurementOutcome::NotNew;
        } else if (v == BaroSampleTracker::Verdict::OutOfOrder) {
            est_.baro.last = MeasurementOutcome::OutOfOrder;
            satInc(est_.baro.outOfOrder);
        }
    }

    if (running_ && timebase::diffMs(in.nowMs, t_) > static_cast<int32_t>(cfg_.maxGapMs)) {
        running_ = false;
        satInc(est_.unavailableEpisodes);
    }
    output(in.nowMs, dt);
    return est_;
}

void BaroAlphaBetaEstimator::output(uint32_t nowMs, float /*dt_s*/)
{
    est_.timestampMs = t_;
    est_.predictionAgeMs = timebase::ageMs(t_, nowMs);
    est_.baro.sinceAcceptedMs = haveAccepted_ ? timebase::ageMs(lastAcceptedAt_, nowMs) : UINT32_MAX;
    est_.outageMs = est_.baro.sinceAcceptedMs;
    est_.verticalAccel_mps2 = kNaN;
    est_.accelSigma_mps2 = kNaN;
    est_.accelerationValid = false;
    if (!running_) {
        est_.state = (est_.initializations == 0u) ? NavState::Uninitialized : NavState::Unavailable;
        est_.altitude_m = est_.verticalVelocity_mps = kNaN;
        est_.altitudeSigma_m = est_.velocitySigma_mps = kNaN;
        est_.altitudeValid = est_.velocityValid = false;
        est_.qualityFlags |= navflag::NotInitialized;
        return;
    }
    const bool settled = samples_ >= cfg_.settleSamples;
    est_.state = settled ? NavState::Tracking : NavState::Initializing;
    est_.altitude_m = h_;
    est_.verticalVelocity_mps = v_;
    const float sz2 = cfg_.baroSigma_m * cfg_.baroSigma_m;
    const float a = lastAlpha_;
    const float b = lastBeta_;
    const float dt = lastDt_;
    if (settled && a > 0.0f && a < 1.0f) {
        const float phh = a * sz2;
        const float pvv = b * (a - 0.5f * b) / (1.0f - a) * sz2 / (dt * dt);
        const float phv = b * sz2 / dt;
        est_.altitudeSigma_m = std::sqrt(phh);
        est_.velocitySigma_mps = std::sqrt(pvv);
        est_.covariance[0][0] = phh;
        est_.covariance[0][1] = est_.covariance[1][0] = phv;
        est_.covariance[1][1] = pvv;
    } else {
        est_.altitudeSigma_m = cfg_.baroSigma_m;
        est_.velocitySigma_mps = kNaN;
    }
    est_.altitudeValid = true;
    est_.velocityValid = settled;
}

}  // namespace pg
