/**
 ******************************************************************************
 * @file    BaroAlphaBetaEstimator.hpp
 * @brief   Barometer-only vertical estimator (alpha-beta filter). Independent
 *          of the UKF: used for comparison, regression and fault diagnosis,
 *          and available as a possible fallback (not switched in
 *          automatically in this milestone).
 *
 * Each new barometer observation z at interval dt:
 *     predict   h- = h + v dt
 *     residual  r  = z - h-
 *     correct   h  = h- + alpha r,   v = v + (beta / dt) r
 *
 * alpha and beta are NOT fixed constants: they are the steady-state Kalman
 * gains of a constant-velocity model with piecewise-constant white
 * acceleration (Kalata 1984), recomputed for the ACTUAL dt each time:
 *     lambda = sigma_a dt^2 / sigma_z          (tracking index)
 *     r      = (4 + lambda - sqrt(8 lambda + lambda^2)) / 4
 *     alpha  = 1 - r^2,   beta = 2 (2 - alpha) - 4 sqrt(1 - alpha)
 * so the filter is timestamp-aware and as good as a 2-state Kalman filter in
 * steady state. Its reported sigmas are that steady state's:
 *     P_hh = alpha sigma_z^2,  P_vv = beta (alpha - beta/2) / (1 - alpha) sigma_z^2 / dt^2
 *
 * It does not use the accelerometer, so it LAGS during acceleration changes:
 * that is the expected weakness the UKF should fix, and the comparison in
 * the validation report measures it honestly.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_BARO_ALPHA_BETA_ESTIMATOR_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_BARO_ALPHA_BETA_ESTIMATOR_HPP

#include <cstdint>

#include "Algorithms/Navigation/BaroSampleTracker.hpp"
#include "Algorithms/Navigation/NavigationTypes.hpp"

namespace pg {

struct BaroAlphaBetaConfig {
    /// sigma_a: how hard the vertical speed may change. 3.0 = best velocity RMSE of the
    /// values swept on the tuning runs (the baseline is tuned as fairly as the UKF).
    float maneuverSigma_mps2 = 3.0f;
    float baroSigma_m = 0.5f;          ///< sigma_z
    float gate_m = 5.0f;               ///< |residual| above this is rejected
    uint32_t rejectResetCount = 10u;   ///< consecutive rejections -> re-anchor at the barometer
    uint32_t settleSamples = 25u;      ///< velocity reported valid after this many samples
    uint32_t maxGapMs = 1000u;         ///< no barometer for longer -> Unavailable, re-initialise
    uint32_t maxAgeMs = 100u;

    bool isValid() const
    {
        return maneuverSigma_mps2 > 0.0f && baroSigma_m > 0.0f && gate_m > 0.0f && maxGapMs > 0u;
    }
};

class BaroAlphaBetaEstimator {
public:
    explicit BaroAlphaBetaEstimator(const BaroAlphaBetaConfig& cfg = BaroAlphaBetaConfig{});

    const NavigationEstimate& update(const NavigationInput& in);
    const NavigationEstimate& latest() const { return est_; }
    void reset();

    /// Kalata steady-state gains for interval @p dt_s (exposed for tests).
    static void gains(float sigmaA, float sigmaZ, float dt_s, float& alpha, float& beta);

private:
    void start(float z, uint32_t ts);
    void output(uint32_t nowMs, float dt_s);

    BaroAlphaBetaConfig cfg_;
    BaroSampleTracker tracker_;
    bool running_ = false;
    float h_ = 0.0f;
    float v_ = 0.0f;
    uint32_t t_ = 0u;
    uint32_t samples_ = 0u;
    uint32_t rejectRun_ = 0u;
    float lastAlpha_ = 0.0f;
    float lastBeta_ = 0.0f;
    float lastDt_ = 0.02f;
    uint32_t lastAcceptedAt_ = 0u;
    bool haveAccepted_ = false;
    NavigationEstimate est_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_BARO_ALPHA_BETA_ESTIMATOR_HPP
