/**
 ******************************************************************************
 * @file    NavigationConfig.hpp
 * @brief   Every tunable number of the vertical navigation estimator.
 *
 * Values are CONSERVATIVE STARTING POINTS chosen from data-sheet figures and
 * the host simulations (VERTICAL_UKF_VALIDATION_REPORT.md section 8). None is
 * tuned on real flight data: there is no hardware yet.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_CONFIG_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_CONFIG_HPP

#include <cmath>
#include <cstdint>

#include "Algorithms/Navigation/UnscentedKalmanFilter.hpp"

namespace pg {

struct NavigationConfig {
    // ---- UKF (NOT the Madgwick beta) ----
    /// alpha = 1, beta = 2, kappa = 0: sigma points at +-sqrt(3) sigma, all
    /// weights >= 0 except Wm0 = 0. Small alpha (1e-3, common in textbooks)
    /// makes Wm0 ~ -1e6 and loses all precision in float; for the linear
    /// placeholder model alpha has no effect on the exact result anyway.
    nav::UkfParams ukf{1.0f, 2.0f, 0.0f};

    /// White-jerk PSD of the constant-acceleration model (m^2/s^5). Over 1 s
    /// without measurements, az uncertainty grows by sqrt(qj) = 1.7 m/s^2.
    /// Sweep (0.1..10): 3 lowers acceleration error and NEES vs 1 with no
    /// change in altitude / velocity error; larger values gain little.
    float jerkPsd_m2ps5 = 3.0f;
    /// Unmodelled acceleration acting on h / vz (m^2/s^3): accelerometer bias,
    /// attitude error, model error. Keeps vz correctable by the barometer
    /// (see ConstantAccelerationModel.hpp). 0 disables it.
    float accelErrorPsd_m2ps3 = 0.02f;

    // ---- barometer ----
    /// 1-sigma of the voted relative altitude (m). BMP585 noise is cm-level;
    /// this mostly covers airflow / body pressure effects on a worn device,
    /// which are CORRELATED in time (not white). Sweep (0.25..1.0): 0.7 gives
    /// lower velocity error and a less over-confident NEES than 0.5 at almost
    /// no altitude cost; 0.25 makes the filter reject good samples.
    float baroSigma_m = 0.7f;
    float baroSingleSensorFactor = 1.2f;   ///< only one barometer contributes (Degraded vote)
    float baroSourceChangeFactor = 2.0f;   ///< first sample after the contributing set changed
    float baroGateSigma = 4.0f;            ///< reject |nu| > gate * sqrt(S)
    uint32_t baroMaxAgeMs = 100u;          ///< older (at processing time) = Stale
    /// Persistent disagreement: after this long with every sample gated out,
    /// the altitude variance is inflated by nu^2 and the sample accepted
    /// (Recovering). Trade-off: a slow barometer fault is followed after this delay.
    uint32_t baroRejectRecoveryMs = 2000u;
    uint32_t baroRejectRecoveryCount = 20u;  ///< ... and at least this many consecutive rejections
    /// After an outage this long, a returning barometer that fails the gate is
    /// re-anchored immediately (no 2 s wait).
    uint32_t baroOutageReanchorMs = 1000u;

    // ---- vertical acceleration (from the attitude estimator) ----
    /// 1-sigma (m/s^2). Covers accelerometer noise, ~1 deg tilt error and part
    /// of the uncalibrated accelerometer bias (LSM6DSRX zero-g offset up to
    /// +-20 mg = 0.2 m/s^2). A constant bias is NOT white noise: see the report.
    float accelSigma_mps2 = 0.35f;
    float accelUnreliableFactor = 3.0f;      ///< attitude Degraded / not reliable
    float accelTurnGateFactor = 1.5f;        ///< Madgwick skipped the accel correction (turn gate / rejection)
    float accelProlongedRejectFactor = 3.0f; ///< Madgwick has had no accel correction for long
    float accelImuDegradedFactor = 1.5f;     ///< IMU vote Degraded
    float accelMaxInflation = 10.0f;         ///< total factor above this -> RejectedQuality
    float accelGateSigma = 6.0f;
    uint32_t accelRejectRecoveryCount = 5u;  ///< consecutive gate rejections before az variance is inflated
    uint32_t accelMaxAgeMs = 50u;

    // ---- GNSS vertical velocity (optional) ----
    bool useGnssVelocity = true;
    uint8_t gnssMinSatellites = 6u;
    float gnssMaxSpeedAccuracy_mps = 1.5f;   ///< sAcc above this: not used
    float gnssSpeedAccuracyScale = 1.5f;     ///< vertical velocity is less accurate than sAcc suggests
    float gnssMinSigma_mps = 0.15f;
    float gnssGateSigma = 4.0f;
    uint32_t gnssMaxAgeMs = 400u;            ///< parse time vs processing time
    uint32_t gnssMaxLateMs = 300u;           ///< may be applied this far behind the filter time (inflated)

    // ---- timing ----
    uint32_t maxLateMs = 30u;                ///< baro / accel samples up to this late are applied (inflated)
    uint32_t maxPredictStepMs = 100u;        ///< long predictions are split into steps of at most this
    uint32_t idlePredictMs = 50u;            ///< no new data for this long: predict to "now" (covariance grows)
    uint32_t maxPredictionMs = 3000u;        ///< no measurement at all for this long -> Unavailable
    uint32_t baroDegradedMs = 500u;          ///< no barometer accepted for this long -> Degraded
    uint32_t accelDegradedMs = 500u;
    uint32_t maxBaroOutageMs = 20000u;       ///< no barometer for this long -> Unavailable (needs re-init)
    uint32_t gnssUnusedMs = 3000u;

    // ---- initialisation ----
    uint32_t initWindowMs = 500u;            ///< barometer window for the initial h / vz fit
    uint32_t initMinSamples = 15u;
    uint32_t initMaxGapMs = 200u;            ///< a gap this long restarts the window
    float initMaxResidual_m = 1.5f;          ///< RMS fit residual above this restarts the window
    float initMinAltitudeSigma_m = 0.3f;
    float initMinVelocitySigma_mps = 0.3f;
    float initAccelSigma_mps2 = 2.0f;        ///< when no acceleration is available at initialisation
    uint32_t recoveryConfirmCount = 10u;     ///< accepted barometer samples to leave Recovering

    // ---- output validity ----
    float maxAltitudeSigma_m = 15.0f;        ///< above: altitude invalid
    float maxVelocitySigma_mps = 2.0f;       ///< above: velocity invalid
    float maxAccelSigma_mps2 = 2.0f;
    float unavailableAltitudeSigma_m = 50.0f;///< above: state Unavailable

    bool isValid() const
    {
        const bool positive = baroSigma_m > 0.0f && accelSigma_mps2 > 0.0f && jerkPsd_m2ps5 > 0.0f &&
                              baroGateSigma > 0.0f && accelGateSigma > 0.0f && gnssGateSigma > 0.0f &&
                              gnssMinSigma_mps > 0.0f && initMinAltitudeSigma_m > 0.0f &&
                              initMinVelocitySigma_mps > 0.0f && initAccelSigma_mps2 > 0.0f &&
                              maxAltitudeSigma_m > 0.0f && maxVelocitySigma_mps > 0.0f && maxAccelSigma_mps2 > 0.0f;
        const bool factors = baroSingleSensorFactor >= 1.0f && baroSourceChangeFactor >= 1.0f &&
                             accelUnreliableFactor >= 1.0f && accelTurnGateFactor >= 1.0f &&
                             accelProlongedRejectFactor >= 1.0f && accelImuDegradedFactor >= 1.0f &&
                             accelMaxInflation >= 1.0f && gnssSpeedAccuracyScale >= 1.0f;
        const bool timing = maxPredictStepMs > 0u && initWindowMs > 0u && initMinSamples >= 3u &&
                            initMinSamples <= kInitCapacity && maxPredictionMs > idlePredictMs &&
                            unavailableAltitudeSigma_m >= maxAltitudeSigma_m;
        const bool finite = std::isfinite(jerkPsd_m2ps5) && std::isfinite(accelErrorPsd_m2ps3) &&
                            accelErrorPsd_m2ps3 >= 0.0f && std::isfinite(baroSigma_m) &&
                            std::isfinite(accelSigma_mps2) && std::isfinite(ukf.alpha) && std::isfinite(ukf.beta) &&
                            std::isfinite(ukf.kappa);
        return positive && factors && timing && finite;
    }

    /// Capacity of the initialisation window buffer (samples).
    static constexpr uint32_t kInitCapacity = 64u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_CONFIG_HPP
