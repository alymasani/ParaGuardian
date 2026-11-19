/**
 ******************************************************************************
 * @file    AttitudeConfig.hpp
 * @brief   Every tunable number of the attitude estimator, with its basis.
 *
 * NONE OF THESE VALUES IS FLIGHT-TUNED. They are documented starting points
 * chosen from the sensor data sheets, the preprocessing configuration and the
 * simulation sweeps in Docs/MADGWICK_VALIDATION_REPORT.md. They must be
 * re-evaluated with recorded flight data.
 *
 * Units: rad/s, m/s^2, uT, ms, degrees only where the name says "Deg".
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_CONFIG_HPP
#define PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_CONFIG_HPP

#include <cmath>
#include <cstdint>

#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

/// How the magnetometer may be used.
enum class MagPolicy : uint8_t {
    Disabled = 0,        ///< never used: pure 6-axis, heading always unreferenced
    CalibratedOnly,      ///< used only when the frame says hard/soft-iron calibrated (DEFAULT)
    AllowUncalibrated,   ///< EXPERIMENTAL: uses uncalibrated data; heading marked "unvalidated"
};

struct AttitudeConfig {
    // ---- Madgwick gain ----------------------------------------------------------------
    /**
     * beta (rad/s): the magnitude of the gradient-descent correction rate.
     * Madgwick (2010) derives beta = sqrt(3/4) * w_err, where w_err is the
     * gyroscope error the correction must cancel. NOT the same quantity as the
     * "gain" of x-io's Fusion library (a dimensionless feedback gain of a
     * different update law). 0.1 is the default of Madgwick's reference code;
     * the sweep in MADGWICK_VALIDATION_REPORT.md section 6 shows the
     * noise / drift / acceleration trade-off. NOT validated for paragliding.
     */
    float beta_radps = 0.1f;
    /// Larger gain used while RECOVERING after a gap or prolonged rejection.
    float recoveryBeta_radps = 0.5f;
    /// Specific force of gravity used to judge |f| and to remove gravity (standard gravity).
    float gravity_mps2 = units::kStandardGravity_mps2;
    /// Gyro measurements must be bias-calibrated (preprocessing) before tracking starts.
    bool requireCalibratedGyro = true;

    // ---- timing --------------------------------------------------------------------
    /// dt below this is not integrated (1 ms = timestamp resolution; 0 ms = same sample).
    uint32_t minDtMs = 1u;
    /// Longest interval integrated with one gyro sample (= PreprocessingPipeline::kMaxGyroDtMs).
    uint32_t maxIntegrationDtMs = 50u;
    /// Longer gyro outage: the orientation is unknown -> Invalid, re-initialise.
    uint32_t maxGapMs = 500u;
    /// Freshness limits at the moment of use (preprocessing already rejects stale samples).
    uint32_t gyroMaxAgeMs = 30u;
    uint32_t accelMaxAgeMs = 30u;
    uint32_t magMaxAgeMs = 100u;

    // ---- initialisation (quasi-static levelling) ------------------------------------
    /// Continuous quasi-static time needed to level the filter from gravity.
    uint32_t initWindowMs = 500u;
    /// Minimum gyro samples in that window (~40 of 50 at 100 Hz: tolerates dropouts).
    uint32_t initMinSamples = 40u;
    /// |w| limit during levelling (same as the preprocessing stationary detector).
    float initMaxRate_radps = 0.10f;
    /// | |f| - g | limit during levelling (same as the preprocessing stationary detector).
    float initMaxAccelError_mps2 = 0.6f;

    // ---- accelerometer correction gating ----------------------------------------------
    /**
     * Correction is rejected when the angle between the measured specific-force
     * direction and the direction the current estimate predicts for "up" in
     * body axes exceeds this. It is an ANGLE test (direction), not a magnitude
     * test: during a coordinated turn |f| can be close to g while its direction
     * is tilted by the bank angle.
     */
    float accelRejectionDeg = 10.0f;
    /// Correction is also rejected when | |f| - g | exceeds this (free fall, strong manoeuvres).
    float accelMagnitudeTolerance_mps2 = 1.0f;
    /**
     * TURN GATE: correction is also rejected while |w| exceeds this. Needed
     * because in a COORDINATED turn the specific force always lies along body
     * -z, so its direction never disagrees much with a slowly drifting
     * estimate and the angle gate alone lets the filter be pulled to "wings
     * level" (simulation: full 15 deg error in a sustained 15 deg turn). A
     * turn at bank phi and airspeed V rotates at g tan(phi) / V
     * (0.26 rad/s at 15 deg, 10 m/s). Value from the sweep in
     * MADGWICK_VALIDATION_REPORT.md; turns slower than this still leak.
     * Large value (e.g. 1000) disables the gate.
     */
    float accelRejectionRate_radps = 0.10f;
    /// No accepted correction for this long -> state Degraded (attitude is gyro-only).
    uint32_t degradedAfterMs = 2000u;
    /**
     * Recovery: after rejections lasting this long (or after a gap), corrections
     * are FORCED (with recoveryBeta) - but only while the motion has been
     * quasi-static for recoveryQuietMs (|w| and |f| near rest values). Time
     * based, not sample counts, because the sample interval varies. 0 disables
     * forced recovery entirely.
     */
    uint32_t accelRecoveryMs = 5000u;
    uint32_t recoveryQuietMs = 1000u;
    float recoveryMaxRate_radps = 0.10f;
    float recoveryMaxAccelError_mps2 = 0.3f;

    // ---- magnetometer -------------------------------------------------------------------
    MagPolicy magPolicy = MagPolicy::CalibratedOnly;
    /// Plausible geomagnetic field strength (Earth: ~25..65 uT). Outside -> disturbed / faulty.
    float magMinField_uT = 20.0f;
    float magMaxField_uT = 70.0f;
    /// Fractional change of |m| from the reference acquired at heading acquisition.
    float magFieldTolerance = 0.15f;
    /// Change of the field's inclination (dip) from the reference.
    float magInclinationToleranceDeg = 10.0f;
    /// Heading disagreement above which the magnetometer correction is rejected.
    float magRejectionDeg = 10.0f;
    /// Persisting, self-consistent heading disagreement accepted after this long (0 = never).
    uint32_t magRecoveryMs = 5000u;
    /// Consecutive consistent magnetometer samples needed to acquire the heading reference.
    uint32_t magAcquireSamples = 10u;
    /// Heading becomes "lost" when no magnetometer correction was accepted for this long.
    uint32_t headingHoldMs = 10000u;
};

/// True if every parameter is finite and in a meaningful range.
inline bool isValid(const AttitudeConfig& c)
{
    auto pos = [](float v) { return std::isfinite(v) && v > 0.0f; };
    return pos(c.beta_radps) && pos(c.recoveryBeta_radps) && pos(c.gravity_mps2) && c.minDtMs >= 1u &&
           c.maxIntegrationDtMs >= c.minDtMs && c.maxGapMs >= c.maxIntegrationDtMs && c.initWindowMs > 0u &&
           c.initMinSamples > 0u && pos(c.initMaxRate_radps) && pos(c.initMaxAccelError_mps2) &&
           pos(c.accelRejectionDeg) && c.accelRejectionDeg <= 180.0f && pos(c.accelMagnitudeTolerance_mps2) &&
           pos(c.accelRejectionRate_radps) &&
           pos(c.recoveryMaxRate_radps) && pos(c.recoveryMaxAccelError_mps2) && pos(c.magMinField_uT) &&
           c.magMaxField_uT > c.magMinField_uT && pos(c.magFieldTolerance) && pos(c.magInclinationToleranceDeg) &&
           pos(c.magRejectionDeg) && c.magAcquireSamples > 0u && c.gyroMaxAgeMs > 0u && c.accelMaxAgeMs > 0u &&
           c.magMaxAgeMs > 0u;
}

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_CONFIG_HPP
