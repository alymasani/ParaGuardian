/**
 ******************************************************************************
 * @file    Calibration.hpp
 * @brief   Gyroscope bias calibration, barometer reference calibration,
 *          barometric relative altitude, and the (future) accelerometer
 *          calibration interface.
 *
 * GYROSCOPE BIAS (estimated now)
 *   When the device is still, the true angular rate is ~0 (Earth rotation,
 *   7.3e-5 rad/s, is far below MEMS noise and is ignored). The mean gyro
 *   reading over a confirmed-still window is therefore the bias:
 *       bias = mean(w_measured)          calibrated w = w_measured - bias
 *   Quality: per-axis sample standard deviation s and the standard error of
 *   the mean s / sqrt(n) (how uncertain the bias estimate itself is).
 *
 * ACCELEROMETER (NOT estimated now - by design)
 *   A still accelerometer measures  a_meas = -g_sensor + b  (specific force
 *   plus offset b). From ONE pose we know only |g|; the direction of g in the
 *   sensor frame is unknown (the orientation is unknown), so the 3 unknowns of
 *   b cannot be separated from the 2 unknown direction angles of g with 3
 *   equations. Subtracting the mean reading would "calibrate away" gravity
 *   and make every later reading wrong. A proper accelerometer calibration
 *   needs several known orientations (e.g. 6-face tumble) or an external
 *   reference. AccelCalibration below is the interface for that future
 *   procedure; until then the driver's factory sensitivity is used unchanged.
 *
 * BAROMETER REFERENCE
 *   P0 = mean pressure over the still window (Welford, double).
 *   Relative altitude (international standard atmosphere approximation):
 *       h_rel = 44330 m * (1 - (P / P0)^0.1903)
 *   0.1903 ~= R*L / (g*M) for the ISA troposphere lapse rate, and
 *   44330 m = T0 / L = 288.15 K / 0.0065 K/m (ISA SEA-LEVEL temperature). With a
 *   reference above sea level this over-states height differences by
 *   44330 / (44330 - h0) (3.5 % at h0 = 1500 m); BaroAltitudeModel::IsaAtReference
 *   uses the ISA temperature at P0 instead. h_rel is a
 *   height difference relative to where P0 was taken, ASSUMING the
 *   atmosphere did not change. Weather (~1 hPa over hours ~ 8 m) and
 *   temperature differences from the ISA make it drift; a pressure change is
 *   therefore NOT proof of vertical motion.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_CALIBRATION_HPP
#define PARAGUARDIAN_PROCESSING_CALIBRATION_HPP

#include <cstdint>

#include "Processing/ProcessingConfig.hpp"
#include "Processing/ProcessingTypes.hpp"
#include "Processing/RunningStats.hpp"
#include "Processing/VectorMath.hpp"

namespace pg {

// ============================================================================
// Gyroscope bias
// ============================================================================

struct GyroCalibration {
    Vec3f bias_radps;             ///< subtract from the measurement (sensor axes)
    Vec3f noiseStd_radps;         ///< per-axis sample standard deviation during calibration
    Vec3f biasStdError_radps;     ///< noiseStd / sqrt(samples): uncertainty of the bias estimate
    uint32_t samples;
    uint32_t timestampMs;         ///< MCU time the result was accepted
    uint32_t sessionId;           ///< which calibration session produced it
    bool valid;
};

class GyroBiasCalibrator {
public:
    explicit GyroBiasCalibrator(const GyroCalibrationConfig& cfg) : cfg_(cfg) {}

    /// Starts collecting a new estimate. The ACTIVE calibration stays in use meanwhile.
    void begin();
    /// Discards the samples collected so far (motion was detected); stays collecting.
    void restart() { stats_.reset(); }
    /// Adds one gyro sample in sensor axes. Ignored unless collecting.
    void add(const Vec3f& gyro_radps);
    /**
     * Ends collection and evaluates the candidate. On success it replaces the
     * active calibration; on failure the PREVIOUS active calibration (if any)
     * is kept - a bias is a property of the chip and remains a better
     * correction than none - and lastFailure() says why.
     */
    bool finish(uint32_t nowMs, uint32_t sessionId);
    /// Ends collection without evaluating (session aborted).
    void abort() { collecting_ = false; }
    void invalidate() { active_.valid = false; }

    /// calibrated = measured - bias. Returns @p measured unchanged when no valid calibration exists.
    Vec3f correct(const Vec3f& measured) const { return active_.valid ? vec::sub(measured, active_.bias_radps) : measured; }

    const GyroCalibration& active() const { return active_; }
    CalibrationState state() const;
    CalibrationFailure lastFailure() const { return lastFailure_; }
    uint32_t collectedSamples() const { return stats_.count(); }
    Vec3f currentMean() const { return stats_.mean(); }

private:
    GyroCalibrationConfig cfg_;
    RunningStats3 stats_{};
    GyroCalibration active_{};
    CalibrationFailure lastFailure_ = CalibrationFailure::None;
    bool collecting_ = false;
    bool attempted_ = false;
};

// ============================================================================
// Accelerometer (future interface; NOT estimated in this milestone)
// ============================================================================

/**
 * calibrated = scaleMisalignment * (measured - bias).
 * Default: invalid -> applyAccelCalibration() returns the measurement
 * unchanged. No constants are invented here; a future multi-orientation
 * procedure (or factory data) fills this in.
 */
struct AccelCalibration {
    Vec3f bias_mps2 = Vec3f{0.0f, 0.0f, 0.0f};
    Mat3f scaleMisalignment = mat::identity();
    bool valid = false;
};

inline Vec3f applyAccelCalibration(const AccelCalibration& c, const Vec3f& measured)
{
    return c.valid ? mat::mul(c.scaleMisalignment, vec::sub(measured, c.bias_mps2)) : measured;
}

// ============================================================================
// Barometer reference pressure
// ============================================================================

struct BaroReference {
    double referencePressure_pa;   ///< P0
    double scaleHeight_m;          ///< 44330 m, or the ISA value at P0 (BaroAltitudeModel)
    float pressureStd_pa;          ///< sample std during calibration
    float segmentSpread_pa;        ///< largest minus smallest 1 s mean (drift indicator)
    uint32_t samples;
    uint32_t timestampMs;
    uint32_t sessionId;            ///< barometers are only compared when their sessions match
    bool valid;
};

class BaroReferenceCalibrator {
public:
    explicit BaroReferenceCalibrator(const BaroCalibrationConfig& cfg) : cfg_(cfg) {}

    void begin(uint32_t startMs);
    void restart(uint32_t startMs);
    /// Adds one validated pressure sample.
    void add(uint32_t timestampMs, float pressure_pa);
    /**
     * Ends collection. On success the new reference REPLACES the old one; on
     * failure the reference becomes INVALID (it is not silently kept from an
     * older session, because the other barometer may have been re-referenced
     * and the two would then describe different zero heights).
     */
    bool finish(uint32_t nowMs, uint32_t sessionId);
    void abort() { collecting_ = false; }
    void invalidate() { active_.valid = false; }

    const BaroReference& active() const { return active_; }
    CalibrationState state() const;
    CalibrationFailure lastFailure() const { return lastFailure_; }
    uint32_t collectedSamples() const { return stats_.count(); }

private:
    BaroCalibrationConfig cfg_;
    RunningStats stats_{};
    SegmentMeans segments_{};
    BaroReference active_{};
    CalibrationFailure lastFailure_ = CalibrationFailure::None;
    bool collecting_ = false;
    bool attempted_ = false;
};

namespace baro {
constexpr double kIsaScaleHeight_m = 44330.0;
constexpr double kIsaExponent = 0.1903;
/**
 * Relative altitude (m, positive UP) of pressure @p p_pa relative to the
 * reference @p p0_pa. Returns NaN if either pressure is not > 0 or not finite
 * (callers must check std::isfinite). Double precision: (P/P0) is close to 1
 * and the result is a small difference of O(1) numbers.
 */
double relativeAltitude_m(double p_pa, double p0_pa);
/// Same, with an explicit scale height (m) instead of 44330 m.
double relativeAltitude_m(double p_pa, double p0_pa, double scaleHeight_m);
/// ISA scale height at reference pressure p0: 44330 * (p0 / 101325)^0.1903 (= T_isa(p0) / L).
double isaScaleHeight_m(double p0_pa);
constexpr double kIsaSeaLevelPressure_pa = 101325.0;
}  // namespace baro

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_CALIBRATION_HPP
