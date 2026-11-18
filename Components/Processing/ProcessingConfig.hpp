/**
 ******************************************************************************
 * @file    ProcessingConfig.hpp
 * @brief   Every tunable number of the preprocessing layer, with its basis.
 *
 * NONE OF THESE VALUES HAS BEEN TUNED ON HARDWARE. They are engineering
 * starting points derived from the acquisition timing (TaskConfig,
 * RTOS_TIMING_ANALYSIS.md), the driver configuration (Lsm6dsrxConfig defaults
 * +-16 g / +-2000 dps, BMP585 300..1250 hPa) and typical datasheet-class
 * figures for MEMS sensors that MUST be re-checked against the LSM6DSRX,
 * BMP585 and MMC5983MA datasheets and against recordings from the real
 * board. Docs/SENSOR_PREPROCESSING_DESIGN.md section 9 lists the justification
 * of each value and how to re-derive it from measured data.
 *
 * All structs are plain aggregates so a test can copy the defaults and
 * change one field.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_PROCESSING_CONFIG_HPP
#define PARAGUARDIAN_PROCESSING_PROCESSING_CONFIG_HPP

#include <cstdint>

#include "Processing/ProcessingTypes.hpp"
#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

/// Per-sample validation limits (SampleValidator).
struct ValidationConfig {
    /**
     * Maximum age (ms) of a sample, per SensorId, both when it is ingested and
     * when it is used in a frame. Basis: nominal period x ~3 (allows one
     * missed acquisition cycle plus 1 ms tick jitter and queue latency).
     *   IMU 10 ms period  -> 30 ms;  baro / mag 20 ms period -> 60 ms;
     *   GNSS (1..25 Hz, receiver-configured) -> 2000 ms (= NeoM9n stale timeout);
     *   LiDAR ~10 ms frames -> 50 ms (TF02Pro driver stale timeout is 100 ms).
     */
    uint32_t maxAgeMs[kSensorCount] = {30u, 30u, 30u, 60u, 60u, 60u, 2000u, 50u};
    /// Timestamps ahead of "now" by up to this much are tolerated (1 ms tick quantisation).
    uint32_t maxFutureMs = 2u;
    /// IMU plausibility = configured driver full scale (Lsm6dsrxConfig default: 16 g, 2000 dps).
    float accelLimit_mps2 = 16.0f * units::kStandardGravity_mps2;
    float gyroLimit_radps = 2000.0f * units::kDegToRad;
    /// Operating temperature ranges (to be confirmed per datasheet).
    float imuTempMin_c = -40.0f;
    float imuTempMax_c = 105.0f;
    float baroTempMin_c = -40.0f;
    float baroTempMax_c = 85.0f;
    /// MMC5983MA full scale is +-8 G = +-800 uT.
    float magLimit_uT = 800.0f;
    /// LiDAR distances below 0 are impossible; the driver already flags out-of-range / weak signal.
    float lidarMin_m = 0.0f;
};

/// StationaryDetector (one instance per IMU).
struct StationaryConfig {
    /// Observation window: the previously proposed 5 s start-up window.
    uint32_t windowMs = 5000u;
    /// Nominal IMU sample period, used to derive the expected sample count.
    uint32_t nominalPeriodMs = 10u;
    /// Fraction of the expected samples (window / period = 500) that must be present: 80 % -> 400.
    float minSampleFraction = 0.8f;
    /// A gap between consecutive samples longer than this restarts the window
    /// (we cannot vouch for an interval we did not observe). 5 IMU periods.
    uint32_t maxGapMs = 50u;
    /**
     * Instantaneous limits applied to EVERY sample. They must tolerate the
     * UNCALIBRATED gyro bias (MEMS zero-rate offsets are typically of the order
     * of 1 dps, worst case several dps), so they only catch clear motion:
     *   |w| <= 0.10 rad/s (5.7 dps);  | |a| - g | <= 0.6 m/s^2 (~0.06 g, covers
     *   tens of mg accel offset + local-gravity variation 9.78..9.83 m/s^2).
     */
    float gyroMagnitudeMax_radps = 0.10f;
    float accelMagnitudeTolerance_mps2 = 0.6f;
    /**
     * Window statistics (checked when the window is complete). The bias is
     * constant, so it does not raise the standard deviation; motion does.
     *   gyro per-axis std <= 0.01 rad/s (0.57 dps): roughly 10x the expected
     *   white-noise RMS of a MEMS gyro at this bandwidth; human sway exceeds it.
     *   |a| std <= 0.05 m/s^2 (~5 mg).
     */
    float gyroStdMax_radps = 0.01f;
    float accelStdMax_mps2 = 0.05f;
    /**
     * Drift check on segment means (window split into 1 s segments):
     *   gyro per axis: a warming gyro or changing rotation;
     *   accel PER AXIS: a slow constant rotation (turns gravity in sensor axes;
     *   invisible to every gyro test and to |a|) or a slow push.
     * 0.05 m/s^2 accel spread ~ a 0.3 deg change of the gravity direction.
     */
    uint32_t segmentMs = 1000u;
    float gyroSegmentSpreadMax_radps = 0.005f;
    float accelSegmentSpreadMax_mps2 = 0.05f;
};

/// GyroBiasCalibrator acceptance.
struct GyroCalibrationConfig {
    uint32_t minSamples = 400u;          ///< same as the stationary minimum (80 % of 500)
    float maxStd_radps = 0.01f;          ///< same as StationaryConfig::gyroStdMax_radps
    float maxAbsBias_radps = 10.0f * units::kDegToRad;  ///< larger "bias" = motion or broken sensor
};

/**
 * Pressure -> relative altitude model (see Calibration.hpp, baro namespace).
 *   StandardSeaLevelScale: h = 44330 * (1 - (P/P0)^0.1903)  - the specified
 *       baseline. 44330 m = T0/L with T0 = 288.15 K (ISA SEA-LEVEL temperature),
 *       so it over-states height changes by 44330 / (44330 - h0) when the
 *       reference is at ISA altitude h0: +3.5 % at a 1500 m take-off.
 *   IsaAtReference: same formula with scale 44330 * (P0/101325)^0.1903 (ISA
 *       temperature AT the reference pressure) - exact inside the ISA model.
 * Neither knows the real air temperature: a +/-20 K deviation from ISA is a
 * +/-7 % scale error in both, larger than the difference between them.
 */
enum class BaroAltitudeModel : uint8_t {
    StandardSeaLevelScale = 0,
    IsaAtReference,
};

/// BaroReferenceCalibrator acceptance.
struct BaroCalibrationConfig {
    uint32_t minSamples = 200u;          ///< 5 s at 50 Hz = 250 expected, 80 %
    /// Pressure std: 10 Pa ~ 0.85 m near sea level. Rejects a module carried up/down stairs
    /// or strong gusts; sensor noise with the configured oversampling is far smaller.
    float maxStd_pa = 10.0f;
    /// Segment-mean spread: 12 Pa ~ 1 m. Weather (~1 hPa / 3 h ~ 0.01 Pa/s) is negligible in 5 s.
    float maxSegmentSpread_pa = 12.0f;
    uint32_t segmentMs = 1000u;
    /// Default = the formula specified for this milestone.
    BaroAltitudeModel altitudeModel = BaroAltitudeModel::StandardSeaLevelScale;
};

/// CalibrationController session.
struct CalibrationSessionConfig {
    /// A session that has not completed after this long ends with TimedOut.
    uint32_t sessionTimeoutMs = 60000u;
    /// After a TimedOut / NoStationaryImu failure, start a new session automatically.
    bool retryOnFailure = true;
    /// Start a session automatically when the pipeline starts.
    bool calibrateAtStartup = true;
};

/// RedundancyVoter (one per redundant channel).
struct VoterConfig {
    /// Two sensors agree when |a - b| <= absTolerance + relTolerance * max(|a|, |b|).
    float absTolerance = 1.0f;
    float relTolerance = 0.05f;
    /// Samples further apart in time than this are not compared with each other.
    uint32_t maxSkewMs = 15u;
    /// Consecutive outlier comparisons before a sensor is EXCLUDED (persistence).
    uint16_t excludeAfter = 5u;
    /// Consecutive agreeing comparisons before an excluded/returning sensor is re-INCLUDED (hysteresis).
    uint16_t includeAfter = 20u;
    /// A sensor missing for this many consecutive comparisons must re-qualify (includeAfter) on return.
    uint16_t requalifyAfterAbsent = 10u;
};

/// Top-level preprocessing configuration.
struct PreprocessingConfig {
    ValidationConfig validation{};
    StationaryConfig stationary{};
    GyroCalibrationConfig gyroCal{};
    BaroCalibrationConfig baroCal{};
    CalibrationSessionConfig session{};
    /**
     * IMU specific force: 1.0 m/s^2 + 5 %. Sources of legitimate difference:
     * accel offsets (tens of mg each), lever-arm effects between chips a few cm
     * apart (w^2 r = 0.5 m/s^2 at 5 rad/s, r = 2 cm), sample-time skew.
     */
    VoterConfig accelVoter{1.0f, 0.05f, 15u, 5u, 20u, 10u};
    /**
     * IMU angular rate (after bias calibration): 0.05 rad/s (2.9 dps) + 5 %
     * (MEMS sensitivity tolerance of a few % plus skew during rotation).
     */
    VoterConfig gyroVoter{0.05f, 0.05f, 15u, 5u, 20u, 10u};
    /**
     * Barometric relative altitude: 2.0 m. Barometers at 50 Hz may be read in
     * different 10 ms cycles: skew up to 30 ms. Persistence counted in baro samples.
     */
    VoterConfig baroVoter{2.0f, 0.0f, 30u, 3u, 10u, 5u};
    /// Gyro output requires a valid bias calibration (recommended). If false,
    /// uncalibrated rates are voted and the frame is flagged.
    bool requireGyroCalibration = true;
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_PROCESSING_CONFIG_HPP
