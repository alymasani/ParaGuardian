/**
 ******************************************************************************
 * @file    PreprocessingPipeline.hpp
 * @brief   Raw SensorMessage stream in -> ProcessedSensorFrame out.
 *          Portable C++ (no HAL, no FreeRTOS): runs unchanged in host tests.
 *
 *   ingest(msg, now)          for every message taken from the queue:
 *     1. SampleValidator      validity, freshness, ranges, sequence, timestamps
 *     2. CalibrationController (only while a session is collecting)
 *     3. per-sensor correction: gyro - bias (sensor axes), then
 *        AxisAlignment sensor -> body (FRD); barometer -> relative altitude
 *     4. store as that sensor's LATEST VALID sample (+ its own timestamp)
 *
 *   buildFrame(now, frame)    once per batch (after draining the queue):
 *     5. freshness of every stored sample at `now` (per-sensor max age)
 *     6. RedundancyVoter: accel (3 IMUs), gyro (3 IMUs), altitude (2 baros)
 *     7. ProcessedSensorFrame with every validity/quality/age field filled
 *
 * Each sensor keeps its own timestamp; no sample is shifted, interpolated or
 * invented. Missing data stays "not valid", never 0.
 *
 * Memory: ~3 KB, all inside this object (static in the RTOS task). No heap.
 * Thread safety: none needed - owned and called by ONE task only.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_PREPROCESSING_PIPELINE_HPP
#define PARAGUARDIAN_PROCESSING_PREPROCESSING_PIPELINE_HPP

#include <cstdint>

#include "Acquisition/SensorMessage.hpp"
#include "Processing/AxisAlignment.hpp"
#include "Processing/Calibration.hpp"
#include "Processing/CalibrationController.hpp"
#include "Processing/ProcessedSensorFrame.hpp"
#include "Processing/ProcessingConfig.hpp"
#include "Processing/RedundancyVoter.hpp"
#include "Processing/SampleValidator.hpp"

namespace pg {

struct PipelineStats {
    uint32_t messagesIngested;
    uint32_t messagesAccepted;
    uint32_t messagesRejected;
    uint32_t framesBuilt;
    uint32_t framesWithoutImu;      ///< frames where no IMU output was valid (critical)
    uint32_t framesWithoutBaro;     ///< frames where no barometric altitude was valid
    uint32_t imuAmbiguousFrames;
    uint32_t baroAmbiguousFrames;
    uint32_t samplesLost;           ///< sum of sequence gaps
};

class PreprocessingPipeline {
public:
    /// Largest accepted interval between consecutive angular-rate outputs for gyroDtValid.
    static constexpr uint32_t kMaxGyroDtMs = 50u;

    explicit PreprocessingPipeline(const PreprocessingConfig& cfg = PreprocessingConfig{});

    /// Call once with the current time; starts the start-up calibration if configured.
    void start(uint32_t nowMs);
    /// Starts a new calibration session (repeat requests restart it).
    void requestCalibration(uint32_t nowMs) { calibration_.request(nowMs); }

    /// Processes one message. Returns the validation result (for tests / diagnostics).
    ValidationResult ingest(const SensorMessage& msg, uint32_t nowMs);

    /// Builds the frame describing the newest valid data at @p nowMs.
    void buildFrame(uint32_t nowMs, ProcessedSensorFrame& out);

    /// True if any accepted sample arrived since the last buildFrame().
    bool hasNewData() const { return newImu_ || newBaro_ || newOther_; }

    /// Sensor mountings (configure before start(); defaults are UNVERIFIED identity).
    AxisAlignment& alignment() { return alignment_; }
    const AxisAlignment& alignment() const { return alignment_; }
    /// Future accelerometer calibration per IMU (not estimated in this milestone).
    void setAccelCalibration(size_t imu, const AccelCalibration& c);

    const CalibrationController& calibration() const { return calibration_; }
    const SampleValidator& validator() const { return validator_; }
    const RedundancyVoter& accelVoter() const { return accelVoter_; }
    const RedundancyVoter& gyroVoter() const { return gyroVoter_; }
    const RedundancyVoter& baroVoter() const { return baroVoter_; }
    const PipelineStats& stats() const { return stats_; }
    const PreprocessingConfig& config() const { return cfg_; }

private:
    struct ImuState {
        bool present;
        uint32_t timestampMs;
        Vec3f accelBody;
        Vec3f gyroBody;           ///< bias-corrected only if gyroCalibrated
        bool accelUsable;
        bool gyroUsable;
        bool gyroCalibrated;
        uint32_t lastRejectReasons;
    };
    struct BaroState {
        bool present;
        uint32_t timestampMs;
        float pressure_pa;
        double relativeAltitude_m;
        bool altitudeValid;
        uint32_t referenceSession;
        uint32_t lastRejectReasons;
    };
    struct MagState {
        bool present;
        uint32_t timestampMs;
        Vec3f fieldBody;
    };
    struct GnssState {
        bool present;
        GnssSample sample;
    };
    struct LidarState {
        bool present;
        LidarSample sample;
    };

    bool fresh(SensorId id, uint32_t timestampMs, uint32_t nowMs) const;
    void handleImu(const ImuSample& s, const ValidationResult& v);
    void handleBaro(const BaroSample& s);
    void fillImu(uint32_t nowMs, ProcessedSensorFrame& f);
    void fillBaro(uint32_t nowMs, ProcessedSensorFrame& f);
    void fillOthers(uint32_t nowMs, ProcessedSensorFrame& f);

    PreprocessingConfig cfg_;
    SampleValidator validator_;
    CalibrationController calibration_;
    AxisAlignment alignment_{};
    AccelCalibration accelCal_[kImuCount] = {};
    RedundancyVoter accelVoter_;
    RedundancyVoter gyroVoter_;
    RedundancyVoter baroVoter_;

    ImuState imu_[kImuCount] = {};
    BaroState baro_[kBaroCount] = {};
    MagState mag_{};
    GnssState gnss_{};
    LidarState lidar_{};

    bool newImu_ = false;
    bool newBaro_ = false;
    bool newOther_ = false;
    uint32_t lostSinceFrame_ = 0u;
    uint32_t rejectedSinceFrame_ = 0u;
    uint32_t frameSequence_ = 0u;
    bool lastGyroValid_ = false;
    uint32_t lastGyroTimestampMs_ = 0u;
    uint8_t lastBaroContributing_ = 0u;
    bool lastBaroValid_ = false;
    PipelineStats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_PREPROCESSING_PIPELINE_HPP
