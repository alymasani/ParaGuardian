/**
 ******************************************************************************
 * @file    CalibrationController.hpp
 * @brief   Start-up (and on-request) calibration session: one stationary
 *          detector + gyro-bias calibrator per IMU, one reference calibrator
 *          per barometer, and the state machine that ties them together.
 *
 * SESSION STATE MACHINE
 *
 *     NotCalibrated --request()--> Collecting --complete--> Valid
 *                                   |  ^   |
 *                motion on ANY IMU  |  |   +--timeout--> Failed  (or a new
 *                (restart window)   +--+                  session if retryOnFailure)
 *
 *   Collecting:
 *     - every accepted IMU sample (both channels usable, sensor axes) goes to
 *       that IMU's StationaryDetector and, while still, to its gyro calibrator;
 *     - every accepted barometer sample goes to its reference calibrator;
 *     - if ANY IMU's detector reports motion, ALL accumulators restart (the
 *       device is one rigid body: if one IMU moved, the barometers moved too).
 *   Completion (checked by poll()):
 *     at least one IMU detector is Stationary (a full still window), and either
 *     every IMU seen in this session is Stationary or completionGraceMs has
 *     passed beyond the window (so a missing / intermittent IMU cannot block
 *     the others forever). Then each calibrator is finished individually.
 *   Nothing is calibrated without inertial confirmation of stillness: a steady
 *   barometer alone never completes a session.
 *
 *   The previous gyro calibrations stay in use while a new session runs.
 *   Barometer references are replaced per session (see BaroReferenceCalibrator).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_CALIBRATION_CONTROLLER_HPP
#define PARAGUARDIAN_PROCESSING_CALIBRATION_CONTROLLER_HPP

#include <cstdint>

#include "Processing/Calibration.hpp"
#include "Processing/ProcessingConfig.hpp"
#include "Processing/ProcessingTypes.hpp"
#include "Processing/StationaryDetector.hpp"

namespace pg {

struct CalibrationSessionStatus {
    CalibrationState state;            ///< overall session state
    CalibrationFailure lastFailure;    ///< reason the last session failed (None if it completed)
    uint32_t sessionId;                ///< current / last session (0 = none yet)
    uint32_t sessionStartMs;
    uint32_t windowStartMs;            ///< start of the current still window
    uint32_t completedSessions;
    uint32_t failedSessions;
    uint32_t motionRestarts;           ///< windows restarted because an IMU moved
    uint32_t lastCompletedMs;
};

class CalibrationController {
public:
    /// Extra time beyond the window to wait for a late / intermittent IMU.
    static constexpr uint32_t kCompletionGraceMs = 1000u;

    explicit CalibrationController(const PreprocessingConfig& cfg);

    /// Starts a new session (also used for repeated requests; restarts a running one).
    void request(uint32_t nowMs);
    /// Accepted IMU sample, both channels usable, SENSOR axes, raw (uncorrected) gyro.
    void onImu(size_t imu, uint32_t timestampMs, const Vec3f& gyroSensor_radps, const Vec3f& accelSensor_mps2);
    /// Accepted barometer sample.
    void onBaro(size_t baro, uint32_t timestampMs, float pressure_pa);
    /// Completion / timeout check. Call after ingesting samples (cheap when idle).
    void poll(uint32_t nowMs);

    bool collecting() const { return status_.state == CalibrationState::Collecting; }
    const CalibrationSessionStatus& status() const { return status_; }

    const GyroBiasCalibrator& gyro(size_t i) const { return gyro_[i < kImuCount ? i : 0u]; }
    const BaroReferenceCalibrator& baro(size_t i) const { return baro_[i < kBaroCount ? i : 0u]; }
    const StationaryDetector& detector(size_t i) const { return detector_[i < kImuCount ? i : 0u]; }

private:
    void restartAll(uint32_t tMs);
    void complete(uint32_t nowMs);
    void failSession(uint32_t nowMs, CalibrationFailure why);

    CalibrationSessionConfig sessionCfg_;
    uint32_t windowMs_;
    StationaryDetector detector_[kImuCount];
    GyroBiasCalibrator gyro_[kImuCount];
    BaroReferenceCalibrator baro_[kBaroCount];
    bool imuSeen_[kImuCount] = {};
    CalibrationSessionStatus status_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_CALIBRATION_CONTROLLER_HPP
