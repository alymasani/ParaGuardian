/**
 ******************************************************************************
 * @file    CalibrationController.cpp
 * @brief   See CalibrationController.hpp.
 ******************************************************************************
 */
#include "Processing/CalibrationController.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

CalibrationController::CalibrationController(const PreprocessingConfig& cfg)
    : sessionCfg_(cfg.session),
      windowMs_(cfg.stationary.windowMs),
      detector_{StationaryDetector(cfg.stationary), StationaryDetector(cfg.stationary),
                StationaryDetector(cfg.stationary)},
      gyro_{GyroBiasCalibrator(cfg.gyroCal), GyroBiasCalibrator(cfg.gyroCal), GyroBiasCalibrator(cfg.gyroCal)},
      baro_{BaroReferenceCalibrator(cfg.baroCal), BaroReferenceCalibrator(cfg.baroCal)}
{
    status_.state = CalibrationState::NotCalibrated;
}

void CalibrationController::request(uint32_t nowMs)
{
    ++status_.sessionId;
    status_.state = CalibrationState::Collecting;
    status_.sessionStartMs = nowMs;
    status_.windowStartMs = nowMs;
    for (size_t i = 0; i < kImuCount; ++i) {
        detector_[i].reset();
        gyro_[i].begin();
        imuSeen_[i] = false;
    }
    for (size_t j = 0; j < kBaroCount; ++j) {
        baro_[j].begin(nowMs);
    }
}

void CalibrationController::restartAll(uint32_t tMs)
{
    if (status_.motionRestarts != UINT32_MAX) {
        ++status_.motionRestarts;
    }
    // The new window starts AFTER the sample that showed motion, so a barometer
    // sample taken in the same millisecond is not used.
    status_.windowStartMs = tMs + 1u;
    for (size_t i = 0; i < kImuCount; ++i) {
        detector_[i].restartWindow();
        gyro_[i].restart();
    }
    for (size_t j = 0; j < kBaroCount; ++j) {
        baro_[j].restart(tMs + 1u);
    }
}

void CalibrationController::onImu(size_t imu, uint32_t t, const Vec3f& gyro, const Vec3f& accel)
{
    if (!collecting() || imu >= kImuCount) {
        return;
    }
    imuSeen_[imu] = true;
    const StationaryStatus st = detector_[imu].add(t, gyro, accel);
    if (st == StationaryStatus::Moving) {
        restartAll(t);
        return;
    }
    // Keep the bias accumulator aligned with the detector's window: when the
    // detector just (re)started its window (first sample, or after a data gap),
    // throw away what the calibrator had.
    if (detector_[imu].report().samples == 1u) {
        gyro_[imu].restart();
    }
    gyro_[imu].add(gyro);
}

void CalibrationController::onBaro(size_t baro, uint32_t t, float pressure_pa)
{
    if (!collecting() || baro >= kBaroCount) {
        return;
    }
    // Samples older than the current window start belong to a period with motion.
    if (timebase::isAfter(status_.windowStartMs, t)) {
        return;
    }
    baro_[baro].add(t, pressure_pa);
}

void CalibrationController::poll(uint32_t nowMs)
{
    if (!collecting()) {
        return;
    }
    if (timebase::ageMs(status_.sessionStartMs, nowMs) >= sessionCfg_.sessionTimeoutMs) {
        bool anyImu = false;
        for (size_t i = 0; i < kImuCount; ++i) anyImu = anyImu || imuSeen_[i];
        failSession(nowMs, anyImu ? CalibrationFailure::TimedOut : CalibrationFailure::NoStationaryImu);
        return;
    }

    bool anyStationary = false;
    bool allSeenStationary = true;
    for (size_t i = 0; i < kImuCount; ++i) {
        const bool still = detector_[i].status() == StationaryStatus::Stationary;
        anyStationary = anyStationary || still;
        if (imuSeen_[i] && !still) {
            allSeenStationary = false;
        }
    }
    if (!anyStationary) {
        return;
    }
    const uint32_t windowAge = timebase::ageMs(status_.windowStartMs, nowMs);
    if (allSeenStationary || windowAge >= windowMs_ + kCompletionGraceMs) {
        complete(nowMs);
    }
}

void CalibrationController::complete(uint32_t nowMs)
{
    for (size_t i = 0; i < kImuCount; ++i) {
        if (detector_[i].status() == StationaryStatus::Stationary) {
            gyro_[i].finish(nowMs, status_.sessionId);  // may still reject (noise / magnitude)
        } else {
            // Not confirmed still (missing, intermittent): evaluated as-is, which
            // fails with InsufficientSamples unless it truly had a full window.
            gyro_[i].restart();
            gyro_[i].finish(nowMs, status_.sessionId);
        }
    }
    for (size_t j = 0; j < kBaroCount; ++j) {
        baro_[j].finish(nowMs, status_.sessionId);
    }
    status_.state = CalibrationState::Valid;
    status_.lastFailure = CalibrationFailure::None;
    status_.lastCompletedMs = nowMs;
    if (status_.completedSessions != UINT32_MAX) {
        ++status_.completedSessions;
    }
}

void CalibrationController::failSession(uint32_t nowMs, CalibrationFailure why)
{
    // Keep every previously valid result (gyro biases AND the previous, mutually
    // consistent set of barometer references): no session result replaced them.
    for (size_t i = 0; i < kImuCount; ++i) gyro_[i].abort();
    for (size_t j = 0; j < kBaroCount; ++j) baro_[j].abort();
    status_.lastFailure = why;
    if (status_.failedSessions != UINT32_MAX) {
        ++status_.failedSessions;
    }
    if (sessionCfg_.retryOnFailure) {
        const CalibrationFailure keep = why;
        request(nowMs);
        status_.lastFailure = keep;  // still report why the previous attempt failed
    } else {
        status_.state = CalibrationState::Failed;
    }
}

}  // namespace pg
