/**
 ******************************************************************************
 * @file    AttitudeFrameAdapter.hpp
 * @brief   ProcessedSensorFrame -> AttitudeInput (field mapping only, no maths).
 *
 * Kept separate so the estimator itself does not depend on the frame layout
 * (simulation tests drive it directly with AttitudeInput).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_FRAME_ADAPTER_HPP
#define PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_FRAME_ADAPTER_HPP

#include "Algorithms/Attitude/AttitudeTypes.hpp"
#include "Processing/ProcessedSensorFrame.hpp"

namespace pg {

inline AttitudeInput attitudeInputFromFrame(const ProcessedSensorFrame& f)
{
    AttitudeInput in{};
    in.nowMs = f.timestampMs;

    in.gyro_radps = f.angularRate_radps.value;
    in.gyroTimestampMs = f.angularRate_radps.timestampMs;   // newest contributing IMU sample
    in.gyroValid = f.angularRate_radps.valid;
    in.gyroCalibrated = !f.has(quality::GyroNotCalibrated);
    in.gyroSourceMask = f.angularRate_radps.contributingMask;
    in.gyroStatus = f.angularRate_radps.status;

    in.accel_mps2 = f.specificForce_mps2.value;
    in.accelTimestampMs = f.specificForce_mps2.timestampMs;
    in.accelValid = f.specificForce_mps2.valid;
    in.accelSourceMask = f.specificForce_mps2.contributingMask;
    in.accelStatus = f.specificForce_mps2.status;

    in.mag_uT = f.mag.fieldBody_uT;
    in.magTimestampMs = f.mag.timestampMs;
    in.magValid = f.mag.valid;
    in.magCalibrated = f.mag.ironCalibrated;

    in.mountingVerified = !f.has(quality::MountingUnverified);
    in.accelCalibrated = !f.has(quality::AccelNotCalibrated);
    return in;
}

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_ATTITUDE_ATTITUDE_FRAME_ADAPTER_HPP
