/**
 ******************************************************************************
 * @file    NavigationInputAdapter.hpp
 * @brief   ProcessedSensorFrame + AttitudeEstimate -> NavigationInput
 *          (field mapping only, no maths).
 *
 * Gravity is NOT removed here: the vertical acceleration is the attitude
 * estimator's gravity-compensated AttitudeEstimate::linearAccel.vertical_mps2
 * (positive up). GNSS vertical velocity is already positive up (the NEO-M9N
 * driver converts NAV-PVT velD, positive DOWN, with a minus sign).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_INPUT_ADAPTER_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_INPUT_ADAPTER_HPP

#include <limits>

#include "Algorithms/Attitude/AttitudeTypes.hpp"
#include "Algorithms/Navigation/NavigationTypes.hpp"
#include "Processing/ProcessedSensorFrame.hpp"

namespace pg {

inline NavigationInput navigationInputFrom(const ProcessedSensorFrame& f, const AttitudeEstimate& a)
{
    constexpr float kDegToRad = 0.017453292519943295f;
    NavigationInput in{};
    in.nowMs = f.timestampMs;

    const ScalarEstimate& b = f.baroAltitude_m;
    in.baroValid = b.valid;
    in.baroAltitude_m = b.value;
    in.baroTimestampMs = b.timestampMs;
    in.baroSpreadMs = b.sampleSpreadMs;
    in.baroStatus = b.status;
    in.baroContributingMask = b.contributingMask;
    for (size_t j = 0; j < kBaroCount; ++j) {
        in.baroChannelTimestampMs[j] = f.baro[j].timestampMs;
        in.baroReferencePressure_pa[j] = f.baro[j].referencePressure_pa;
    }
    in.baroSourceChanged = f.has(quality::BaroSourceChanged);

    in.accelValid = a.orientationValid && a.linearAccel.valid;
    in.accelReliable = a.linearAccel.reliable;
    in.accelUp_mps2 = a.linearAccel.vertical_mps2;
    in.accelTimestampMs = a.linearAccel.timestampMs;
    in.attitudeState = a.state;
    in.attitudeAccelCorrection = a.accelCorrection;
    in.attitudeFlags = a.qualityFlags;
    in.imuAccelStatus = f.specificForce_mps2.status;
    in.imuGyroStatus = f.angularRate_radps.status;

    const GnssInfo& g = f.gnss;
    in.gnssValid = g.valid;
    in.gnssVerticalVelocity_mps = g.verticalVelocity_mps;
    in.gnssSpeedAccuracy_mps = g.speedAccuracy_mps;
    in.gnssTimestampMs = g.timestampMs;
    in.gnssTimeOfWeekMs = g.timeOfWeek_ms;
    in.gnssFixType = g.fixType;
    in.gnssNumSatellites = g.numSatellites;

    in.model.timestampMs = a.timestampMs;
    in.model.attitudeValid = a.orientationValid;
    in.model.bankAngle_rad = a.orientationValid ? a.eulerDeg.roll * kDegToRad : std::numeric_limits<float>::quiet_NaN();
    in.model.pitchAngle_rad = a.orientationValid ? a.eulerDeg.pitch * kDegToRad : std::numeric_limits<float>::quiet_NaN();
    return in;
}

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_INPUT_ADAPTER_HPP
