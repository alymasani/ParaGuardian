/**
 ******************************************************************************
 * @file    CrashInputAdapter.hpp
 * @brief   Fills the CrashInput from the real pipeline outputs, keeps the
 *          IncidentContext (last valid position / navigation for an SOS
 *          request) and hands the CrashReport to other tasks.
 *
 * IMPULSE DATA come from ProcessedSensorFrame::imu[i] peaks: preprocessed
 * (validated, factory-calibrated, aligned) but NOT voted, NOT filtered by
 * Madgwick and NOT smoothed by the UKF, which would attenuate or reject the
 * very spike an impact produces. The UKF is used only for CONTEXT (sink rate
 * before the impact).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_INPUT_ADAPTER_HPP
#define PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_INPUT_ADAPTER_HPP

#include <cmath>
#include <cstdint>

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection
#include "Algorithms/Attitude/AttitudeTypes.hpp"
#include "Algorithms/Attitude/Quaternion.hpp"
#include "Algorithms/Crash/CrashTypes.hpp"
#include "Algorithms/Landing/LandingSafetyTypes.hpp"
#include "Algorithms/Navigation/NavigationTypes.hpp"
#include "Processing/ProcessedSensorFrame.hpp"

namespace pg {

/// Angle between the body z (down) axis and the true vertical, from q_ENU_from_body.
inline float bodyTiltFromVertical_rad(const Quaternion& qEnuFromBody)
{
    const Vec3f down = quat::rotate(qEnuFromBody, Vec3f{0.0f, 0.0f, 1.0f});   // body down axis in ENU
    float c = -down.z;   // cos(angle to ENU down)
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return std::acos(c);
}

inline CrashInput crashInputFrom(const ProcessedSensorFrame& f, const AttitudeEstimate& a, const SafetySnapshot& s,
                                 const NavigationEstimate& n)
{
    CrashInput in{};
    in.nowMs = f.timestampMs;
    for (size_t i = 0; i < kImuCount; ++i) {
        const ImuChannelInfo& c = f.imu[i];
        CrashImuChannel& o = in.imu[i];
        o.present = c.present;
        o.samples = c.samplesSinceFrame;
        o.peakAccel_mps2 = c.peakAccelNorm_mps2;
        o.peakRate_radps = c.peakGyroNorm_radps;
        o.peakTimestampMs = c.peakTimestampMs;
        o.accelSaturated = c.accelSaturatedSinceFrame;
        o.gyroSaturated = c.gyroSaturatedSinceFrame;
    }
    const Vec3f& sf = f.specificForce_mps2.value;
    in.forceValid = f.specificForce_mps2.valid;
    in.forceNorm_mps2 = std::sqrt(sf.x * sf.x + sf.y * sf.y + sf.z * sf.z);
    const Vec3f& w = f.angularRate_radps.value;
    in.rateValid = f.angularRate_radps.valid;
    in.rateNorm_radps = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
    in.newImuData = !f.has(quality::NoNewImuData) && !f.has(quality::ImuUnavailable);

    in.attitudeValid = a.orientationValid;
    in.tilt_rad = a.orientationValid ? bodyTiltFromVertical_rad(a.q) : std::nanf("");

    in.phase = s.phase;
    in.clearanceValid = s.clearanceValid;
    in.clearance_m = s.clearance_m;
    in.closureValid = s.closureRateValid;
    in.closureRate_mps = s.closureRate_mps;

    in.vzValid = n.velocityValid;
    in.vz_mps = n.verticalVelocity_mps;

    in.gnssValid = f.gnss.valid;
    in.groundSpeed_mps = std::sqrt(f.gnss.velocityNorth_mps * f.gnss.velocityNorth_mps +
                                   f.gnss.velocityEast_mps * f.gnss.velocityEast_mps);
    in.gnssTimestampMs = f.gnss.timestampMs;
    return in;
}

/**
 * Keeps the LAST VALID fix (never replaced by an invalid one, never invented) and the
 * latest valid navigation values. Each keeps its own timestamp so the SOS request can
 * state its age.
 */
inline void updateIncidentContext(IncidentContext& ctx, const ProcessedSensorFrame& f, const NavigationEstimate& n)
{
    const GnssInfo& g = f.gnss;
    const bool fix = g.valid && (g.fixType == GnssFixType::Fix2D || g.fixType == GnssFixType::Fix3D ||
                                 g.fixType == GnssFixType::GnssPlusDeadReckoning);
    if (fix && (!ctx.gnssFixValid || g.timestampMs != ctx.fixTimestampMs)) {
        ctx.gnssFixValid = true;
        ctx.latitude_1e7deg = g.latitude_1e7deg;
        ctx.longitude_1e7deg = g.longitude_1e7deg;
        ctx.heightMsl_m = g.heightMsl_m;
        ctx.horizontalAccuracy_m = g.horizontalAccuracy_m;
        ctx.fixType = static_cast<uint8_t>(g.fixType);
        ctx.numSatellites = g.numSatellites;
        ctx.fixTimestampMs = g.timestampMs;
    }
    if (n.altitudeValid && std::isfinite(n.altitude_m)) {
        ctx.navAltitudeValid = true;
        ctx.navAltitude_m = n.altitude_m;
        ctx.navTimestampMs = n.timestampMs;
    }
    if (n.velocityValid && std::isfinite(n.verticalVelocity_mps)) {
        ctx.navVelocityValid = true;
        ctx.navVerticalVelocity_mps = n.verticalVelocity_mps;
    }
    ctx.frameQualityFlags = f.qualityFlags;
}

/// Latest-value hand-over of the CrashReport (same pattern as the other mailboxes).
class CrashReportMailbox {
public:
    explicit CrashReportMailbox(ICriticalSection& lock) : lock_(lock) {}
    void publish(const CrashReport& r)
    {
        lock_.enter();
        value_ = r;
        present_ = true;
        lock_.exit();
    }
    bool read(CrashReport& out) const
    {
        lock_.enter();
        const bool p = present_;
        if (p) out = value_;
        lock_.exit();
        return p;
    }

private:
    ICriticalSection& lock_;
    CrashReport value_{};
    bool present_ = false;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_INPUT_ADAPTER_HPP
