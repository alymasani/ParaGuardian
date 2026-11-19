/**
 ******************************************************************************
 * @file    GroundClearanceEstimator.hpp
 * @brief   TF02-Pro range + attitude -> terrain clearance, closure rate and
 *          time to contact, each with explicit validity and uncertainty.
 *
 * Per update (one call per processing frame):
 *   1. Is there a NEW accepted LiDAR sample? (same timestamp = NotNew: the
 *      previous result is carried and ages; never processed twice)
 *   2. If the LiDAR is invalid, WHY? (recent rejection reasons from
 *      preprocessing: beyond range / weak signal; otherwise stale / none)
 *   3. Attitude valid and synchronised with the LiDAR sample?
 *   4. Beam into ENU: d_ENU = q (x) d_body (x) q*; projection = -d_ENU.z;
 *      reject if the beam is more than maxBeamTiltDeg from straight down.
 *   5. clearance = r * projection, with sigma from range + tilt error.
 *   6. Step check against the predicted clearance (flag + restart the rate
 *      history, the sample stays valid: real terrain steps are preserved).
 *   7. Closure rate = -(least-squares slope) over the last rateWindowMs.
 *   8. TTC = clearance / closure, only when the closure is significantly > 0.
 *
 * No smoothing is applied to the clearance itself (no added latency). Only
 * the closure RATE is filtered (window fit, latency ~ half the window).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_LANDING_GROUND_CLEARANCE_ESTIMATOR_HPP
#define PARAGUARDIAN_ALGORITHMS_LANDING_GROUND_CLEARANCE_ESTIMATOR_HPP

#include <cstdint>

#include "Algorithms/Landing/GroundClearanceTypes.hpp"
#include "Algorithms/Safety/SafetyInput.hpp"

namespace pg {

class GroundClearanceEstimator {
public:
    explicit GroundClearanceEstimator(const GroundClearanceConfig& cfg = GroundClearanceConfig{});

    bool configValid() const { return configValid_; }
    const GroundClearance& update(const SafetyInput& in);
    const GroundClearance& latest() const { return out_; }
    void reset();

    /// Beam direction in ENU for orientation @p q (exposed for tests).
    Vec3f beamEnu(const Quaternion& q) const;

private:
    void setUnavailable(ClearanceStatus s);
    void resetRate();
    void addRateSample(uint32_t ts, float c);
    void fitRate();

    GroundClearanceConfig cfg_;
    bool configValid_;
    Vec3f beamBodyUnit_{0.0f, 0.0f, 1.0f};
    float cosMaxTilt_ = 0.766f;
    float cosHighTilt_ = 0.94f;

    bool haveSample_ = false;
    uint32_t lastSampleTs_ = 0u;
    bool havePrevClearance_ = false;
    float prevClearance_ = 0.0f;
    uint32_t prevClearanceTs_ = 0u;

    uint32_t rateTs_[GroundClearanceConfig::kRateCapacity] = {};
    float rateC_[GroundClearanceConfig::kRateCapacity] = {};
    uint32_t rateCount_ = 0u;

    GroundClearance out_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_LANDING_GROUND_CLEARANCE_ESTIMATOR_HPP
