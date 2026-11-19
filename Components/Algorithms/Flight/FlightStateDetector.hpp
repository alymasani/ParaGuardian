/**
 ******************************************************************************
 * @file    FlightStateDetector.hpp
 * @brief   Evidence-based flight-phase state machine (see FlightStateTypes.hpp
 *          and FLIGHT_LANDING_DESIGN.md section 4).
 *
 * Each update computes independent EVIDENCE bits (airborne: LiDAR high, GNSS
 * fast, sustained vertical motion, altitude departure from the ground
 * reference; ground: LiDAR near ground, GNSS slow, altitude steady, IMU
 * still). Transitions need several sources and/or persistence:
 *
 *   Unknown --(consistent evidence startupDecisionMs)--> Ground | Flying
 *   Ground/Landed --(any airborne evidence)--> TakeoffCandidate
 *   TakeoffCandidate --(>=2 sources or LiDAR-high for takeoffConfirmMs,
 *                       or 1 source except altitude-departure for
 *                       takeoffSingleSourceMs; LiDAR near ground vetoes)--> Flying
 *   TakeoffCandidate --(no airborne evidence takeoffAbortMs)--> Ground/Landed (aborted)
 *   Flying <--(valid LiDAR clearance < approach / > exit, hysteresis)--> Approach
 *   Flying/Approach --(>=2 ground sources, no airborne, landingCandidateMs)--> LandingCandidate
 *   LandingCandidate --(held landingConfirmMs)--> Landed
 *   LandingCandidate --(airborne evidence / ground evidence lost)--> Flying/Approach
 *
 * With NO usable input at all the phase is HELD (flag PhaseHeld) and timers
 * do not advance: a sensor outage never produces "Landed".
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_FLIGHT_FLIGHT_STATE_DETECTOR_HPP
#define PARAGUARDIAN_ALGORITHMS_FLIGHT_FLIGHT_STATE_DETECTOR_HPP

#include <cstdint>

#include "Algorithms/Flight/FlightStateTypes.hpp"
#include "Algorithms/Landing/GroundClearanceTypes.hpp"
#include "Algorithms/Safety/SafetyInput.hpp"

namespace pg {

class FlightStateDetector {
public:
    explicit FlightStateDetector(const FlightStateConfig& cfg = FlightStateConfig{});

    bool configValid() const { return configValid_; }
    const FlightStateStatus& update(const SafetyInput& in, const GroundClearance& clearance);
    const FlightStateStatus& latest() const { return out_; }
    void reset();

private:
    static constexpr uint32_t kSteadySlots = 17u;   ///< altitude samples, one per 500 ms (8 s window + 1)

    void updateMotion(const SafetyInput& in);
    void updateSteadyWindow(const SafetyInput& in, bool navOk);
    void enter(FlightPhase p, uint32_t nowMs);
    bool dwellOk(uint32_t nowMs) const;
    /// Tracks how long @p cond has been continuously true (0 = false now).
    static uint32_t persist(bool cond, bool& active, uint32_t& since, uint32_t nowMs);

    FlightStateConfig cfg_;
    bool configValid_;
    FlightStateStatus out_{};

    // motion RMS (exponential)
    float msAccel_ = 0.0f, msRate_ = 0.0f;
    bool motionAccelInit_ = false, motionRateInit_ = false;
    uint32_t lastAccelTs_ = 0u, lastRateTs_ = 0u;
    uint32_t accelObservedMs_ = 0u, rateObservedMs_ = 0u;

    // steady-altitude window
    float steadyAlt_[kSteadySlots] = {};
    uint32_t steadyCount_ = 0u;
    uint32_t lastSteadyPushMs_ = 0u;
    bool steadyHavePush_ = false;

    // persistence trackers
    bool vzActive_ = false;      uint32_t vzSince_ = 0u;
    bool multiActive_ = false;   uint32_t multiSince_ = 0u;
    bool singleActive_ = false;  uint32_t singleSince_ = 0u;
    bool groundActive_ = false;  uint32_t groundSince_ = 0u;
    bool startAirActive_ = false; uint32_t startAirSince_ = 0u;
    bool startGndActive_ = false; uint32_t startGndSince_ = 0u;
    bool approachLostActive_ = false; uint32_t approachLostSince_ = 0u;
    uint32_t lastAirborneEvidenceMs_ = 0u;
    uint32_t lastGroundEvidenceMs_ = 0u;
    FlightPhase groundPhaseBeforeCandidate_ = FlightPhase::Ground;
    FlightPhase airPhaseBeforeCandidate_ = FlightPhase::Flying;
    bool started_ = false;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_FLIGHT_FLIGHT_STATE_DETECTOR_HPP
