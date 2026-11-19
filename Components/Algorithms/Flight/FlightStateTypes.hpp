/**
 ******************************************************************************
 * @file    FlightStateTypes.hpp
 * @brief   Flight phase, evidence and configuration of the FlightStateDetector.
 *
 * The phase is a BEST-SUPPORTED HYPOTHESIS from heuristic evidence, not an
 * observed fact: the sensor set cannot prove "airborne" or "landed".
 * "evidence" counts and the EvidenceLevel are heuristic, NOT probabilities.
 *
 * All thresholds are PROVISIONAL DEVELOPMENT VALUES chosen from general
 * paragliding magnitudes (trim sink ~1-1.5 m/s, airspeed ~8-15 m/s, walking
 * < 2 m/s) and the host simulations (FLIGHT_LANDING_VALIDATION_REPORT.md).
 * None is validated on real flights.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_FLIGHT_FLIGHT_STATE_TYPES_HPP
#define PARAGUARDIAN_ALGORITHMS_FLIGHT_FLIGHT_STATE_TYPES_HPP

#include <cstdint>

namespace pg {

/// Best-supported flight phase (see FLIGHT_LANDING_DESIGN.md section 4).
enum class FlightPhase : uint8_t {
    Unknown = 0,       ///< not enough evidence yet (start-up, or started in flight before confirmation)
    Ground,            ///< on the ground, never confirmed airborne since start
    TakeoffCandidate,  ///< airborne evidence appeared; confirming
    Flying,            ///< airborne confirmed
    Approach,          ///< flying with valid LiDAR clearance below approachClearance_m
    LandingCandidate,  ///< ground evidence while flying; confirming
    Landed,            ///< on the ground after a flight
};

constexpr bool isAirbornePhase(FlightPhase p)
{
    return p == FlightPhase::Flying || p == FlightPhase::Approach || p == FlightPhase::LandingCandidate;
}
constexpr bool isGroundPhase(FlightPhase p) { return p == FlightPhase::Ground || p == FlightPhase::Landed; }

/// How well the current phase is supported RIGHT NOW (heuristic, not a probability).
enum class EvidenceLevel : uint8_t {
    None = 0,      ///< no usable input at all: phase is held from the last decision
    Weak,          ///< one evidence source supports the phase
    Moderate,      ///< two sources
    Strong,        ///< three or more sources, or a strong source (valid LiDAR clearance)
};

namespace flightev {
/// Evidence source bits (FlightStateStatus::airborneEvidence / groundEvidence).
enum : uint16_t {
    LidarHigh         = 1u << 0,   ///< valid clearance above groundMaxClearance_m, or beyond LiDAR range with good geometry
    LidarNearGround   = 1u << 1,   ///< valid clearance below groundMaxClearance_m
    GnssFast          = 1u << 2,   ///< GNSS ground speed above airborneGroundSpeed_mps
    GnssSlow          = 1u << 3,   ///< GNSS ground speed below groundGroundSpeed_mps
    VerticalMotion    = 1u << 4,   ///< |vz| above verticalMotion_mps, sustained
    AltitudeDeparture = 1u << 5,   ///< relative altitude left the ground reference by departureAltitude_m
    AltitudeSteady    = 1u << 6,   ///< altitude within +-steadyAltitudeBand_m and |vz| small over the window
    Still             = 1u << 7,   ///< IMU: linear acceleration and rotation rate quiet over the window
    LowSpeedVertical  = 1u << 8,   ///< |vz| below groundVz_mps (and valid)
};
}  // namespace flightev

namespace flightflag {
/// FlightStateStatus::flags
enum : uint32_t {
    NavigationUnavailable = 1u << 0,   ///< no valid UKF altitude / velocity this update
    NavigationDegraded    = 1u << 1,   ///< UKF Degraded / Recovering
    LidarUnavailable      = 1u << 2,
    GnssUnavailable       = 1u << 3,
    ImuUnavailable        = 1u << 4,
    PhaseHeld             = 1u << 5,   ///< no evidence either way: phase held, not re-decided
    TakeoffVetoedByLidar  = 1u << 6,   ///< airborne evidence present, but LiDAR says near ground
    ModelPlaceholder      = 1u << 7,   ///< navigation from the placeholder process model (inherited)
    StartedUnknown        = 1u << 8,   ///< start-up phase not yet decided
};
}  // namespace flightflag

struct FlightStateConfig {
    // ---- evidence thresholds ----
    float groundMaxClearance_m = 2.5f;        ///< LiDAR clearance below = near ground (harness height + margin)
    float airborneGroundSpeed_mps = 6.0f;     ///< GNSS: faster than a running pilot
    float groundGroundSpeed_mps = 1.0f;       ///< GNSS: slower than walking
    float gnssMaxSpeedAccuracy_mps = 1.0f;    ///< GNSS evidence only if sAcc below
    float verticalMotion_mps = 0.8f;          ///< |vz| above, for verticalMotionMs = airborne evidence
    uint32_t verticalMotionMs = 3000u;
    float groundVz_mps = 0.3f;                ///< |vz| below = consistent with ground
    float departureAltitude_m = 15.0f;        ///< |h - h_ground| above = altitude departure
    float steadyAltitudeBand_m = 1.0f;        ///< ground: altitude stays within +- this over steadyWindowMs
    uint32_t steadyWindowMs = 8000u;
    float stillAccel_mps2 = 0.4f;             ///< ground stillness: RMS |linear accel| below
    float stillRate_radps = 0.05f;            ///< ground stillness: RMS |rate| below
    float motionTau_s = 2.0f;                 ///< time constant of the IMU motion RMS
    float maxNavVelocitySigma_mps = 1.0f;     ///< vz evidence only if sigma below
    uint32_t maxInputAgeMs = 500u;            ///< older inputs are ignored as evidence

    // ---- transitions (dwell / persistence) ----
    uint32_t takeoffConfirmMs = 3000u;        ///< candidate -> Flying: >= 2 sources (or strong) for this long
    uint32_t takeoffSingleSourceMs = 15000u;  ///< ... or one non-strong source for this long (never AltitudeDeparture alone)
    uint32_t takeoffAbortMs = 2000u;          ///< candidate with no airborne evidence this long -> back to ground
    uint32_t landingCandidateMs = 2000u;      ///< ground evidence (>= 2 sources, no airborne) this long -> LandingCandidate
    uint32_t landingConfirmMs = 5000u;        ///< LandingCandidate held this long -> Landed
    uint32_t startupDecisionMs = 5000u;       ///< Unknown -> Ground / Flying needs this long of consistent evidence
    float approachClearance_m = 30.0f;        ///< Flying -> Approach below (valid LiDAR)
    float approachExitClearance_m = 45.0f;    ///< Approach -> Flying above (hysteresis)
    uint32_t approachExitMs = 3000u;          ///< ... or LiDAR unavailable this long
    uint32_t minPhaseDwellMs = 1000u;         ///< no phase change sooner than this after the last one (anti-oscillation)

    bool isValid() const
    {
        return groundMaxClearance_m > 0.0f && airborneGroundSpeed_mps > groundGroundSpeed_mps &&
               verticalMotion_mps > groundVz_mps && departureAltitude_m > steadyAltitudeBand_m &&
               approachExitClearance_m > approachClearance_m && approachClearance_m > groundMaxClearance_m &&
               motionTau_s > 0.0f && takeoffConfirmMs > 0u && landingConfirmMs > 0u && steadyWindowMs >= 1000u &&
               takeoffSingleSourceMs >= takeoffConfirmMs;
    }
};

/// Output of the FlightStateDetector.
struct FlightStateStatus {
    FlightPhase phase;
    FlightPhase previousPhase;
    uint32_t phaseSinceMs;          ///< time the phase was entered
    uint32_t timeInPhaseMs;
    EvidenceLevel support;          ///< support for the CURRENT phase this update (heuristic)
    uint16_t airborneEvidence;      ///< flightev::* bits active now
    uint16_t groundEvidence;
    uint32_t flags;                 ///< flightflag::*
    float groundReferenceAltitude_m;///< UKF altitude adopted as "ground" (NaN = none)
    bool groundReferenceValid;
    float motionAccelRms_mps2;      ///< IMU motion indicator (NaN = unavailable)
    float motionRateRms_radps;
    uint32_t transitions;           ///< lifetime counters (saturating)
    uint32_t takeoffs;
    uint32_t landings;
    uint32_t abortedTakeoffs;
    uint32_t heldUpdates;           ///< updates with no evidence at all (phase held)
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_FLIGHT_FLIGHT_STATE_TYPES_HPP
