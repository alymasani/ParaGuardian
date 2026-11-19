/**
 ******************************************************************************
 * @file    LandingSafetyTypes.hpp
 * @brief   Warnings, configuration and the SafetySnapshot of the
 *          LandingSafetyAnalyzer.
 *
 * ALL THRESHOLDS ARE INITIAL DEVELOPMENT PARAMETERS. They are NOT a validated
 * "safe descent rate" or "safe clearance" for any wing or pilot; they were
 * chosen from general paragliding magnitudes (trim sink ~1-1.5 m/s; big ears
 * ~3-4 m/s; spiral dives > 10 m/s) and the host simulations
 * (FLIGHT_LANDING_VALIDATION_REPORT.md). The analyzer does not replace pilot
 * judgement, approved equipment or flying procedures.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_LANDING_LANDING_SAFETY_TYPES_HPP
#define PARAGUARDIAN_ALGORITHMS_LANDING_LANDING_SAFETY_TYPES_HPP

#include <cstdint>

#include "Algorithms/Flight/FlightStateTypes.hpp"
#include "Algorithms/Landing/GroundClearanceTypes.hpp"
#include "Algorithms/Navigation/NavigationTypes.hpp"
#include "Algorithms/Safety/SafetyEvent.hpp"

namespace pg {

/// The warnings the analyzer tracks (index into the per-warning arrays).
enum class LandingWarning : uint8_t {
    ExcessiveDescent = 0,
    LowGroundClearance,
    RapidGroundClosure,
    HazardousApproach,
    TerrainClearanceUnavailable,
    NavigationDegraded,
    Count
};
constexpr uint8_t kLandingWarningCount = static_cast<uint8_t>(LandingWarning::Count);

/// Persistence and repetition policy of one warning.
struct WarningPolicy {
    uint32_t raiseMs;        ///< condition must hold this long before Raised
    uint32_t escalateMs;     ///< higher level must hold this long before Escalated
    uint32_t relaxMs;        ///< lower level / no condition must hold this long before Deescalated / Cleared
    uint32_t cooldownMs;     ///< after Cleared, a re-activation at <= the last severity is not announced again
};

struct LandingSafetyConfig {
    // ---- ExcessiveDescent (navigation vertical speed; needs valid navigation) ----
    float descentCaution_mps = 3.5f;
    float descentWarning_mps = 6.0f;
    float descentCritical_mps = 10.0f;
    float descentHysteresis_mps = 0.5f;
    float maxVelocitySigma_mps = 1.0f;      ///< navigation velocity with larger sigma is not used
    WarningPolicy descentPolicy{1000u, 500u, 2000u, 20000u};

    // ---- LowGroundClearance (LiDAR; advisory only) ----
    float lowClearance_m = 10.0f;
    float lowClearanceExit_m = 15.0f;
    /// In phase Unknown a clearance this low may simply be a device standing on the ground:
    /// LowGroundClearance is raised in Unknown only above it (matches FlightStateConfig::groundMaxClearance_m).
    float unknownPhaseMinClearance_m = 2.5f;
    WarningPolicy lowClearancePolicy{1000u, 0u, 2000u, 30000u};

    // ---- RapidGroundClosure (LiDAR only: works without navigation) ----
    float closureMaxClearance_m = 40.0f;    ///< only within LiDAR range
    float closureCaution_mps = 4.0f;
    float closureHysteresis_mps = 0.5f;
    float ttcWarning_s = 4.0f;
    float ttcCritical_s = 2.0f;
    float ttcHysteresis_s = 0.5f;
    WarningPolicy closurePolicy{300u, 200u, 1000u, 10000u};

    // ---- HazardousApproach (low clearance AND fast descent) ----
    float approachClearance_m = 15.0f;
    float approachClearanceExit_m = 20.0f;
    float approachDescentWarning_mps = 2.5f;     ///< closure rate (or -vz when the rate is unavailable)
    float approachCriticalClearance_m = 6.0f;
    float approachDescentCritical_mps = 3.5f;
    float approachDescentHysteresis_mps = 0.5f;
    WarningPolicy approachPolicy{500u, 300u, 1500u, 10000u};

    // ---- TerrainClearanceUnavailable ----
    uint32_t terrainUnavailableMs = 3000u;       ///< LiDAR unusable this long (included in raiseMs below)
    float terrainRecentClearance_m = 45.0f;      ///< "near terrain": a valid clearance below this ...
    uint32_t terrainRecentMs = 30000u;           ///< ... within this time
    float terrainDescentAdvisory_mps = 1.5f;     ///< or descending faster than this
    float terrainDescentCaution_mps = 3.5f;      ///< Caution when descending faster than this without LiDAR
    WarningPolicy terrainPolicy{0u, 1000u, 2000u, 60000u};   // raise delay = terrainUnavailableMs

    // ---- NavigationDegraded ----
    WarningPolicy navigationPolicy{2000u, 1000u, 3000u, 60000u};

    /// When the evidence a warning needs disappears (LiDAR / navigation invalid), the
    /// warning is HELD at its level this long (missing data is not "safe"), then released.
    uint32_t evidenceHoldMs = 3000u;

    bool isValid() const
    {
        return descentCaution_mps > 0.0f && descentWarning_mps > descentCaution_mps &&
               descentCritical_mps > descentWarning_mps && descentHysteresis_mps >= 0.0f &&
               descentHysteresis_mps < descentCaution_mps && lowClearanceExit_m > lowClearance_m &&
               closureCaution_mps > closureHysteresis_mps && ttcWarning_s > ttcCritical_s &&
               approachClearanceExit_m > approachClearance_m && approachCriticalClearance_m < approachClearance_m &&
               approachDescentCritical_mps > approachDescentWarning_mps &&
               approachDescentWarning_mps > approachDescentHysteresis_mps;
    }
};

/// Current state of one warning (in the snapshot).
struct WarningState {
    SafetySeverity severity;     ///< meaningful only if active
    bool active;
    bool announced;              ///< a Raised event was published for this episode
    bool degraded;               ///< decided on partial evidence
    uint32_t episodeId;
    uint32_t activeSinceMs;
};

/// The safety layer's current view, published every update (latest-value mailbox).
struct SafetySnapshot {
    uint32_t sequence;
    uint32_t timestampMs;
    FlightPhase phase;
    EvidenceLevel phaseSupport;
    uint32_t phaseSinceMs;
    uint16_t airborneEvidence;
    uint16_t groundEvidence;

    // navigation summary (as used)
    NavState navState;
    bool velocityValid;
    float verticalVelocity_mps;
    float velocitySigma_mps;

    // terrain summary
    ClearanceStatus clearanceStatus;
    bool clearanceValid;
    float clearance_m;
    float clearanceSigma_m;
    float clearanceLowerBound_m;
    float tiltDeg;
    bool closureRateValid;
    float closureRate_mps;
    bool ttcValid;
    float timeToContact_s;

    WarningState warnings[kLandingWarningCount];
    SafetySeverity highestSeverity;   ///< over active warnings (Info if none)
    uint8_t activeWarningMask;        ///< bit i = LandingWarning i active
    uint32_t qualityFlags;            ///< safetyq::*

    // lifetime counters (saturating)
    uint32_t eventsPublished;
    uint32_t eventsRejectedBySink;    ///< sink reported a drop
    uint32_t suppressedReactivations; ///< cooldown: re-activations not announced
    uint32_t warningsRaised;
    uint32_t lowClearanceRaised;
    uint32_t phaseTransitions;
    uint32_t takeoffs;
    uint32_t landings;
    uint32_t lidarRejectedSamples;    ///< beam geometry + attitude invalid / unsynchronised
    uint32_t clearanceStaleUpdates;   ///< updates with a stale / dropped-out LiDAR
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_LANDING_LANDING_SAFETY_TYPES_HPP
