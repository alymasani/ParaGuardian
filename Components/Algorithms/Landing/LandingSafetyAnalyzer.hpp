/**
 ******************************************************************************
 * @file    LandingSafetyAnalyzer.hpp
 * @brief   Turns flight phase + navigation + LiDAR clearance into warning
 *          states and SafetyEvents (transitions only).
 *
 * Each warning has a desired LEVEL per update (0 = none, else a
 * SafetySeverity), computed with hysteresis (exit thresholds differ from
 * entry thresholds). A per-warning tracker then applies:
 *   raiseMs    persistence before Raised          (no single-sample alarms)
 *   escalateMs persistence before Escalated       (escalation ALWAYS announced,
 *                                                  even during a cooldown)
 *   relaxMs    persistence before Deescalated / Cleared
 *   cooldownMs after Cleared, a re-activation at a severity <= the cleared
 *              one is tracked (snapshot shows it active) but NOT announced
 *              again; anything more severe is announced at once.
 * Missing evidence is not "safe": a warning whose input disappears is HELD
 * at its level for evidenceHoldMs, then released (Cleared, flagged degraded).
 * In Ground / Landed every warning is released (no post-landing alarms).
 *
 * Which warnings need which evidence:
 *   ExcessiveDescent           navigation vz (valid, sigma <= max); not LiDAR
 *   LowGroundClearance         LiDAR clearance only
 *   RapidGroundClosure         LiDAR clearance + closure rate / TTC only (works without navigation)
 *   HazardousApproach          LiDAR clearance + (LiDAR closure, or navigation -vz: "degraded evidence")
 *   TerrainClearanceUnavailable LiDAR status + phase / recent clearance / navigation vz
 *   NavigationDegraded         navigation state only
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_LANDING_LANDING_SAFETY_ANALYZER_HPP
#define PARAGUARDIAN_ALGORITHMS_LANDING_LANDING_SAFETY_ANALYZER_HPP

#include <cstdint>

#include "Algorithms/Flight/FlightStateTypes.hpp"
#include "Algorithms/Landing/GroundClearanceTypes.hpp"
#include "Algorithms/Landing/LandingSafetyTypes.hpp"
#include "Algorithms/Safety/SafetyEventEmitter.hpp"
#include "Algorithms/Safety/SafetyInput.hpp"

namespace pg {

class LandingSafetyAnalyzer {
public:
    explicit LandingSafetyAnalyzer(const LandingSafetyConfig& cfg = LandingSafetyConfig{});

    bool configValid() const { return configValid_; }
    /// Updates every warning; publishes transitions through @p emitter; fills
    /// the warning part of @p snap (warnings[], mask, highest severity, counters).
    void update(const SafetyInput& in, const FlightStateStatus& fs, const GroundClearance& gc,
                SafetyEventEmitter& emitter, SafetySnapshot& snap);
    void reset();

private:
    struct Tracker {
        bool active;
        uint8_t level;
        bool announced;
        bool degraded;
        uint32_t episode;
        uint32_t activeSince;
        bool pending;      uint32_t pendingSince;
        bool escalating;   uint32_t escalateSince;
        bool relaxing;     uint32_t relaxSince;
        bool cooldown;     uint32_t clearedAt;   uint8_t clearedLevel;
        uint16_t suppressed;
        bool holding;      uint32_t holdSince;
    };

    /// Desired level with evidence-hold: if @p known is false, keep the active level for evidenceHoldMs.
    uint8_t held(Tracker& t, bool known, uint8_t level, uint32_t now);
    void step(LandingWarning w, uint8_t level, bool degraded, const WarningPolicy& p, uint32_t now,
              SafetyEventEmitter& em, const SafetyEvent& proto, SafetySnapshot& snap);

    LandingSafetyConfig cfg_;
    bool configValid_;
    Tracker t_[kLandingWarningCount] = {};
    bool lidarUnusable_ = false;
    uint32_t lidarUnusableSince_ = 0u;
    bool haveRecentClearance_ = false;
    uint32_t lastLowClearanceAt_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_LANDING_LANDING_SAFETY_ANALYZER_HPP
