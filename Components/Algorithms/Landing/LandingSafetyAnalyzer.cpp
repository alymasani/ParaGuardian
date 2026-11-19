/**
 ******************************************************************************
 * @file    LandingSafetyAnalyzer.cpp
 * @brief   See LandingSafetyAnalyzer.hpp and FLIGHT_LANDING_DESIGN.md section 8.
 ******************************************************************************
 */
#include "Algorithms/Landing/LandingSafetyAnalyzer.hpp"

#include <cmath>

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {

constexpr uint8_t lv(SafetySeverity s) { return static_cast<uint8_t>(s); }
constexpr uint8_t kAdvisory = lv(SafetySeverity::Advisory);
constexpr uint8_t kCaution = lv(SafetySeverity::Caution);
constexpr uint8_t kWarning = lv(SafetySeverity::Warning);
constexpr uint8_t kCritical = lv(SafetySeverity::Critical);

void satInc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}

/// Highest level whose threshold x exceeds; a level already reached uses (threshold - hysteresis).
uint8_t graded(float x, const float* thresholds, const uint8_t* levels, int n, float hyst, uint8_t current)
{
    uint8_t out = 0u;
    for (int i = 0; i < n; ++i) {
        const float t = (current >= levels[i]) ? thresholds[i] - hyst : thresholds[i];
        if (x > t) out = levels[i];
    }
    return out;
}

SafetyEventType typeOf(LandingWarning w)
{
    switch (w) {
        case LandingWarning::ExcessiveDescent:            return SafetyEventType::ExcessiveDescent;
        case LandingWarning::LowGroundClearance:          return SafetyEventType::LowGroundClearance;
        case LandingWarning::RapidGroundClosure:          return SafetyEventType::RapidGroundClosure;
        case LandingWarning::HazardousApproach:           return SafetyEventType::HazardousApproach;
        case LandingWarning::TerrainClearanceUnavailable: return SafetyEventType::TerrainClearanceUnavailable;
        default:                                          return SafetyEventType::NavigationDegraded;
    }
}

SafetySource sourceOf(LandingWarning w)
{
    return w == LandingWarning::NavigationDegraded ? SafetySource::Navigation : SafetySource::LandingSafety;
}

}  // namespace

LandingSafetyAnalyzer::LandingSafetyAnalyzer(const LandingSafetyConfig& cfg) : cfg_(cfg), configValid_(cfg.isValid())
{
    reset();
}

void LandingSafetyAnalyzer::reset()
{
    for (Tracker& t : t_) t = Tracker{};
    lidarUnusable_ = false;
    haveRecentClearance_ = false;
}

uint8_t LandingSafetyAnalyzer::held(Tracker& t, bool known, uint8_t level, uint32_t now)
{
    if (known) {
        t.holding = false;
        return level;
    }
    if (!t.active) return 0u;
    if (!t.holding) {
        t.holding = true;
        t.holdSince = now;
    }
    t.degraded = true;
    return (timebase::ageMs(t.holdSince, now) < cfg_.evidenceHoldMs) ? t.level : 0u;
}

void LandingSafetyAnalyzer::step(LandingWarning w, uint8_t level, bool degraded, const WarningPolicy& p, uint32_t now,
                                 SafetyEventEmitter& em, const SafetyEvent& proto, SafetySnapshot& snap)
{
    Tracker& t = t_[static_cast<uint8_t>(w)];
    SafetyEvent e = proto;
    e.type = typeOf(w);
    e.source = sourceOf(w);
    auto publish = [&](SafetyTransition tr, uint8_t sev, uint8_t prev) {
        e.transition = tr;
        e.severity = static_cast<SafetySeverity>(sev);
        e.previousSeverity = static_cast<SafetySeverity>(prev);
        e.episodeId = t.episode;
        e.suppressedCount = t.suppressed;
        if (t.degraded) e.qualityFlags |= safetyq::DegradedEvidence;
        if (t.suppressed != 0u) e.qualityFlags |= safetyq::CooldownSuppressed;
        em.emit(e);
    };

    if (!t.active) {
        t.escalating = t.relaxing = false;
        if (level == 0u) {
            t.pending = false;
        } else {
            if (!t.pending) {
                t.pending = true;
                t.pendingSince = now;
            }
            const uint32_t need = (level == kCritical) ? p.raiseMs / 2u : p.raiseMs;   // critical: faster
            if (timebase::ageMs(t.pendingSince, now) >= need) {
                t.pending = false;
                t.active = true;
                t.level = level;
                t.degraded = degraded;
                t.activeSince = now;
                const bool inCooldown = t.cooldown && timebase::ageMs(t.clearedAt, now) < p.cooldownMs &&
                                        level <= t.clearedLevel && level != kCritical;
                if (inCooldown) {
                    // Same hazard coming back soon at no higher severity: tracked, not re-announced.
                    t.announced = false;
                    if (t.suppressed != UINT16_MAX) ++t.suppressed;
                    satInc(snap.suppressedReactivations);
                } else {
                    t.episode = em.newEpisode();
                    t.suppressed = 0u;
                    t.announced = true;
                    satInc(snap.warningsRaised);
                    if (w == LandingWarning::LowGroundClearance) satInc(snap.lowClearanceRaised);
                    publish(SafetyTransition::Raised, level, 0u);
                }
            }
        }
    } else {
        t.degraded = t.degraded || degraded;
        if (level > t.level) {
            t.relaxing = false;
            if (!t.escalating) {
                t.escalating = true;
                t.escalateSince = now;
            }
            const uint32_t need = (level == kCritical) ? 0u : p.escalateMs;
            if (timebase::ageMs(t.escalateSince, now) >= need) {
                t.escalating = false;
                const uint8_t prev = t.level;
                t.level = level;
                if (t.announced) {
                    publish(SafetyTransition::Escalated, level, prev);
                } else {
                    // A silently re-activated warning got worse: announce it now (cooldown never hides escalation).
                    t.episode = em.newEpisode();
                    t.announced = true;
                    satInc(snap.warningsRaised);
                    publish(SafetyTransition::Raised, level, 0u);
                }
            }
        } else if (level < t.level) {
            t.escalating = false;
            if (!t.relaxing) {
                t.relaxing = true;
                t.relaxSince = now;
            }
            if (timebase::ageMs(t.relaxSince, now) >= p.relaxMs) {
                t.relaxing = false;
                const uint8_t prev = t.level;
                if (level == 0u) {
                    if (t.announced) publish(SafetyTransition::Cleared, prev, prev);
                    t.active = false;
                    t.holding = false;
                    t.cooldown = true;
                    t.clearedAt = now;
                    t.clearedLevel = t.announced ? prev : (prev > t.clearedLevel ? prev : t.clearedLevel);
                } else {
                    t.level = level;
                    if (t.announced) publish(SafetyTransition::Deescalated, level, prev);
                }
            }
        } else {
            t.escalating = t.relaxing = false;
        }
    }

    WarningState& ws = snap.warnings[static_cast<uint8_t>(w)];
    ws.active = t.active;
    ws.severity = t.active ? static_cast<SafetySeverity>(t.level) : SafetySeverity::Info;
    ws.announced = t.active && t.announced;
    ws.degraded = t.active && t.degraded;
    ws.episodeId = t.episode;
    ws.activeSinceMs = t.active ? t.activeSince : 0u;
}

void LandingSafetyAnalyzer::update(const SafetyInput& in, const FlightStateStatus& fs, const GroundClearance& gc,
                                   SafetyEventEmitter& em, SafetySnapshot& snap)
{
    const uint32_t now = in.nowMs;
    const FlightPhase ph = fs.phase;
    const bool ground = isGroundPhase(ph);
    const bool airborne = isAirbornePhase(ph);
    const bool airOrUnknown = airborne || ph == FlightPhase::Unknown;

    const SafetyNavInput& N = in.nav;
    const bool navFresh = timebase::ageMs(N.timestampMs, now) <= 500u && !timebase::isAfter(N.timestampMs, now);
    const bool navOk = configValid_ && navFresh && N.velocityValid && std::isfinite(N.verticalVelocity_mps) &&
                       N.velocitySigma_mps <= cfg_.maxVelocitySigma_mps;
    const bool navDegraded = N.state == NavState::Degraded || N.state == NavState::Recovering;
    const float descent = navOk ? -N.verticalVelocity_mps : 0.0f;   // positive = going down

    const bool clrOk = gc.clearanceValid;
    const bool clrHigh = gc.status == ClearanceStatus::BeyondRange;   // known: far from terrain
    const bool clrKnown = clrOk || clrHigh;
    if (clrOk && gc.clearance_m < cfg_.terrainRecentClearance_m) {
        haveRecentClearance_ = true;
        lastLowClearanceAt_ = now;
    }
    const bool recentLow = haveRecentClearance_ && timebase::ageMs(lastLowClearanceAt_, now) <= cfg_.terrainRecentMs;
    const bool lidarUnusable = !clrKnown;
    if (lidarUnusable && !lidarUnusable_) lidarUnusableSince_ = now;
    lidarUnusable_ = lidarUnusable;

    // Event prototype (values at the decision).
    SafetyEvent proto{};
    proto.timestampMs = now;
    proto.phase = ph;
    proto.qualityFlags = snap.qualityFlags;
    proto.verticalVelocity_mps = navOk ? N.verticalVelocity_mps : NAN;
    proto.clearance_m = clrOk ? gc.clearance_m : NAN;
    proto.closureRate_mps = gc.closureRateValid ? gc.closureRate_mps : NAN;
    proto.timeToContact_s = gc.ttcValid ? gc.timeToContact_s : NAN;

    if (!configValid_) return;

    // ---- ExcessiveDescent (navigation) ----
    {
        Tracker& t = t_[static_cast<uint8_t>(LandingWarning::ExcessiveDescent)];
        static const uint8_t L[3] = {kCaution, kWarning, kCritical};
        const float T[3] = {cfg_.descentCaution_mps, cfg_.descentWarning_mps, cfg_.descentCritical_mps};
        uint8_t level = 0u;
        if (!ground) level = held(t, navOk, navOk ? graded(descent, T, L, 3, cfg_.descentHysteresis_mps, t.active ? t.level : 0u) : 0u, now);
        step(LandingWarning::ExcessiveDescent, level, navDegraded, cfg_.descentPolicy, now, em, proto, snap);
    }
    // ---- LowGroundClearance (LiDAR, advisory) ----
    {
        Tracker& t = t_[static_cast<uint8_t>(LandingWarning::LowGroundClearance)];
        uint8_t level = 0u;
        const bool phaseOk = ph == FlightPhase::Flying || ph == FlightPhase::Approach || ph == FlightPhase::Unknown;
        if (phaseOk) {
            const float limit = t.active ? cfg_.lowClearanceExit_m : cfg_.lowClearance_m;
            const bool maybeStanding = ph == FlightPhase::Unknown && clrOk && gc.clearance_m <= cfg_.unknownPhaseMinClearance_m;
            level = held(t, clrKnown, (clrOk && gc.clearance_m < limit && !maybeStanding) ? kAdvisory : 0u, now);
        }
        step(LandingWarning::LowGroundClearance, level, false, cfg_.lowClearancePolicy, now, em, proto, snap);
    }
    // ---- RapidGroundClosure (LiDAR only) ----
    {
        Tracker& t = t_[static_cast<uint8_t>(LandingWarning::RapidGroundClosure)];
        uint8_t level = 0u;
        if (airOrUnknown) {
            const bool known = clrHigh || (clrOk && gc.closureRateValid);
            uint8_t want = 0u;
            if (clrOk && gc.closureRateValid && gc.clearance_m <= cfg_.closureMaxClearance_m) {
                const uint8_t cur = t.active ? t.level : 0u;
                const float cautionT = (cur >= kCaution) ? cfg_.closureCaution_mps - cfg_.closureHysteresis_mps
                                                         : cfg_.closureCaution_mps;
                if (gc.closureRate_mps > cautionT) want = kCaution;
                if (gc.ttcValid) {
                    const float wT = (cur >= kWarning) ? cfg_.ttcWarning_s + cfg_.ttcHysteresis_s : cfg_.ttcWarning_s;
                    const float cT = (cur >= kCritical) ? cfg_.ttcCritical_s + cfg_.ttcHysteresis_s : cfg_.ttcCritical_s;
                    if (gc.timeToContact_s < wT && want >= kCaution) want = kWarning;
                    if (gc.timeToContact_s < cT && want >= kCaution) want = kCritical;
                }
            }
            level = held(t, known, want, now);
        }
        step(LandingWarning::RapidGroundClosure, level, false, cfg_.closurePolicy, now, em, proto, snap);
    }
    // ---- HazardousApproach (low clearance AND fast descent / closure) ----
    {
        Tracker& t = t_[static_cast<uint8_t>(LandingWarning::HazardousApproach)];
        uint8_t level = 0u;
        bool degraded = false;
        if (airOrUnknown) {
            float down = NAN;
            if (gc.closureRateValid) {
                down = gc.closureRate_mps;
            } else if (navOk) {
                down = descent;      // terrain may not be flat / static: partial evidence
                degraded = true;
            }
            const bool known = clrHigh || (clrOk && std::isfinite(down));
            uint8_t want = 0u;
            if (clrOk && std::isfinite(down)) {
                const uint8_t cur = t.active ? t.level : 0u;
                const float clrW = (cur >= kWarning) ? cfg_.approachClearanceExit_m : cfg_.approachClearance_m;
                const float dW = (cur >= kWarning) ? cfg_.approachDescentWarning_mps - cfg_.approachDescentHysteresis_mps
                                                   : cfg_.approachDescentWarning_mps;
                const float clrC = (cur >= kCritical) ? cfg_.approachCriticalClearance_m + 2.0f : cfg_.approachCriticalClearance_m;
                const float dC = (cur >= kCritical) ? cfg_.approachDescentCritical_mps - cfg_.approachDescentHysteresis_mps
                                                    : cfg_.approachDescentCritical_mps;
                if (gc.clearance_m < clrW && down > dW) want = kWarning;
                if (gc.clearance_m < clrC && down > dC) want = kCritical;
            }
            level = held(t, known, want, now);
        }
        step(LandingWarning::HazardousApproach, level, degraded, cfg_.approachPolicy, now, em, proto, snap);
    }
    // ---- TerrainClearanceUnavailable ----
    {
        uint8_t level = 0u;
        if (airborne && lidarUnusable && timebase::ageMs(lidarUnusableSince_, now) >= cfg_.terrainUnavailableMs) {
            const bool nearTerrain = recentLow || ph == FlightPhase::Approach ||
                                     (navOk && descent > cfg_.terrainDescentAdvisory_mps);
            if (nearTerrain) level = kAdvisory;
            if (navOk && descent > cfg_.terrainDescentCaution_mps) level = kCaution;
        }
        step(LandingWarning::TerrainClearanceUnavailable, level, false, cfg_.terrainPolicy, now, em, proto, snap);
    }
    // ---- NavigationDegraded ----
    {
        uint8_t level = 0u;
        if (airborne) {
            // Degraded state, or a velocity too uncertain to be used for descent warnings
            // (the consumer must know WHY ExcessiveDescent cannot be confirmed).
            const bool vzUnusable = !(N.velocityValid && N.velocitySigma_mps <= cfg_.maxVelocitySigma_mps);
            if (N.state == NavState::Degraded || N.state == NavState::Recovering || vzUnusable) level = kAdvisory;
            if (N.state == NavState::Unavailable || N.state == NavState::NumericalFault ||
                N.state == NavState::Initializing || N.state == NavState::Uninitialized || !navFresh) {
                level = kCaution;
            }
        }
        step(LandingWarning::NavigationDegraded, level, false, cfg_.navigationPolicy, now, em, proto, snap);
    }

    // ---- summary ----
    snap.activeWarningMask = 0u;
    snap.highestSeverity = SafetySeverity::Info;
    for (uint8_t i = 0; i < kLandingWarningCount; ++i) {
        if (!snap.warnings[i].active) continue;
        snap.activeWarningMask = static_cast<uint8_t>(snap.activeWarningMask | (1u << i));
        if (snap.warnings[i].severity > snap.highestSeverity) snap.highestSeverity = snap.warnings[i].severity;
    }
}

}  // namespace pg
