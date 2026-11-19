/**
 ******************************************************************************
 * @file    SafetyMonitor.cpp
 * @brief   See SafetyMonitor.hpp.
 ******************************************************************************
 */
#include "Algorithms/Safety/SafetyMonitor.hpp"

#include <cmath>
#include <limits>

namespace pg {

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
}

SafetyMonitor::SafetyMonitor(const SafetyConfig& cfg)
    : configValid_(cfg.isValid()), clearance_(cfg.clearance), flight_(cfg.flight), landing_(cfg.landing)
{
    snap_.phase = FlightPhase::Unknown;
    snap_.clearanceStatus = ClearanceStatus::NoLidar;
    snap_.clearance_m = snap_.clearanceSigma_m = snap_.clearanceLowerBound_m = kNaN;
    snap_.closureRate_mps = snap_.timeToContact_s = snap_.verticalVelocity_mps = snap_.velocitySigma_mps = kNaN;
    snap_.tiltDeg = kNaN;
}

const SafetySnapshot& SafetyMonitor::update(const SafetyInput& in)
{
    if (snap_.sequence != UINT32_MAX) ++snap_.sequence;
    snap_.timestampMs = in.nowMs;

    // 1-2. Terrain clearance, then flight phase (which uses the clearance of this frame).
    const GroundClearance& gc = clearance_.update(in);
    const FlightStateStatus& fs = flight_.update(in, gc);

    // Snapshot summaries + quality flags (used by the analyzer's event prototypes).
    snap_.phase = fs.phase;
    snap_.phaseSupport = fs.support;
    snap_.phaseSinceMs = fs.phaseSinceMs;
    snap_.airborneEvidence = fs.airborneEvidence;
    snap_.groundEvidence = fs.groundEvidence;
    snap_.phaseTransitions = fs.transitions;
    snap_.takeoffs = fs.takeoffs;
    snap_.landings = fs.landings;
    snap_.lidarRejectedSamples = gc.rejectedGeometry + gc.rejectedAttitude;
    snap_.clearanceStaleUpdates = gc.staleUpdates;
    snap_.navState = in.nav.state;
    snap_.velocityValid = in.nav.velocityValid;
    snap_.verticalVelocity_mps = in.nav.velocityValid ? in.nav.verticalVelocity_mps : kNaN;
    snap_.velocitySigma_mps = in.nav.velocityValid ? in.nav.velocitySigma_mps : kNaN;
    snap_.clearanceStatus = gc.status;
    snap_.clearanceValid = gc.clearanceValid;
    snap_.clearance_m = gc.clearance_m;
    snap_.clearanceSigma_m = gc.clearanceSigma_m;
    snap_.clearanceLowerBound_m = gc.lowerBound_m;
    snap_.tiltDeg = gc.tiltDeg;
    snap_.closureRateValid = gc.closureRateValid;
    snap_.closureRate_mps = gc.closureRate_mps;
    snap_.ttcValid = gc.ttcValid;
    snap_.timeToContact_s = gc.timeToContact_s;

    uint32_t q = safetyq::Unvalidated;
    if (!in.nav.velocityValid || !in.nav.altitudeValid) q |= safetyq::NavigationInvalid;
    if (in.nav.state == NavState::Degraded || in.nav.state == NavState::Recovering) q |= safetyq::NavigationDegraded;
    if ((in.nav.qualityFlags & navflag::ModelPlaceholder) != 0u) q |= safetyq::ModelPlaceholder;
    if (!gc.clearanceValid && gc.status != ClearanceStatus::BeyondRange) q |= safetyq::LidarUnavailable;
    if ((gc.flags & clearflag::MountingUnverified) != 0u) q |= safetyq::LidarMountingUnverified;
    if ((gc.flags & clearflag::ImuMountingUnverified) != 0u) q |= safetyq::ImuMountingUnverified;
    if ((gc.flags & clearflag::FlatTerrainAssumed) != 0u) q |= safetyq::FlatTerrainAssumed;
    if ((gc.flags & clearflag::StepChange) != 0u) q |= safetyq::ClearanceStepChange;
    if (fs.phase == FlightPhase::Unknown || fs.phase == FlightPhase::TakeoffCandidate ||
        fs.phase == FlightPhase::LandingCandidate) {
        q |= safetyq::PhaseUncertain;
    }
    snap_.qualityFlags = q;

    // 4a. Phase-change notice (before the warnings it may release, so logs read in order).
    if (fs.phase != lastPhase_) {
        SafetyEvent e{};
        e.timestampMs = in.nowMs;
        e.type = SafetyEventType::FlightPhaseChanged;
        e.severity = SafetySeverity::Info;
        e.previousSeverity = SafetySeverity::Info;
        e.transition = SafetyTransition::Notice;
        e.source = SafetySource::FlightState;
        e.phase = fs.phase;
        e.qualityFlags = q;
        e.verticalVelocity_mps = snap_.verticalVelocity_mps;
        e.clearance_m = gc.clearance_m;
        e.closureRate_mps = gc.closureRate_mps;
        e.timeToContact_s = gc.timeToContact_s;
        emitter_.emit(e);
        lastPhase_ = fs.phase;
    }

    // 3. Warnings.
    landing_.update(in, fs, gc, emitter_, snap_);
    snap_.eventsPublished = emitter_.published();
    snap_.eventsRejectedBySink = emitter_.rejectedBySinks();
    return snap_;
}

}  // namespace pg
