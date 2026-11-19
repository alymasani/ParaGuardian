/**
 ******************************************************************************
 * @file    FlightStateDetector.cpp
 * @brief   See FlightStateDetector.hpp.
 ******************************************************************************
 */
#include "Algorithms/Flight/FlightStateDetector.hpp"

#include <cmath>
#include <limits>

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr uint32_t kSteadyPeriodMs = 500u;

void satInc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}

unsigned bits(uint16_t v)
{
    unsigned n = 0u;
    for (; v != 0u; v = static_cast<uint16_t>(v & (v - 1u))) ++n;
    return n;
}

bool fresh(uint32_t ts, uint32_t now, uint32_t maxAge) { return timebase::ageMs(ts, now) <= maxAge && !timebase::isAfter(ts, now); }
}  // namespace

FlightStateDetector::FlightStateDetector(const FlightStateConfig& cfg) : cfg_(cfg), configValid_(cfg.isValid()) { reset(); }

void FlightStateDetector::reset()
{
    out_ = FlightStateStatus{};
    out_.phase = FlightPhase::Unknown;
    out_.previousPhase = FlightPhase::Unknown;
    out_.groundReferenceAltitude_m = kNaN;
    out_.motionAccelRms_mps2 = kNaN;
    out_.motionRateRms_radps = kNaN;
    out_.flags = flightflag::StartedUnknown;
    msAccel_ = msRate_ = 0.0f;
    motionAccelInit_ = motionRateInit_ = false;
    accelObservedMs_ = rateObservedMs_ = 0u;
    steadyCount_ = 0u;
    steadyHavePush_ = false;
    vzActive_ = multiActive_ = singleActive_ = groundActive_ = false;
    startAirActive_ = startGndActive_ = approachLostActive_ = false;
    started_ = false;
}

uint32_t FlightStateDetector::persist(bool cond, bool& active, uint32_t& since, uint32_t nowMs)
{
    if (!cond) {
        active = false;
        return 0u;
    }
    if (!active) {
        active = true;
        since = nowMs;
    }
    const uint32_t d = timebase::ageMs(since, nowMs);
    return d == 0u ? 1u : d;   // 1 ms minimum so "true now" is distinguishable from "false"
}

bool FlightStateDetector::dwellOk(uint32_t nowMs) const
{
    return timebase::ageMs(out_.phaseSinceMs, nowMs) >= cfg_.minPhaseDwellMs;
}

void FlightStateDetector::enter(FlightPhase p, uint32_t nowMs)
{
    if (p == out_.phase) return;
    out_.previousPhase = out_.phase;
    out_.phase = p;
    out_.phaseSinceMs = nowMs;
    satInc(out_.transitions);
    multiActive_ = singleActive_ = groundActive_ = approachLostActive_ = false;
    lastAirborneEvidenceMs_ = nowMs;
    lastGroundEvidenceMs_ = nowMs;
}

void FlightStateDetector::updateMotion(const SafetyInput& in)
{
    const float tau = cfg_.motionTau_s;
    // Linear acceleration magnitude (gravity removed by the attitude estimator).
    const SafetyAttitudeInput& A = in.attitude;
    if (A.linearAccelValid && fresh(A.linearAccelTimestampMs, in.nowMs, cfg_.maxInputAgeMs) &&
        std::isfinite(A.linearAccelNorm_mps2)) {
        if (!motionAccelInit_) {
            msAccel_ = A.linearAccelNorm_mps2 * A.linearAccelNorm_mps2;
            motionAccelInit_ = true;
            accelObservedMs_ = 0u;
            lastAccelTs_ = A.linearAccelTimestampMs;
        } else if (timebase::isAfter(A.linearAccelTimestampMs, lastAccelTs_)) {
            const uint32_t dtMs = timebase::ageMs(lastAccelTs_, A.linearAccelTimestampMs);
            const float dt = static_cast<float>(dtMs < 200u ? dtMs : 200u) * 1e-3f;
            const float alpha = 1.0f - std::exp(-dt / tau);
            msAccel_ += alpha * (A.linearAccelNorm_mps2 * A.linearAccelNorm_mps2 - msAccel_);
            accelObservedMs_ += dtMs;
            lastAccelTs_ = A.linearAccelTimestampMs;
        }
    } else {
        motionAccelInit_ = false;
    }
    const SafetyImuInput& I = in.imu;
    if (I.valid && fresh(I.timestampMs, in.nowMs, cfg_.maxInputAgeMs) && std::isfinite(I.rateNorm_radps)) {
        if (!motionRateInit_) {
            msRate_ = I.rateNorm_radps * I.rateNorm_radps;
            motionRateInit_ = true;
            rateObservedMs_ = 0u;
            lastRateTs_ = I.timestampMs;
        } else if (timebase::isAfter(I.timestampMs, lastRateTs_)) {
            const uint32_t dtMs = timebase::ageMs(lastRateTs_, I.timestampMs);
            const float dt = static_cast<float>(dtMs < 200u ? dtMs : 200u) * 1e-3f;
            const float alpha = 1.0f - std::exp(-dt / tau);
            msRate_ += alpha * (I.rateNorm_radps * I.rateNorm_radps - msRate_);
            rateObservedMs_ += dtMs;
            lastRateTs_ = I.timestampMs;
        }
    } else {
        motionRateInit_ = false;
    }
    const uint32_t warm = static_cast<uint32_t>(tau * 1000.0f);
    out_.motionAccelRms_mps2 = (motionAccelInit_ && accelObservedMs_ >= warm) ? std::sqrt(msAccel_) : kNaN;
    out_.motionRateRms_radps = (motionRateInit_ && rateObservedMs_ >= warm) ? std::sqrt(msRate_) : kNaN;
}

void FlightStateDetector::updateSteadyWindow(const SafetyInput& in, bool navOk)
{
    if (!navOk) {
        steadyCount_ = 0u;
        steadyHavePush_ = false;
        return;
    }
    if (steadyHavePush_ && timebase::ageMs(lastSteadyPushMs_, in.nowMs) < kSteadyPeriodMs) return;
    if (steadyCount_ == kSteadySlots) {
        for (uint32_t i = 1; i < kSteadySlots; ++i) steadyAlt_[i - 1u] = steadyAlt_[i];
        --steadyCount_;
    }
    steadyAlt_[steadyCount_++] = in.nav.altitude_m;
    lastSteadyPushMs_ = in.nowMs;
    steadyHavePush_ = true;
}

const FlightStateStatus& FlightStateDetector::update(const SafetyInput& in, const GroundClearance& clr)
{
    const uint32_t now = in.nowMs;
    if (!started_) {
        started_ = true;
        out_.phaseSinceMs = now;
        lastAirborneEvidenceMs_ = lastGroundEvidenceMs_ = now;
    }
    out_.flags &= flightflag::StartedUnknown;
    if (out_.phase != FlightPhase::Unknown) out_.flags = 0u;
    if (!configValid_) {
        out_.flags |= flightflag::PhaseHeld;
        return out_;
    }

    // ---------------- evidence ----------------
    const SafetyNavInput& N = in.nav;
    const bool navFresh = fresh(N.timestampMs, now, cfg_.maxInputAgeMs);
    const bool navOk = navFresh && N.altitudeValid && N.velocityValid && std::isfinite(N.altitude_m) &&
                       std::isfinite(N.verticalVelocity_mps) && N.velocitySigma_mps <= cfg_.maxNavVelocitySigma_mps;
    if (!navOk) out_.flags |= flightflag::NavigationUnavailable;
    if (N.state == NavState::Degraded || N.state == NavState::Recovering) out_.flags |= flightflag::NavigationDegraded;
    if ((N.qualityFlags & navflag::ModelPlaceholder) != 0u) out_.flags |= flightflag::ModelPlaceholder;

    uint16_t air = 0u, gnd = 0u;
    // LiDAR (strongest evidence; only with usable geometry)
    const bool lidarUsable = clr.clearanceValid || (clr.status == ClearanceStatus::BeyondRange && std::isfinite(clr.lowerBound_m));
    if (!lidarUsable) out_.flags |= flightflag::LidarUnavailable;
    if (clr.clearanceValid) {
        if (clr.clearance_m > cfg_.groundMaxClearance_m) air |= flightev::LidarHigh;
        else gnd |= flightev::LidarNearGround;
    } else if (clr.status == ClearanceStatus::BeyondRange && std::isfinite(clr.lowerBound_m) &&
               clr.lowerBound_m > cfg_.groundMaxClearance_m) {
        air |= flightev::LidarHigh;
    }
    // GNSS ground speed
    const SafetyGnssInput& G = in.gnss;
    const bool gnssOk = G.valid && fresh(G.timestampMs, now, 2000u) &&
                        (G.fixType == GnssFixType::Fix3D || G.fixType == GnssFixType::GnssPlusDeadReckoning) &&
                        std::isfinite(G.groundSpeed_mps) && G.speedAccuracy_mps <= cfg_.gnssMaxSpeedAccuracy_mps;
    if (!gnssOk) out_.flags |= flightflag::GnssUnavailable;
    if (gnssOk && G.groundSpeed_mps > cfg_.airborneGroundSpeed_mps) air |= flightev::GnssFast;
    if (gnssOk && G.groundSpeed_mps < cfg_.groundGroundSpeed_mps) gnd |= flightev::GnssSlow;
    // Navigation: sustained vertical motion, altitude departure, steady altitude
    const uint32_t vzFor =
        persist(navOk && std::fabs(N.verticalVelocity_mps) > cfg_.verticalMotion_mps, vzActive_, vzSince_, now);
    if (vzFor >= cfg_.verticalMotionMs) air |= flightev::VerticalMotion;
    if (navOk && std::fabs(N.verticalVelocity_mps) < cfg_.groundVz_mps) gnd |= flightev::LowSpeedVertical;
    updateSteadyWindow(in, navOk);
    if (steadyCount_ == kSteadySlots && navOk) {
        // Judged over the whole window (spread AND net change), not from one noisy vz sample:
        // a single-sample test flickers with 0.1 m/s velocity noise and keeps restarting the dwell timers.
        float lo = steadyAlt_[0], hi = steadyAlt_[0];
        for (uint32_t i = 1; i < steadyCount_; ++i) {
            lo = std::fmin(lo, steadyAlt_[i]);
            hi = std::fmax(hi, steadyAlt_[i]);
        }
        const float windowS = static_cast<float>((kSteadySlots - 1u) * kSteadyPeriodMs) * 1e-3f;
        const float netRate = std::fabs(steadyAlt_[steadyCount_ - 1u] - steadyAlt_[0]) / windowS;
        if (hi - lo <= 2.0f * cfg_.steadyAltitudeBand_m && netRate < cfg_.groundVz_mps) gnd |= flightev::AltitudeSteady;
    }
    // Ground reference: follows the device while on the ground with no airborne evidence;
    // frozen from the takeoff candidate on (so departure is measured from the launch).
    const bool onGroundPhase = isGroundPhase(out_.phase) || out_.phase == FlightPhase::Unknown;
    if (navOk && onGroundPhase && (air & ~flightev::AltitudeDeparture) == 0u) {
        out_.groundReferenceAltitude_m = N.altitude_m;
        out_.groundReferenceValid = true;
    }
    if (navOk && out_.groundReferenceValid && !onGroundPhase &&
        std::fabs(N.altitude_m - out_.groundReferenceAltitude_m) > cfg_.departureAltitude_m) {
        air |= flightev::AltitudeDeparture;
    }
    // IMU stillness
    updateMotion(in);
    const bool imuOk = std::isfinite(out_.motionAccelRms_mps2) && std::isfinite(out_.motionRateRms_radps);
    if (!imuOk) out_.flags |= flightflag::ImuUnavailable;
    if (imuOk && out_.motionAccelRms_mps2 < cfg_.stillAccel_mps2 && out_.motionRateRms_radps < cfg_.stillRate_radps) {
        gnd |= flightev::Still;
    }

    out_.airborneEvidence = air;
    out_.groundEvidence = gnd;

    // Counted evidence (LowSpeedVertical is informative only: it is part of AltitudeSteady).
    const uint16_t gndCounted = static_cast<uint16_t>(gnd & ~flightev::LowSpeedVertical);
    const uint16_t airNoDeparture = static_cast<uint16_t>(air & ~flightev::AltitudeDeparture);
    const unsigned nAir = bits(air);
    const unsigned nAirNoDep = bits(airNoDeparture);
    const unsigned nGnd = bits(gndCounted);
    const bool strongAir = (air & flightev::LidarHigh) != 0u;
    const bool lidarVeto = (gnd & flightev::LidarNearGround) != 0u;

    // No usable input at all: hold the phase, freeze the timers (never "lands" on an outage).
    const bool anyInput = navOk || lidarUsable || gnssOk || imuOk;
    if (!anyInput) {
        out_.flags |= flightflag::PhaseHeld;
        satInc(out_.heldUpdates);
        multiActive_ = singleActive_ = groundActive_ = startAirActive_ = startGndActive_ = false;
        lastAirborneEvidenceMs_ = lastGroundEvidenceMs_ = now;
        out_.support = EvidenceLevel::None;
        out_.timeInPhaseMs = timebase::ageMs(out_.phaseSinceMs, now);
        return out_;
    }
    if (nAirNoDep > 0u) lastAirborneEvidenceMs_ = now;
    if (nGnd >= 2u) lastGroundEvidenceMs_ = now;
    if (lidarVeto && nAirNoDep > 0u) out_.flags |= flightflag::TakeoffVetoedByLidar;

    // ---------------- transitions ----------------
    const bool takeoffMulti = !lidarVeto && (strongAir || nAir >= 2u);
    const bool takeoffSingle = !lidarVeto && nAirNoDep >= 1u;
    const bool groundLike = nGnd >= 2u && nAirNoDep == 0u;

    switch (out_.phase) {
        case FlightPhase::Unknown: {
            const uint32_t a = persist(strongAir || nAirNoDep >= 2u, startAirActive_, startAirSince_, now);
            const uint32_t g = persist(groundLike, startGndActive_, startGndSince_, now);
            // One airborne source only (e.g. navigation alone): same long dwell as a single-source takeoff.
            const uint32_t s = persist(nAirNoDep >= 1u && !lidarVeto, singleActive_, singleSince_, now);
            if (a >= cfg_.startupDecisionMs || s >= cfg_.takeoffSingleSourceMs) enter(FlightPhase::Flying, now);   // started in flight
            else if (g >= cfg_.startupDecisionMs) enter(FlightPhase::Ground, now);
            break;
        }
        case FlightPhase::Ground:
        case FlightPhase::Landed:
            if (nAirNoDep >= 1u && dwellOk(now)) {
                groundPhaseBeforeCandidate_ = out_.phase;
                enter(FlightPhase::TakeoffCandidate, now);
            }
            break;
        case FlightPhase::TakeoffCandidate: {
            const uint32_t m = persist(takeoffMulti, multiActive_, multiSince_, now);
            const uint32_t s = persist(takeoffSingle, singleActive_, singleSince_, now);
            if (m >= cfg_.takeoffConfirmMs || s >= cfg_.takeoffSingleSourceMs) {
                enter(FlightPhase::Flying, now);
                satInc(out_.takeoffs);
            } else if (timebase::ageMs(lastAirborneEvidenceMs_, now) >= cfg_.takeoffAbortMs) {
                enter(groundPhaseBeforeCandidate_, now);
                satInc(out_.abortedTakeoffs);
            }
            break;
        }
        case FlightPhase::Flying:
        case FlightPhase::Approach: {
            if (out_.phase == FlightPhase::Flying) {
                if (clr.clearanceValid && clr.clearance_m < cfg_.approachClearance_m && dwellOk(now)) {
                    enter(FlightPhase::Approach, now);
                    break;
                }
            } else {
                const bool lost = !(clr.clearanceValid && clr.clearance_m <= cfg_.approachExitClearance_m);
                const bool high = (clr.clearanceValid && clr.clearance_m > cfg_.approachExitClearance_m) ||
                                  clr.status == ClearanceStatus::BeyondRange;
                const uint32_t l = persist(lost, approachLostActive_, approachLostSince_, now);
                if ((high && dwellOk(now)) || l >= cfg_.approachExitMs) {
                    enter(FlightPhase::Flying, now);
                    break;
                }
            }
            const uint32_t g = persist(groundLike, groundActive_, groundSince_, now);
            if (g >= cfg_.landingCandidateMs && dwellOk(now)) {
                airPhaseBeforeCandidate_ = out_.phase;
                enter(FlightPhase::LandingCandidate, now);
            }
            break;
        }
        case FlightPhase::LandingCandidate:
            if (nAirNoDep >= 1u || timebase::ageMs(lastGroundEvidenceMs_, now) >= cfg_.takeoffAbortMs) {
                enter(airPhaseBeforeCandidate_, now);   // ground evidence did not hold
            } else if (timebase::ageMs(out_.phaseSinceMs, now) >= cfg_.landingConfirmMs) {
                enter(FlightPhase::Landed, now);
                satInc(out_.landings);
                if (navOk) {
                    out_.groundReferenceAltitude_m = N.altitude_m;
                    out_.groundReferenceValid = true;
                }
            }
            break;
    }
    if (out_.phase != FlightPhase::Unknown) out_.flags &= ~static_cast<uint32_t>(flightflag::StartedUnknown);

    // Support for the phase now held (heuristic level, not a probability).
    unsigned n = 0u;
    bool strong = false;
    if (isAirbornePhase(out_.phase) || out_.phase == FlightPhase::TakeoffCandidate) {
        n = nAir;
        strong = strongAir;
    } else if (isGroundPhase(out_.phase)) {
        n = nGnd;
        strong = lidarVeto;
    } else {
        n = (nAir > nGnd) ? nAir : nGnd;
    }
    out_.support = (strong || n >= 3u) ? EvidenceLevel::Strong
                   : (n == 2u)         ? EvidenceLevel::Moderate
                   : (n == 1u)         ? EvidenceLevel::Weak
                                       : EvidenceLevel::None;
    out_.timeInPhaseMs = timebase::ageMs(out_.phaseSinceMs, now);
    return out_;
}

}  // namespace pg
