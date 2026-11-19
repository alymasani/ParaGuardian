/**
 ******************************************************************************
 * @file    CrashDetector.cpp
 * @brief   See CrashDetector.hpp and Docs/CRASH_EMERGENCY_DESIGN.md section 6.
 ******************************************************************************
 */
#include "Algorithms/Crash/CrashDetector.hpp"

#include <cmath>
#include <limits>

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr uint32_t kBaseFlags = crashflag::Unvalidated | crashflag::SampleRateLimited;
/// Peaks closer than this belong to the same impulse (contiguous frames of one pulse).
constexpr uint32_t kSameImpulseMs = 300u;
/// A GNSS speed older than this is not used as evidence.
constexpr uint32_t kGnssMaxAgeMs = 2000u;

void inc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}
uint32_t elapsed(uint32_t now, uint32_t since) { return timebase::ageMs(since, now); }
bool finite(float v) { return std::isfinite(v); }
uint32_t addSat(uint32_t a, uint32_t b) { return (UINT32_MAX - a < b) ? UINT32_MAX : a + b; }
uint8_t popcount(uint32_t v)
{
    uint8_t n = 0u;
    for (; v != 0u; v &= v - 1u) ++n;
    return n;
}
/// Sorts up to 3 values descending (in place).
void sortDesc(float* v, uint8_t n)
{
    for (uint8_t i = 0; i + 1u < n; ++i)
        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < n; ++j)
            if (v[j] > v[i]) {
                const float t = v[i];
                v[i] = v[j];
                v[j] = t;
            }
}
}  // namespace

CrashDetector::CrashDetector(const CrashConfig& cfg) : cfg_(cfg), configValid_(cfg.isValid())
{
    st_.state = CrashState::Monitoring;
    st_.flags = kBaseFlags;
    st_.impact.peakAccel_mps2 = st_.impact.peakRate_radps = st_.impact.maxChannelAccel_mps2 = kNaN;
    st_.impact.baselineAccel_mps2 = st_.impact.vzBefore_mps = st_.impact.clearanceBefore_m = kNaN;
    st_.impact.groundSpeedBefore_mps = kNaN;
}

// ---------------------------------------------------------------------------
CrashDetector::Impulse CrashDetector::readImpulse(const CrashInput& in)
{
    Impulse r{};
    float a[kImuCount] = {};
    float w[kImuCount] = {};
    uint8_t n = 0u, satA = 0u, satW = 0u;
    float best = -1.0f;
    for (size_t i = 0; i < kImuCount; ++i) {
        const CrashImuChannel& c = in.imu[i];
        if (!c.present || c.samples == 0u) continue;
        // Each IMU sample is evaluated once: a peak not newer than the last one is a replay.
        if (peakSeen_[i] && timebase::diffMs(c.peakTimestampMs, lastPeakMs_[i]) <= 0) {
            inc(st_.staleInputs);
            continue;
        }
        if (!finite(c.peakAccel_mps2) || !finite(c.peakRate_radps) || c.peakAccel_mps2 < 0.0f ||
            c.peakRate_radps < 0.0f) {
            continue;   // never turned into a valid "0"
        }
        peakSeen_[i] = true;
        lastPeakMs_[i] = c.peakTimestampMs;
        a[n] = c.peakAccel_mps2;
        w[n] = c.peakRate_radps;
        if (c.peakAccel_mps2 > best) {
            best = c.peakAccel_mps2;
            r.timestampMs = c.peakTimestampMs;
        }
        if (c.accelSaturated) ++satA;
        if (c.gyroSaturated) ++satW;
        ++n;
    }
    r.channels = n;
    r.any = n > 0u;
    if (!r.any) return r;
    sortDesc(a, n);
    sortDesc(w, n);
    r.maxAccel = a[0];
    // Corroboration: the SECOND-largest IMU. With 3 IMUs one faulty sensor can neither
    // create nor hide an impulse; with 2 the smaller is used (conservative).
    r.accel = n >= 2u ? a[1] : a[0];
    r.rate = n >= 2u ? w[1] : w[0];
    r.saturated = n >= 2u ? (satA >= 2u || satW >= 2u) : (satA + satW >= 1u);
    r.uncorroborated = n >= 2u && ((a[0] >= cfg_.impactAccel_mps2 && a[1] < cfg_.impactAccel_mps2) ||
                                   (w[0] >= cfg_.impactRate_radps && w[1] < cfg_.impactRate_radps));
    return r;
}

bool CrashDetector::isImpulse(const Impulse& imp) const
{
    if (!imp.any) return false;
    const float base = baselineValid_ ? baseline_ : CrashConfig::kG;
    const float rateBase = baselineValid_ ? rateBaseline_ : 0.0f;
    const bool accelImpulse = imp.accel >= cfg_.impactAccel_mps2 && (imp.accel - base) >= cfg_.impulseMin_mps2;
    const bool rateImpulse = imp.rate >= cfg_.impactRate_radps && (imp.rate - rateBase) >= cfg_.rateImpulseMin_radps;
    return accelImpulse || rateImpulse || imp.saturated;
}

ImpactClass CrashDetector::classify(const Impulse& imp) const
{
    if (!isImpulse(imp)) return ImpactClass::None;
    const float rateBase = baselineValid_ ? rateBaseline_ : 0.0f;
    const bool rateImpulse = imp.rate >= cfg_.impactRate_radps && (imp.rate - rateBase) >= cfg_.rateImpulseMin_radps;
    const bool strong = imp.accel >= cfg_.severeAccel_mps2 || (imp.rate >= cfg_.severeRate_radps && rateImpulse) ||
                        imp.saturated || (imp.accel >= cfg_.impactAccel_mps2 && rateImpulse);
    // A single IMU is never "strong": nothing corroborates it.
    return (strong && imp.channels >= 2u) ? ImpactClass::Strong : ImpactClass::Moderate;
}

// ---------------------------------------------------------------------------
void CrashDetector::recordHistory(const CrashInput& in)
{
    const uint32_t now = in.nowMs;
    Slot* s = &hist_[head_];
    if (!s->used || elapsed(now, s->startMs) >= cfg_.historySlotMs) {
        if (s->used && elapsed(now, s->startMs) >= cfg_.historySlotMs * kHistorySlots) {
            // Outage longer than the whole history: nothing in it describes "just before".
            for (Slot& x : hist_) x.used = false;
            st_.flags |= crashflag::HistoryGap;
        }
        if (s->used) head_ = static_cast<uint8_t>((head_ + 1u) % kHistorySlots);
        s = &hist_[head_];
        *s = Slot{};
        s->used = true;
        s->startMs = now;
    }
    s->airborne = s->airborne || isAirbornePhase(in.phase);
    s->takeoffCandidate = s->takeoffCandidate || in.phase == FlightPhase::TakeoffCandidate;
    if (in.vzValid && finite(in.vz_mps)) {
        s->minVz = s->vzValid ? (in.vz_mps < s->minVz ? in.vz_mps : s->minVz) : in.vz_mps;
        s->vzValid = true;
    }
    if (in.clearanceValid && finite(in.clearance_m)) {
        s->minClearance = s->clearanceValid ? (in.clearance_m < s->minClearance ? in.clearance_m : s->minClearance)
                                            : in.clearance_m;
        s->clearanceValid = true;
    }
    if (in.closureValid && finite(in.closureRate_mps)) {
        s->maxClosure = s->closureValid ? (in.closureRate_mps > s->maxClosure ? in.closureRate_mps : s->maxClosure)
                                        : in.closureRate_mps;
        s->closureValid = true;
    }
    if (in.gnssValid && finite(in.groundSpeed_mps) && elapsed(now, in.gnssTimestampMs) <= kGnssMaxAgeMs) {
        s->maxSpeed = s->speedValid ? (in.groundSpeed_mps > s->maxSpeed ? in.groundSpeed_mps : s->maxSpeed)
                                    : in.groundSpeed_mps;
        s->speedValid = true;
    }
}

void CrashDetector::evaluatePreContext(uint32_t t0, uint32_t nowMs)
{
    bool air = false, takeoff = false, vzOk = false, clrOk = false, cloOk = false, spdOk = false;
    float minVz = 0.0f, minClr = 0.0f, maxClo = 0.0f, maxSpd = 0.0f;
    for (const Slot& s : hist_) {
        if (!s.used) continue;
        const int32_t d = timebase::diffMs(t0, s.startMs);   // slot start before the impact by d ms
        if (d < -static_cast<int32_t>(cfg_.historySlotMs) ||
            d > static_cast<int32_t>(cfg_.preWindowMs + cfg_.historySlotMs)) {
            continue;
        }
        air = air || s.airborne;
        takeoff = takeoff || s.takeoffCandidate;
        if (s.vzValid) {
            minVz = vzOk ? (s.minVz < minVz ? s.minVz : minVz) : s.minVz;
            vzOk = true;
        }
        if (s.clearanceValid) {
            minClr = clrOk ? (s.minClearance < minClr ? s.minClearance : minClr) : s.minClearance;
            clrOk = true;
        }
        if (s.closureValid) {
            maxClo = cloOk ? (s.maxClosure > maxClo ? s.maxClosure : maxClo) : s.maxClosure;
            cloOk = true;
        }
        if (s.speedValid) {
            maxSpd = spdOk ? (s.maxSpeed > maxSpd ? s.maxSpeed : maxSpd) : s.maxSpeed;
            spdOk = true;
        }
    }
    uint32_t& ev = st_.evidence;
    if (air) ev |= crashev::AirborneAtImpact;
    if (takeoff && !air) ev |= crashev::TakeoffAttempt;   // not counted twice right after a takeoff
    if (everAirborne_ && elapsed(nowMs, lastAirborneMs_) <= cfg_.recentAirborneMs) ev |= crashev::RecentlyAirborne;
    if (vzOk && minVz <= -cfg_.fastDescent_mps) ev |= crashev::FastDescent;
    if (clrOk && cloOk && minClr <= cfg_.closureClearance_m && maxClo >= cfg_.closureRate_mps)
        ev |= crashev::GroundClosure;
    if (spdOk && maxSpd >= cfg_.highGroundSpeed_mps) ev |= crashev::HighGroundSpeed;
    if (freeFallDurationMs_ >= cfg_.freeFallMinMs && timebase::diffMs(t0, freeFallEndMs_) <= static_cast<int32_t>(cfg_.freeFallGapMs) &&
        timebase::diffMs(t0, freeFallEndMs_) >= -static_cast<int32_t>(cfg_.candidateWindowMs)) {
        ev |= crashev::FreeFallBeforeImpact;
    }
    if (!vzOk) st_.flags |= crashflag::NavigationMissing;
    if (!clrOk) st_.flags |= crashflag::LidarMissing;
    if (!spdOk) st_.flags |= crashflag::GnssMissing;
    st_.impact.vzBefore_mps = vzOk ? minVz : kNaN;
    st_.impact.clearanceBefore_m = clrOk ? minClr : kNaN;
    st_.impact.groundSpeedBefore_mps = spdOk ? maxSpd : kNaN;
}

uint8_t CrashDetector::preCount() const
{
    uint32_t m = crashev::AirborneAtImpact | crashev::TakeoffAttempt | crashev::FastDescent | crashev::GroundClosure |
                 crashev::HighGroundSpeed;
    const bool airContext = (st_.evidence & (crashev::AirborneAtImpact | crashev::TakeoffAttempt)) != 0u;
    if (airContext) m |= crashev::FreeFallBeforeImpact;   // free fall supports a crash only in flight
    return popcount(st_.evidence & m);
}

uint8_t CrashDetector::postCount() const
{
    return popcount(st_.evidence & (crashev::Inactivity | crashev::AbnormalOrientation | crashev::RepeatedImpacts));
}

// ---------------------------------------------------------------------------
void CrashDetector::trackPostImpact(const CrashInput& in, const Impulse& imp, float f, float w, uint32_t dtMs)
{
    const uint32_t now = in.nowMs;
    observedTotalMs_ = addSat(observedTotalMs_, dtMs);
    const bool haveF = finite(f), haveW = finite(w);
    if (!imp.any && !in.newImuData) dataMissingMs_ = addSat(dataMissingMs_, dtMs);

    // Inactivity: quiet specific force (|f| ~ g, no gravity subtraction needed) and rotation.
    if (haveF && haveW) {
        const float dev = std::fabs(f - CrashConfig::kG);
        if (dev < cfg_.quietForce_mps2 && w < cfg_.quietRate_radps) {
            quietMs_ = addSat(quietMs_, dtMs);
            notQuietMs_ = 0u;
        } else {
            notQuietMs_ = addSat(notQuietMs_, dtMs);
            if (notQuietMs_ > cfg_.quietToleranceMs) quietMs_ = 0u;
        }
        if (dev > cfg_.activity_mps2) activityMs_ = addSat(activityMs_, dtMs);
    }   // no data: neither quiet nor active (missing data is not inactivity)

    // Orientation far from upright while not turning (a banked turn is not "lying down").
    if (in.attitudeValid && finite(in.tilt_rad) && haveW) {
        if (in.tilt_rad >= cfg_.abnormalTilt_rad && w < cfg_.tiltMaxRate_radps) {
            tiltMs_ = addSat(tiltMs_, dtMs);
            notTiltMs_ = 0u;
        } else {
            notTiltMs_ = addSat(notTiltMs_, dtMs);
            if (notTiltMs_ > cfg_.quietToleranceMs) tiltMs_ = 0u;
        }
    } else if (!in.attitudeValid) {
        st_.flags |= crashflag::AttitudeMissing;
    }

    // Further impulses (tumbling): separated pulses are counted, contiguous frames are one pulse.
    if (isImpulse(imp)) {
        if (elapsed(now, lastImpulseMs_) >= kSameImpulseMs) {
            if (extraImpacts_ != UINT8_MAX) ++extraImpacts_;
            if (st_.impact.impacts != UINT8_MAX) ++st_.impact.impacts;
        }
        lastImpulseMs_ = now;
        if (imp.accel > st_.impact.peakAccel_mps2) st_.impact.peakAccel_mps2 = imp.accel;
        if (imp.rate > st_.impact.peakRate_radps) st_.impact.peakRate_radps = imp.rate;
        if (imp.saturated) st_.evidence |= crashev::ImuSaturation;
    }

    // Resumed flight: fast GNSS ground speed, sustained, while the phase is airborne and
    // (if LiDAR works) clear of the ground. Walking / being dragged is slower.
    const bool gnssFresh = in.gnssValid && finite(in.groundSpeed_mps) &&
                           elapsed(now, in.gnssTimestampMs) <= kGnssMaxAgeMs;
    const bool clearOfGround = !in.clearanceValid || (finite(in.clearance_m) && in.clearance_m > 3.0f);
    if (gnssFresh && in.groundSpeed_mps >= cfg_.resumedFlightSpeed_mps && clearOfGround) {
        fastMovingMs_ = addSat(fastMovingMs_, dtMs);
    } else {
        fastMovingMs_ = 0u;
    }

    uint32_t& ev = st_.evidence;
    if (quietMs_ >= cfg_.inactivityMs) ev |= crashev::Inactivity;
    if (tiltMs_ >= cfg_.abnormalTiltMs) ev |= crashev::AbnormalOrientation;
    if (extraImpacts_ >= cfg_.repeatedImpacts) ev |= crashev::RepeatedImpacts;
    if (activityMs_ >= cfg_.activityMs) ev |= crashev::PilotActivity;
    if (fastMovingMs_ >= cfg_.resumedFlightMs && isAirbornePhase(in.phase)) ev |= crashev::ResumedFlight;
    if (observedTotalMs_ >= cfg_.minObservationMs &&
        static_cast<float>(dataMissingMs_) >= cfg_.dataLossFraction * static_cast<float>(observedTotalMs_)) {
        ev |= crashev::PostImpactDataMissing;
    } else {
        ev &= ~static_cast<uint32_t>(crashev::PostImpactDataMissing);
    }
}

// ---------------------------------------------------------------------------
void CrashDetector::enter(CrashState s, uint32_t nowMs)
{
    st_.state = s;
    st_.stateSinceMs = nowMs;
}

void CrashDetector::emit(const CrashInput& in, SafetyEventEmitter& em, SafetyEventType type, SafetyTransition tr,
                         SafetySeverity sev, SafetySeverity prev)
{
    SafetyEvent e{};
    e.timestampMs = in.nowMs;
    e.episodeId = st_.episodeId;
    e.type = type;
    e.severity = sev;
    e.previousSeverity = prev;
    e.transition = tr;
    e.source = SafetySource::CrashDetector;
    e.phase = in.phase;
    e.qualityFlags = safetyq::Unvalidated;
    if ((st_.flags & (crashflag::SingleImuOnly | crashflag::NavigationMissing | crashflag::AttitudeMissing)) != 0u ||
        (st_.evidence & crashev::PostImpactDataMissing) != 0u) {
        e.qualityFlags |= safetyq::DegradedEvidence;
    }
    if ((st_.flags & crashflag::PhaseUnknown) != 0u) e.qualityFlags |= safetyq::PhaseUncertain;
    e.verticalVelocity_mps = st_.impact.vzBefore_mps;
    e.clearance_m = st_.impact.clearanceBefore_m;
    e.closureRate_mps = kNaN;
    e.timeToContact_s = kNaN;
    em.emit(e);
}

void CrashDetector::startCandidate(const CrashInput& in, const Impulse& imp, SafetyEventEmitter& em)
{
    const uint32_t now = in.nowMs;
    st_.episodeId = em.newEpisode();
    st_.evidence = 0u;
    st_.flags = kBaseFlags;
    st_.decision = CrashDecision::None;
    // Peak timestamp (read time of that sample) unless implausible against the frame time.
    const int32_t lag = timebase::diffMs(now, imp.timestampMs);
    impactMs_ = (lag >= 0 && lag <= 1000) ? imp.timestampMs : now;
    lastImpulseMs_ = now;
    extraImpacts_ = 0u;
    quietMs_ = notQuietMs_ = tiltMs_ = notTiltMs_ = activityMs_ = fastMovingMs_ = 0u;
    dataMissingMs_ = observedTotalMs_ = 0u;

    ImpactSummary& s = st_.impact;
    s = ImpactSummary{};
    s.timestampMs = impactMs_;
    s.peakAccel_mps2 = imp.accel;
    s.peakRate_radps = imp.rate;
    s.maxChannelAccel_mps2 = imp.maxAccel;
    s.baselineAccel_mps2 = baselineValid_ ? baseline_ : kNaN;
    s.channels = imp.channels;
    s.impacts = 1u;
    s.saturated = imp.saturated;
    s.cls = classify(imp);
    strong_ = s.cls == ImpactClass::Strong;

    uint32_t& ev = st_.evidence;
    if (imp.accel >= cfg_.impactAccel_mps2) ev |= crashev::HighSpecificForce;
    if (imp.accel >= cfg_.severeAccel_mps2) ev |= crashev::SevereSpecificForce;
    if (imp.rate >= cfg_.impactRate_radps) ev |= crashev::HighAngularRate;
    if (imp.rate >= cfg_.severeRate_radps) ev |= crashev::SevereAngularRate;
    if (jumpForce_) ev |= crashev::SuddenForceChange;
    if (jumpRate_) ev |= crashev::SuddenRateChange;
    if (imp.saturated) {
        ev |= crashev::ImuSaturation;
        inc(st_.saturationEvents);
    }
    if (imp.channels < 2u) st_.flags |= crashflag::SingleImuOnly;
    if (in.phase == FlightPhase::Unknown) st_.flags |= crashflag::PhaseUnknown;
    if (gapSinceEpisode_) st_.flags |= crashflag::HistoryGap;
    gapSinceEpisode_ = false;

    evaluatePreContext(impactMs_, now);
    inc(st_.impactCandidates);
    enter(CrashState::ImpactCandidate, now);
    emit(in, em, SafetyEventType::ImpactCandidate, SafetyTransition::Raised, SafetySeverity::Advisory,
         SafetySeverity::Info);
}

void CrashDetector::suspect(const CrashInput& in, CrashDecision why, SafetyEventEmitter& em)
{
    st_.decision = why;
    inc(st_.suspicions);
    inc(st_.suspicionSerial);
    enter(CrashState::CrashSuspected, in.nowMs);
    emit(in, em, SafetyEventType::CrashSuspected, SafetyTransition::Raised, SafetySeverity::Critical,
         SafetySeverity::Info);
    emit(in, em, SafetyEventType::ImpactCandidate, SafetyTransition::Cleared, SafetySeverity::Advisory,
         SafetySeverity::Advisory);
}

void CrashDetector::reject(const CrashInput& in, CrashDecision why, SafetyEventEmitter& em)
{
    st_.decision = why;
    inc(st_.rejections);
    enter(CrashState::Rejected, in.nowMs);
    emit(in, em, SafetyEventType::ImpactCandidate, SafetyTransition::Cleared, SafetySeverity::Advisory,
         SafetySeverity::Advisory);
}

void CrashDetector::decide(const CrashInput& in, SafetyEventEmitter& em, bool windowEnd)
{
    const uint32_t ev = st_.evidence;
    if ((ev & crashev::ResumedFlight) != 0u) {
        reject(in, CrashDecision::FlightResumed, em);
        return;
    }
    const uint32_t observed = elapsed(in.nowMs, impactMs_);
    if (observed < cfg_.minObservationMs && !windowEnd) return;

    const uint8_t pre = preCount();
    const uint8_t post = postCount();
    if (strong_) {
        if (pre >= 2u) return suspect(in, CrashDecision::StrongImpactWithContext, em);
        if (pre >= 1u && post >= 1u) return suspect(in, CrashDecision::StrongImpactWithSupport, em);
        if (post >= 2u) return suspect(in, CrashDecision::ImpactWithPostSupport, em);
    } else if ((pre >= 2u && post >= 1u) || (pre >= 1u && post >= 2u)) {
        return suspect(in, CrashDecision::ImpactWithPostSupport, em);
    }
    if (!windowEnd) return;

    // End of the observation window without a rule satisfied.
    if (strong_ && (ev & crashev::AirborneAtImpact) != 0u) {
        // A strong impulse in flight after which flight never resumed. The pilot may be
        // moving (conscious, or hanging in a tree): suspicion lets the countdown ask.
        if ((ev & crashev::PostImpactDataMissing) != 0u && cfg_.suspectOnPostImpactDataLoss) {
            return suspect(in, CrashDecision::StrongImpactDataLoss, em);
        }
        return suspect(in, CrashDecision::StrongImpactFlightEnded, em);
    }
    reject(in, CrashDecision::InsufficientEvidence, em);
}

// ---------------------------------------------------------------------------
const CrashStatus& CrashDetector::update(const CrashInput& in, SafetyEventEmitter& em)
{
    const uint32_t now = in.nowMs;
    if (started_ && timebase::diffMs(now, lastNowMs_) < 0) {
        inc(st_.staleInputs);   // older than the previous frame: ignored
        return st_;
    }
    uint32_t dt = started_ ? elapsed(now, lastNowMs_) : 0u;
    if (!started_) firstUpdateMs_ = now;
    if (started_ && dt > 2u * cfg_.historySlotMs) gapSinceEpisode_ = true;
    if (dt > cfg_.observationMs) dt = cfg_.observationMs;   // one gap cannot count for more than a window
    started_ = true;
    lastNowMs_ = now;
    if (st_.sequence != UINT32_MAX) ++st_.sequence;
    st_.timestampMs = now;

    if (isAirbornePhase(in.phase)) {
        everAirborne_ = true;
        lastAirborneMs_ = now;
    }
    recordHistory(in);

    const Impulse imp = readImpulse(in);
    if (imp.any || in.newImuData) {
        lastImuDataMs_ = now;
        imuEverSeen_ = true;
    }
    if (imp.uncorroborated) inc(st_.uncorroboratedPeaks);
    // Per-frame force / rate sample: the voted value, else the corroborated peak, else none.
    const float f = in.forceValid && finite(in.forceNorm_mps2) ? in.forceNorm_mps2 : (imp.any ? imp.accel : kNaN);
    const float w = in.rateValid && finite(in.rateNorm_radps) ? in.rateNorm_radps : (imp.any ? imp.rate : kNaN);

    // Sudden changes between consecutive frames (observations for the episode).
    jumpForce_ = jumpRate_ = false;
    if (prevValid_ && finite(f) && finite(w)) {
        const float fp = imp.any ? imp.accel : f;
        jumpForce_ = std::fabs(fp - prevForce_) >= cfg_.forceJump_mps2;
        jumpRate_ = std::fabs((imp.any ? imp.rate : w) - prevRate_) >= cfg_.rateJump_radps;
    }
    // Free fall (|f| near zero): start / end times; the impact frame ends it.
    if (finite(f)) {
        if (f < cfg_.freeFall_mps2) {
            if (!inFreeFall_) {
                inFreeFall_ = true;
                freeFallStartMs_ = now;
            }
        } else if (inFreeFall_) {
            inFreeFall_ = false;
            freeFallEndMs_ = now;
            freeFallDurationMs_ = elapsed(now, freeFallStartMs_);
        }
    }

    const bool impulse = isImpulse(imp);
    switch (st_.state) {
        case CrashState::Monitoring:
        case CrashState::Unavailable:
            if (impulse) {
                startCandidate(in, imp, em);
            } else if (st_.state == CrashState::Monitoring &&
                       elapsed(now, imuEverSeen_ ? lastImuDataMs_ : firstUpdateMs_) > cfg_.imuUnavailableMs) {
                inc(st_.unavailableEpisodes);
                enter(CrashState::Unavailable, now);
            } else if (st_.state == CrashState::Unavailable && (imp.any || in.newImuData)) {
                enter(CrashState::Monitoring, now);
            }
            break;

        case CrashState::ImpactCandidate:
            if (impulse) {   // the rest of the same pulse: keep the largest values
                lastImpulseMs_ = now;
                if (imp.accel > st_.impact.peakAccel_mps2) st_.impact.peakAccel_mps2 = imp.accel;
                if (imp.rate > st_.impact.peakRate_radps) st_.impact.peakRate_radps = imp.rate;
                if (imp.maxAccel > st_.impact.maxChannelAccel_mps2) st_.impact.maxChannelAccel_mps2 = imp.maxAccel;
                if (classify(imp) == ImpactClass::Strong) {
                    strong_ = true;
                    st_.impact.cls = ImpactClass::Strong;
                }
                if (imp.saturated) {
                    st_.impact.saturated = true;
                    st_.evidence |= crashev::ImuSaturation;
                }
                if (imp.accel >= cfg_.severeAccel_mps2) st_.evidence |= crashev::SevereSpecificForce;
                if (imp.rate >= cfg_.impactRate_radps) st_.evidence |= crashev::HighAngularRate;
                if (imp.rate >= cfg_.severeRate_radps) st_.evidence |= crashev::SevereAngularRate;
            }
            if (elapsed(now, impactMs_) >= cfg_.candidateWindowMs) {
                const uint32_t ev = st_.evidence;
                const bool airContext = (ev & (crashev::AirborneAtImpact | crashev::TakeoffAttempt)) != 0u;
                const bool eligible = airContext || (ev & crashev::RecentlyAirborne) != 0u ||
                                      ((st_.flags & crashflag::PhaseUnknown) != 0u && strong_);
                if ((ev & crashev::FreeFallBeforeImpact) != 0u && !airContext) {
                    reject(in, CrashDecision::DropSignature, em);
                } else if (!eligible) {
                    reject(in, CrashDecision::NoFlightContext, em);
                } else {
                    enter(CrashState::PostImpactObservation, now);
                }
            }
            break;

        case CrashState::PostImpactObservation:
            trackPostImpact(in, imp, f, w, dt);
            decide(in, em, elapsed(now, impactMs_) >= cfg_.observationMs);
            break;

        case CrashState::CrashSuspected:
            trackPostImpact(in, imp, f, w, dt);
            if ((st_.evidence & crashev::ResumedFlight) != 0u) {
                inc(st_.suspicionsCleared);
                emit(in, em, SafetyEventType::CrashSuspected, SafetyTransition::Cleared, SafetySeverity::Critical,
                     SafetySeverity::Critical);
                enter(CrashState::Cooldown, now);
            } else if (elapsed(now, st_.stateSinceMs) >= cfg_.suspectedHoldMs) {
                enter(CrashState::Cooldown, now);   // the emergency manager latched it long ago
            }
            break;

        case CrashState::Rejected:
            if (impulse) {
                startCandidate(in, imp, em);
            } else if (elapsed(now, st_.stateSinceMs) >= cfg_.rejectedHoldMs) {
                enter(CrashState::Monitoring, now);
            }
            break;

        case CrashState::Cooldown:
            if (impulse) {   // counted against the last episode; no new suspicion
                if (elapsed(now, lastImpulseMs_) >= kSameImpulseMs && st_.impact.impacts != UINT8_MAX) {
                    ++st_.impact.impacts;
                }
                lastImpulseMs_ = now;
            }
            if (elapsed(now, st_.stateSinceMs) >= cfg_.cooldownMs) enter(CrashState::Monitoring, now);
            break;
    }

    // Baseline |f| (EMA) from non-impulse frames only: it describes the load BEFORE an impulse.
    if (finite(f) && finite(w) && !impulse) {
        if (!baselineValid_) {
            baseline_ = f;
            rateBaseline_ = w;
            baselineValid_ = true;
        } else {
            const float dts = static_cast<float>(dt) * 1e-3f;
            const float alpha = dts / (cfg_.baselineTau_s + dts);
            baseline_ += alpha * (f - baseline_);
            rateBaseline_ += alpha * (w - rateBaseline_);
        }
    }
    if (finite(f) && finite(w)) {
        prevForce_ = imp.any ? imp.accel : f;
        prevRate_ = imp.any ? imp.rate : w;
        prevValid_ = true;
    }

    const bool observing = st_.state == CrashState::PostImpactObservation || st_.state == CrashState::CrashSuspected;
    st_.observedMs = observing ? elapsed(now, impactMs_) : 0u;
    st_.preCount = preCount();
    st_.postCount = postCount();
    return st_;
}

}  // namespace pg
