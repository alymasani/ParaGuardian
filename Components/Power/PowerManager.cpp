/**
 ******************************************************************************
 * @file    PowerManager.cpp
 * @brief   See PowerManager.hpp.
 ******************************************************************************
 */
#include "Power/PowerManager.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
uint32_t age(uint32_t since, uint32_t now) { return timebase::ageMs(since, now); }
void inc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}
uint16_t levelThreshold(BatteryLevel l, const BatteryConfig& c)
{
    return l == BatteryLevel::Critical ? c.criticalMv : c.lowMv;
}
}  // namespace

// ---------------------------------------------------------------------------
void PowerRail::start(uint32_t nowMs)
{
    sinceMs_ = lastWantedMs_ = nowMs;
    if (sw_ == nullptr || !sw_->configured()) {
        state_ = RailState::Unconfigured;
        return;
    }
    // Explicit safe default at start-up (never "whatever the pin happened to be").
    if (!isOk(sw_->set(cfg_.defaultOn))) {
        state_ = RailState::Fault;
        inc(faults_);
        return;
    }
    state_ = cfg_.defaultOn ? RailState::Starting : RailState::Off;
}

void PowerRail::update(bool wanted, uint32_t nowMs)
{
    if (state_ == RailState::Unconfigured) return;
    if (wanted) lastWantedMs_ = nowMs;
    const bool keep = wanted || (state_ != RailState::Off && age(lastWantedMs_, nowMs) < cfg_.offDelayMs);
    switch (state_) {
        case RailState::Off:
            if (keep) {
                if (isOk(sw_->set(true))) {
                    state_ = RailState::Starting;
                    inc(switches_);
                } else {
                    state_ = RailState::Fault;
                    inc(faults_);
                }
                sinceMs_ = nowMs;
            }
            break;
        case RailState::Starting:
            if (!keep) {
                sw_->set(false);
                state_ = RailState::Off;
                sinceMs_ = nowMs;
            } else if (age(sinceMs_, nowMs) >= cfg_.settleMs) {
                if (sw_->powerGood() == Signal::Inactive) {   // feedback says it did not come up
                    sw_->set(false);
                    state_ = RailState::Fault;
                    inc(faults_);
                } else {
                    state_ = RailState::Ready;
                }
                sinceMs_ = nowMs;
            }
            break;
        case RailState::Ready:
            if (!keep) {
                sw_->set(false);
                state_ = RailState::Off;
                inc(switches_);
                sinceMs_ = nowMs;
            } else if (sw_->powerGood() == Signal::Inactive) {
                sw_->set(false);
                state_ = RailState::Fault;
                inc(faults_);
                sinceMs_ = nowMs;
            }
            break;
        case RailState::Fault:
            if (keep && age(sinceMs_, nowMs) >= cfg_.faultRetryMs) {
                state_ = RailState::Off;   // retried on the next update
                sinceMs_ = nowMs;
            }
            break;
        case RailState::Unconfigured:
        default:
            break;
    }
}

void PowerRail::forceOff(uint32_t nowMs)
{
    if (state_ != RailState::Starting && state_ != RailState::Ready) return;
    sw_->set(false);
    state_ = RailState::Off;
    sinceMs_ = nowMs;
    lastWantedMs_ = nowMs - cfg_.offDelayMs;   // no residual hold
}

// ---------------------------------------------------------------------------
PowerManager::PowerManager(const BatteryConfig& battery, const PowerPolicyConfig& policy, IPowerSwitch* rail5v,
                           IPowerSwitch* amplifier, const RailConfig& railCfg, const RailConfig& ampCfg)
    : battery_(battery),
      policy_(policy),
      rail_(rail5v, railCfg),
      amp_(amplifier, RailConfig{ampCfg.settleMs, policy.ampHoldMs, ampCfg.faultRetryMs, false})
{
    snap_.stateOfChargePercent = 255u;
}

void PowerManager::start(uint32_t nowMs)
{
    rail_.start(nowMs);
    amp_.start(nowMs);   // amplifier always starts disabled (no pop, no idle current)
    snap_.rail5v = rail_.state();
    snap_.amplifier = amp_.state();
}

void PowerManager::updateLevel(const PowerInputs& in, uint32_t nowMs)
{
    snap_.flags = 0u;
    if (battery_.lowMv == 0u || battery_.criticalMv == 0u) {
        snap_.flags |= powerflag::ThresholdsUnconfigured;
        snap_.level = BatteryLevel::Unknown;
        snap_.millivolts = in.batteryValid ? in.batteryMillivolts : 0u;
        return;
    }
    if (!in.batteryValid || in.batteryMillivolts == 0u) {
        if (!haveSample_ || age(lastValidMs_, nowMs) > battery_.maxSampleAgeMs) {
            snap_.flags |= powerflag::StaleMeasurement;
            if (snap_.level != BatteryLevel::Unknown) inc(snap_.levelChanges);
            snap_.level = BatteryLevel::Unknown;   // missing data is never "healthy"
            snap_.millivolts = 0u;
            candidateCount_ = 0u;
        }
        return;
    }
    haveSample_ = true;
    lastValidMs_ = nowMs;
    snap_.millivolts = in.batteryMillivolts;
    const uint16_t mv = in.batteryMillivolts;
    BatteryLevel raw = mv <= battery_.criticalMv ? BatteryLevel::Critical
                       : mv <= battery_.lowMv    ? BatteryLevel::Low
                                                 : BatteryLevel::Healthy;
    const BatteryLevel cur = snap_.level;
    BatteryLevel target = raw;
    const bool better = cur != BatteryLevel::Unknown && static_cast<uint8_t>(raw) < static_cast<uint8_t>(cur);
    if (better) {
        // Upward moves need the hysteresis margin, and are not taken from a charger-lifted voltage.
        if (snap_.charge == ChargeState::Charging) {
            snap_.flags |= powerflag::LevelHeldWhileCharging;
            target = cur;
        } else if (mv < static_cast<uint32_t>(levelThreshold(cur, battery_)) + battery_.hysteresisMv) {
            target = cur;
        }
    }
    if (target == cur) {
        candidateCount_ = 0u;
        return;
    }
    if (target != candidate_) {
        candidate_ = target;
        candidateCount_ = 0u;
    }
    if (++candidateCount_ >= battery_.confirmSamples) {
        snap_.level = target;
        inc(snap_.levelChanges);
        candidateCount_ = 0u;
    }
}

void PowerManager::updateCharge(const PowerInputs& in)
{
    if (in.charging == Signal::Active && in.chargeComplete == Signal::Active) {
        snap_.charge = ChargeState::Fault;
        snap_.flags |= powerflag::ChargerConflict;
    } else if (in.charging == Signal::Active) {
        snap_.charge = ChargeState::Charging;
    } else if (in.chargeComplete == Signal::Active) {
        snap_.charge = ChargeState::Complete;
    } else if (in.charging == Signal::Inactive && in.chargeComplete == Signal::Inactive) {
        snap_.charge = ChargeState::NotCharging;
    } else {
        snap_.charge = ChargeState::Unknown;
    }
    if (in.supplyPresent == Signal::Active) {
        snap_.supply = SupplyState::Present;
    } else if (in.supplyPresent == Signal::Inactive) {
        snap_.supply = SupplyState::Disconnected;
    } else {
        snap_.supply = (snap_.charge == ChargeState::Charging || snap_.charge == ChargeState::Complete)
                           ? SupplyState::Present
                           : SupplyState::Unknown;
    }
}

const PowerSnapshot& PowerManager::update(const PowerInputs& in, const PowerContext& ctx, uint32_t nowMs)
{
    if (snap_.sequence != UINT32_MAX) ++snap_.sequence;
    snap_.timestampMs = nowMs;
    updateLevel(in, nowMs);   // clears flags first
    updateCharge(in);
    snap_.stateOfChargePercent = (in.gaugeValid && in.gaugePercent <= 100u) ? in.gaugePercent : 255u;

    bool lidar = policy_.lidarOnByDefault;
    if (lidar && policy_.allowLidarShedOnGround && snap_.level == BatteryLevel::Critical && !ctx.inFlight &&
        !ctx.emergencyActive) {
        lidar = false;
        snap_.flags |= powerflag::LidarShed;
    }
    if (snap_.level == BatteryLevel::Critical && !ctx.emergencyActive) snap_.flags |= powerflag::AudioRestricted;
    snap_.lidarPowerRequested = lidar;

    if (ctx.audioRequested) {
        audioSeen_ = true;
        lastAudioMs_ = nowMs;
    }
    // An open emergency keeps the rail up (audio path ready for the SOS patterns).
    const bool railWanted = lidar || ctx.audioRequested || ctx.emergencyActive ||
                            (audioSeen_ && age(lastAudioMs_, nowMs) < policy_.ampHoldMs);
    rail_.update(railWanted, nowMs);
    if (rail_.ready()) {
        amp_.update(ctx.audioRequested, nowMs);
    } else {
        amp_.forceOff(nowMs);   // rail gone: the amplifier cannot stay enabled
    }
    snap_.rail5v = rail_.state();
    snap_.amplifier = amp_.state();
    snap_.audioReady = rail_.ready() && amp_.ready();
    snap_.railFaults = rail_.faults() + amp_.faults();
    return snap_;
}

bool PowerManager::audioAllowed(uint8_t alertPriority, bool emergencyActive) const
{
    if (snap_.level == BatteryLevel::Critical && !emergencyActive) {
        return alertPriority >= policy_.criticalAudioMinPriority;
    }
    return true;
}

}  // namespace pg
