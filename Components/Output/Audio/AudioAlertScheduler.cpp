/**
 ******************************************************************************
 * @file    AudioAlertScheduler.cpp
 * @brief   See AudioAlertScheduler.hpp.
 ******************************************************************************
 */
#include "Output/Audio/AudioAlertScheduler.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
void inc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}
bool reached(uint32_t now, uint32_t t) { return timebase::diffMs(now, t) >= 0; }

// tone, on, gap, beeps, repeat, volume. Design placeholders: distinct rhythms per urgency.
const AudioPatternSpec kSpecs[static_cast<uint8_t>(AudioPatternId::Count)] = {
    {AlertTone::None, 0u, 0u, 0u, 0u, 0u},                  // None
    {AlertTone::Advisory, 120u, 0u, 1u, 0u, 60u},           // AdvisoryChime
    {AlertTone::Caution, 150u, 150u, 2u, 10000u, 75u},      // CautionDouble
    {AlertTone::Warning, 150u, 120u, 3u, 3000u, 90u},       // WarningTriple
    {AlertTone::Critical, 120u, 120u, 1u, 240u, 100u},      // CriticalFast
    {AlertTone::Emergency, 700u, 300u, 1u, 1000u, 100u},    // CrashAlarm
    {AlertTone::Countdown, 100u, 0u, 1u, 1000u, 100u},      // CountdownTick
    {AlertTone::Countdown, 100u, 0u, 1u, 500u, 100u},       // CountdownFinal
    {AlertTone::Emergency, 800u, 0u, 1u, 10000u, 90u},      // SosConfirm
    {AlertTone::CommFailure, 200u, 150u, 3u, 5000u, 100u},  // CommFailure
    {AlertTone::Advisory, 100u, 100u, 2u, 0u, 60u},         // InfoChime
};
}  // namespace

const AudioPatternSpec& audioPatternSpec(AudioPatternId id)
{
    const uint8_t i = static_cast<uint8_t>(id);
    return kSpecs[i < static_cast<uint8_t>(AudioPatternId::Count) ? i : 0u];
}

AlertPriority audioPriorityFor(uint8_t p)
{
    if (p >= 90u) return AlertPriority::Critical;
    if (p >= 60u) return AlertPriority::High;
    if (p >= 40u) return AlertPriority::Normal;
    return AlertPriority::Low;
}

void AudioAlertScheduler::setDesired(AudioPatternId pattern, uint8_t priority, uint32_t key)
{
    desired_ = pattern;
    desiredPriority_ = priority;
    desiredKey_ = key;
}

void AudioAlertScheduler::stopCurrent()
{
    if (backend_ != nullptr && current_ != AudioPatternId::None) backend_->cancel(audioPatternSpec(current_).tone);
}

void AudioAlertScheduler::update(uint32_t nowMs)
{
    if (desired_ != current_ || desiredKey_ != currentKey_) {
        if (current_ != AudioPatternId::None && desired_ != AudioPatternId::None && desiredPriority_ > currentPriority_) {
            inc(preemptions_);
        }
        stopCurrent();
        current_ = desired_;
        currentPriority_ = desiredPriority_;
        currentKey_ = desiredKey_;
        step_ = 0u;
        done_ = false;
        beepsPlayed_ = 0u;
        nextMs_ = nowMs;
        groupStartMs_ = nowMs;
    }
    if (current_ == AudioPatternId::None) {
        state_ = backend_ == nullptr ? OutputState::Unavailable : OutputState::Idle;
        return;
    }
    if (backend_ == nullptr) {
        state_ = OutputState::Unavailable;
        return;
    }
    if (done_) {
        state_ = OutputState::Idle;   // a play-once pattern finished; it stays silent until a new key
        return;
    }
    if (!reached(nowMs, nextMs_)) return;

    const AudioPatternSpec& s = audioPatternSpec(current_);
    if (step_ < s.beeps) {
        const AlertRequest r{s.tone, audioPriorityFor(currentPriority_), s.onMs, s.volumePercent};
        if (backend_->request(r) != Status::Ok) {
            inc(failures_);
            state_ = OutputState::Unavailable;
            nextMs_ = nowMs + kRetryUnavailableMs;   // retry later; never busy-loop
            return;
        }
        inc(requests_);
        ++step_;
        if (beepsPlayed_ != UINT32_MAX) ++beepsPlayed_;
        state_ = OutputState::Active;
        nextMs_ = nowMs + s.onMs + s.gapMs;
        if (step_ < s.beeps) return;
    }
    // Group complete.
    if (s.repeatMs == 0u) {
        if (reached(nowMs, nextMs_)) {
            done_ = true;
            state_ = OutputState::Idle;
        }
        return;
    }
    if (step_ >= s.beeps) {
        uint32_t next = groupStartMs_ + s.repeatMs;
        if (timebase::diffMs(next, nextMs_) < 0) next = nextMs_;   // never overlap the last beep
        if (timebase::diffMs(next, nowMs) < 0) next = nowMs;       // late: no burst of missed groups
        groupStartMs_ = next;
        nextMs_ = next;
        step_ = 0u;
    }
}

}  // namespace pg
