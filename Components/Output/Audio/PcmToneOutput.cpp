/**
 ******************************************************************************
 * @file    PcmToneOutput.cpp
 * @brief   See PcmToneOutput.hpp.
 ******************************************************************************
 */
#include "Output/Audio/PcmToneOutput.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
void inc(uint32_t& c, uint32_t by = 1u)
{
    c = (c > UINT32_MAX - by) ? UINT32_MAX : c + by;
}
struct Guard {
    explicit Guard(ICriticalSection& l) : l_(l) { l_.enter(); }
    ~Guard() { l_.exit(); }
    ICriticalSection& l_;
};
}  // namespace

uint16_t toneFrequencyHz(AlertTone t)
{
    switch (t) {
        case AlertTone::PowerOn: return 1000u;
        case AlertTone::SensorFault: return 800u;
        case AlertTone::LowBattery: return 700u;
        case AlertTone::Warning: return 1500u;
        case AlertTone::Emergency: return 2500u;
        case AlertTone::Advisory: return 880u;
        case AlertTone::Caution: return 1100u;
        case AlertTone::Critical: return 2000u;
        case AlertTone::Countdown: return 1200u;
        case AlertTone::CommFailure: return 600u;
        case AlertTone::None:
        default: return 0u;
    }
}

PcmToneOutput::PcmToneOutput(ICriticalSection& lock, const PcmToneConfig& cfg) : lock_(lock), cfg_(cfg) {}

Status PcmToneOutput::request(const AlertRequest& r)
{
    if (!cfg_.isValid() || toneFrequencyHz(r.tone) == 0u || r.volumePercent > 100u) return Status::InvalidArgument;
    Guard g(lock_);
    inc(stats_.requests);
    if (playing_ || pending_) inc(stats_.preempted);
    pendingReq_ = r;
    pending_ = true;
    pendingSinceMs_ = 0u;   // stamped by service() (the caller's clock)
    stopRequested_ = playing_;   // current tone releases, then the new one starts
    return Status::Ok;
}

void PcmToneOutput::cancel(AlertTone tone)
{
    Guard g(lock_);
    if (pending_ && pendingReq_.tone == tone) pending_ = false;
    if (playing_ && tone_ == tone) stopRequested_ = true;
}

bool PcmToneOutput::isPlaying() const
{
    Guard g(lock_);
    return playing_;
}

bool PcmToneOutput::wantsAmplifier() const
{
    Guard g(lock_);
    return pending_ || playing_;
}

void PcmToneOutput::setAmplifierReady(bool ready, uint32_t nowMs)
{
    Guard g(lock_);
    ampReady_ = ready;
    if (pending_ && pendingSinceMs_ == 0u) pendingSinceMs_ = nowMs == 0u ? 1u : nowMs;
}

void PcmToneOutput::service(uint32_t nowMs)
{
    Guard g(lock_);
    if (!pending_) return;
    if (pendingSinceMs_ == 0u) {
        pendingSinceMs_ = nowMs == 0u ? 1u : nowMs;
        return;
    }
    if (!ampReady_ && timebase::ageMs(pendingSinceMs_, nowMs) > cfg_.maxStartDelayMs) {
        pending_ = false;   // never "played" into a powered-off amplifier
        inc(stats_.droppedNoAmplifier);
    }
}

size_t PcmToneOutput::render(int16_t* dst, size_t frames)
{
    if (dst == nullptr) return 0u;
    // 1. Control snapshot (short critical section: a few words).
    bool startNew = false;
    bool playing = false;
    bool amp = false;
    AlertRequest req{};
    {
        Guard g(lock_);
        amp = ampReady_;
        if (stopRequested_) {
            if (playing_ && releaseLeft_ == 0u) releaseLeft_ = ramp_ == 0u ? 1u : ramp_;
            stopRequested_ = false;
        }
        if (playing_ && !amp) {   // amplifier lost mid-tone: the rest would be silent -> not audible
            playing_ = false;
            tone_ = AlertTone::None;
            remaining_ = 0u;
            inc(stats_.droppedNoAmplifier);
        }
        if (pending_ && amp && !playing_) {
            req = pendingReq_;
            pending_ = false;
            startNew = true;
        }
        playing = playing_ || startNew;
    }
    if (startNew) {
        const uint32_t rate = cfg_.sampleRateHz;
        phase_ = 0u;
        phaseStep_ = static_cast<uint32_t>((static_cast<uint64_t>(toneFrequencyHz(req.tone)) << 16) / rate);
        remaining_ = req.durationMs == 0u ? UINT32_MAX : static_cast<uint32_t>(req.durationMs) * rate / 1000u;
        ramp_ = static_cast<uint32_t>(cfg_.rampMs) * rate / 1000u;
        amplitude_ = static_cast<int32_t>(static_cast<uint32_t>(cfg_.maxAmplitude) * req.volumePercent / 100u);
        played_ = 0u;
        releaseLeft_ = 0u;
        Guard g(lock_);
        playing_ = true;
        tone_ = req.tone;
        inc(stats_.tonesStarted);
    }
    // 2. Render without the lock: square wave, linear attack / release envelope (Q16).
    bool finished = false;
    for (size_t i = 0; i < frames; ++i) {
        if (!playing || finished || remaining_ == 0u) {
            dst[i] = 0;
            continue;
        }
        const int32_t sq = (phase_ & 0x8000u) != 0u ? amplitude_ : -amplitude_;
        int64_t env = 65536;
        if (ramp_ > 0u && played_ < ramp_) env = static_cast<int64_t>(played_) * 65536 / ramp_;
        if (releaseLeft_ > 0u && ramp_ > 0u) {
            const int64_t r = static_cast<int64_t>(releaseLeft_) * 65536 / ramp_;
            if (r < env) env = r;
        }
        if (remaining_ != UINT32_MAX && ramp_ > 0u && remaining_ < ramp_) {
            const int64_t r = static_cast<int64_t>(remaining_) * 65536 / ramp_;
            if (r < env) env = r;
        }
        dst[i] = static_cast<int16_t>(static_cast<int64_t>(sq) * env / 65536);
        phase_ += phaseStep_;
        ++played_;
        if (remaining_ != UINT32_MAX) --remaining_;
        if (releaseLeft_ > 0u && --releaseLeft_ == 0u) remaining_ = 0u;
        if (remaining_ == 0u) finished = true;
    }
    {
        Guard g(lock_);
        inc(stats_.framesRendered, static_cast<uint32_t>(frames));
        if (finished && playing_) {
            playing_ = false;
            tone_ = AlertTone::None;
            if (played_ >= ramp_) inc(stats_.tonesAudible);
        }
    }
    return frames;
}

PcmToneStats PcmToneOutput::stats() const
{
    Guard g(lock_);
    return stats_;
}

}  // namespace pg
