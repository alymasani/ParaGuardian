/**
 ******************************************************************************
 * @file    AudioAlertScheduler.hpp
 * @brief   Portable, non-blocking audio pattern sequencer on top of
 *          IAudioAlert (future MAX98357A I2S backend).
 *
 * The dispatcher says WHICH pattern should sound (and its priority); this
 * class turns it into timed IAudioAlert::request() calls (one per beep, each
 * with its duration), advanced by update(now) from the emergency task. It
 * never waits: a beep is "start a tone of N ms" and the backend plays it.
 *
 * PREEMPTION: a different pattern (or a new episode of the same one) cancels
 * the current tone immediately and starts the new pattern from its first
 * beep. When a higher-priority pattern ends, the lower one RESTARTS from its
 * beginning (a half-finished beep group carries no meaning).
 * SCHEDULER GAPS: late updates play the next beep when due; missed beeps are
 * not replayed in a burst. Times are wrap-safe.
 * NO HARDWARE: with a null backend, or one that refuses requests (no I2S
 * configured), the state is Unavailable and requests are retried every
 * second; nothing claims a sound was produced.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_OUTPUT_AUDIO_ALERT_SCHEDULER_HPP
#define PARAGUARDIAN_OUTPUT_AUDIO_ALERT_SCHEDULER_HPP

#include <cstdint>

#include "Algorithms/Safety/AlertTypes.hpp"
#include "Output/Audio/IAudioAlert.hpp"

namespace pg {

struct AudioPatternSpec {
    AlertTone tone;
    uint16_t onMs;       ///< beep length
    uint16_t gapMs;      ///< silence after each beep
    uint8_t beeps;       ///< beeps per group
    uint32_t repeatMs;   ///< group period (start to start); 0 = play once
    uint8_t volumePercent;
};

/// The single timing table (AlertTypes.hpp lists the ids).
const AudioPatternSpec& audioPatternSpec(AudioPatternId id);

/// Maps an alert priority (0..100) onto the backend's coarse AlertPriority.
AlertPriority audioPriorityFor(uint8_t alertPriority);

class AudioAlertScheduler {
public:
    static constexpr uint32_t kRetryUnavailableMs = 1000u;

    explicit AudioAlertScheduler(IAudioAlert* backend) : backend_(backend) {}

    /// @param key identifies the alert episode: a new key restarts the pattern.
    void setDesired(AudioPatternId pattern, uint8_t priority, uint32_t key);
    void update(uint32_t nowMs);

    OutputState state() const { return state_; }
    AudioPatternId current() const { return current_; }
    uint32_t requests() const { return requests_; }
    uint32_t failures() const { return failures_; }
    uint32_t preemptions() const { return preemptions_; }
    uint32_t beepsInPattern() const { return beepsPlayed_; }

private:
    void stopCurrent();

    IAudioAlert* backend_;
    AudioPatternId desired_ = AudioPatternId::None;
    uint8_t desiredPriority_ = 0u;
    uint32_t desiredKey_ = 0u;
    AudioPatternId current_ = AudioPatternId::None;
    uint8_t currentPriority_ = 0u;
    uint32_t currentKey_ = 0u;
    bool done_ = false;
    uint8_t step_ = 0u;
    uint32_t nextMs_ = 0u;
    uint32_t groupStartMs_ = 0u;
    uint32_t beepsPlayed_ = 0u;
    OutputState state_ = OutputState::Idle;
    uint32_t requests_ = 0u;
    uint32_t failures_ = 0u;
    uint32_t preemptions_ = 0u;
};

/// Firmware default: no I2S / SAI configured (pins unknown). Refuses every request.
class UnconfiguredAudioAlert final : public IAudioAlert {
public:
    Status request(const AlertRequest&) override { return Status::NotInitialized; }
    void cancel(AlertTone) override {}
    bool isPlaying() const override { return false; }
};

}  // namespace pg

#endif  // PARAGUARDIAN_OUTPUT_AUDIO_ALERT_SCHEDULER_HPP
