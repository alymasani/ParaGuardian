/**
 ******************************************************************************
 * @file    PcmToneOutput.hpp
 * @brief   IAudioAlert implementation that RENDERS alert tones as 16-bit PCM
 *          for an I2S amplifier (MAX98357A), with amplifier power gating.
 *
 * WHAT IS REAL: tone generation (frequency per tone, volume, click-free
 * attack / release ramps, exact durations in samples), pre-emption, the
 * request -> amplifier-power -> playback sequencing and honest statistics.
 * WHAT IS NOT CONFIGURED: the SAI/I2S peripheral, its pins, DMA channel and
 * sample-rate clock (schematic + CubeMX). Until they exist, nothing calls
 * render() from a DMA interrupt in the firmware and the firmware keeps
 * UnconfiguredAudioAlert; host tests call render() directly.
 *
 * THREADING: request()/cancel() run in the emergency task; render() runs in
 * the I2S DMA half/complete interrupt (or a host test). Shared fields are
 * exchanged under a short ICriticalSection (a few words copied).
 *
 * AMPLIFIER: a tone is only rendered audibly once setAmplifierReady(true)
 * (PowerManager: 5 V rail + amplifier enabled and settled). While not ready
 * the request waits up to maxStartDelayMs, then it is DROPPED and counted:
 * tonesAudible never counts a tone that was rendered into a powered-off amp.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_OUTPUT_AUDIO_PCM_TONE_OUTPUT_HPP
#define PARAGUARDIAN_OUTPUT_AUDIO_PCM_TONE_OUTPUT_HPP

#include <cstddef>
#include <cstdint>

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection
#include "Output/Audio/IAudioAlert.hpp"

namespace pg {

struct PcmToneConfig {
    uint32_t sampleRateHz = 16000u;    ///< MAX98357A accepts 8-96 kHz; 16 kHz is ample for beeps
    uint16_t rampMs = 2u;              ///< attack / release (avoids clicks)
    uint16_t maxAmplitude = 12000u;    ///< full-scale cap (speaker / hearing safety margin; tune on hardware)
    uint32_t maxStartDelayMs = 250u;   ///< amplifier not ready this long: the tone is dropped (reported)
    bool isValid() const { return sampleRateHz >= 8000u && sampleRateHz <= 96000u && maxAmplitude <= 32767u; }
};

/// Tone frequency table (Hz). Distinct, mid-range frequencies for small speakers.
uint16_t toneFrequencyHz(AlertTone t);

struct PcmToneStats {
    uint32_t requests;
    uint32_t tonesStarted;      ///< rendering began with the amplifier ready
    uint32_t tonesAudible;      ///< completed (or pre-empted) after >= rampMs of audible output
    uint32_t droppedNoAmplifier;
    uint32_t preempted;
    uint32_t framesRendered;
};

class PcmToneOutput final : public IAudioAlert {
public:
    PcmToneOutput(ICriticalSection& lock, const PcmToneConfig& cfg = PcmToneConfig{});

    // IAudioAlert (emergency task)
    Status request(const AlertRequest& r) override;
    void cancel(AlertTone tone) override;
    bool isPlaying() const override;

    /// For the PowerManager: a tone is waiting or playing (keep / turn the amplifier on).
    bool wantsAmplifier() const;
    void setAmplifierReady(bool ready, uint32_t nowMs);
    /// Called periodically by the emergency task: drops requests that waited too long for the amp.
    void service(uint32_t nowMs);

    /// Fills @p frames mono samples (the I2S backend duplicates to L/R if needed). ISR-safe, bounded.
    size_t render(int16_t* dst, size_t frames);

    PcmToneStats stats() const;
    const PcmToneConfig& config() const { return cfg_; }

private:
    ICriticalSection& lock_;
    PcmToneConfig cfg_;
    // shared (under lock_)
    bool pending_ = false;
    AlertRequest pendingReq_{};
    uint32_t pendingSinceMs_ = 0u;
    bool ampReady_ = false;
    bool playing_ = false;
    bool stopRequested_ = false;
    AlertTone tone_ = AlertTone::None;
    // render state (ISR side; set when a pending tone is promoted)
    uint32_t phase_ = 0u;          ///< 16.16 fixed-point phase accumulator
    uint32_t phaseStep_ = 0u;
    uint32_t remaining_ = 0u;      ///< samples left; UINT32_MAX = until cancelled
    uint32_t played_ = 0u;
    uint32_t ramp_ = 0u;           ///< ramp length in samples
    uint32_t releaseLeft_ = 0u;
    int32_t amplitude_ = 0;
    PcmToneStats stats_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_OUTPUT_AUDIO_PCM_TONE_OUTPUT_HPP
