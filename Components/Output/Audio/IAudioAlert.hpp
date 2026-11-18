/**
 ******************************************************************************
 * @file    IAudioAlert.hpp
 * @brief   Hardware-independent alert-tone interface (future MAX98357A I2S amp).
 *
 * The MAX98357A is a digital (I2S) class-D amplifier: the MCU must stream PCM
 * samples to it over SAI/I2S (normally via DMA). Which SAI/I2S instance, its
 * pins, the SD_MODE/gain pins and the sample rate all depend on the schematic,
 * so ONLY the interface exists for now. No pins are assumed.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_OUTPUT_AUDIO_ALERT_HPP
#define PARAGUARDIAN_OUTPUT_AUDIO_ALERT_HPP

#include <cstdint>

#include "Common/Status.hpp"

namespace pg {

enum class AlertTone : uint8_t {
    None = 0,
    PowerOn,
    SensorFault,
    LowBattery,
    Warning,     ///< e.g. future obstacle / terrain proximity warning
    Emergency,   ///< SOS in progress
};

enum class AlertPriority : uint8_t { Low = 0, Normal, High, Critical };

struct AlertRequest {
    AlertTone tone;
    AlertPriority priority;
    uint16_t durationMs;   ///< 0 = until cancelled
    uint8_t volumePercent; ///< 0..100
};

class IAudioAlert {
public:
    /// Starts (or pre-empts a lower-priority) alert. Must not block.
    virtual Status request(const AlertRequest& request) = 0;
    virtual void cancel(AlertTone tone) = 0;
    virtual bool isPlaying() const = 0;

protected:
    ~IAudioAlert() = default;
};

}  // namespace pg

#endif  // PARAGUARDIAN_OUTPUT_AUDIO_ALERT_HPP
