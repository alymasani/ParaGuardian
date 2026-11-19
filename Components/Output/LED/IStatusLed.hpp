/**
 ******************************************************************************
 * @file    IStatusLed.hpp
 * @brief   Hardware-independent status LED interface.
 *
 * LED count, colour (single / RGB), polarity and GPIO/PWM pins are unknown
 * until the schematic exists, so only the patterns are defined here.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_OUTPUT_STATUS_LED_HPP
#define PARAGUARDIAN_OUTPUT_STATUS_LED_HPP

#include <cstdint>

namespace pg {

enum class LedPattern : uint8_t {
    Off = 0,
    SolidOn,
    SlowBlink,     ///< e.g. 1 Hz: system running normally
    FastBlink,     ///< e.g. 5 Hz: sensor fault / attention
    DoubleBlink,   ///< e.g. waiting for GNSS fix
    SosBlink,      ///< emergency mode
};

enum class LedColor : uint8_t { Default = 0, Red, Green, Blue, Amber };

class IStatusLed {
public:
    virtual void setPattern(LedPattern pattern, LedColor color = LedColor::Default) = 0;
    /// Called periodically (e.g. every 10 ms) by the owning task to animate patterns.
    virtual void tick(uint32_t nowMs) = 0;
    /// False while no LED hardware is configured (pins unknown). Default: available.
    virtual bool available() const { return true; }

protected:
    ~IStatusLed() = default;
};

}  // namespace pg

#endif  // PARAGUARDIAN_OUTPUT_STATUS_LED_HPP
