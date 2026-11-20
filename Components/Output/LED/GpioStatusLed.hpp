/**
 ******************************************************************************
 * @file    GpioStatusLed.hpp
 * @brief   IStatusLed backend driving up to three logical GPIO outputs
 *          (red / green / blue) with the LedPatternScheduler timing table.
 *
 * No pin is assumed: the board layer hands in IDigitalOutput objects (with
 * their polarity) once the schematic and CubeMX configuration exist. Any of
 * the three may be null (single-colour LED). The firmware default remains
 * UnconfiguredStatusLed. Non-blocking: tick() only computes the pattern level
 * and writes pins whose level CHANGED.
 * Colours: Red, Green, Blue map to one output; Amber = red + green;
 * Default = the first configured output. A missing colour falls back to the
 * first configured output (the pattern still blinks: information kept).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_OUTPUT_LED_GPIO_STATUS_LED_HPP
#define PARAGUARDIAN_OUTPUT_LED_GPIO_STATUS_LED_HPP

#include <cstdint>

#include "Output/LED/IStatusLed.hpp"

namespace pg {

/// One logical output pin. write(true) = LED on (the adapter applies the polarity).
class IDigitalOutput {
public:
    virtual void write(bool on) = 0;

protected:
    ~IDigitalOutput() = default;
};

class GpioStatusLed final : public IStatusLed {
public:
    GpioStatusLed(IDigitalOutput* red, IDigitalOutput* green, IDigitalOutput* blue)
    {
        out_[0] = red;
        out_[1] = green;
        out_[2] = blue;
    }

    void setPattern(LedPattern pattern, LedColor color) override;
    void tick(uint32_t nowMs) override;
    bool available() const override { return out_[0] != nullptr || out_[1] != nullptr || out_[2] != nullptr; }

    uint32_t pinWrites() const { return writes_; }

private:
    uint8_t maskFor(LedColor c) const;

    IDigitalOutput* out_[3] = {};
    LedPattern pattern_ = LedPattern::Off;
    LedColor color_ = LedColor::Default;
    bool restart_ = true;
    uint32_t startMs_ = 0u;
    uint8_t lastMask_ = 0u;
    bool written_ = false;       ///< false: every configured pin is written on the next tick
    uint32_t writes_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_OUTPUT_LED_GPIO_STATUS_LED_HPP
