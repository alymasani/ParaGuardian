/**
 ******************************************************************************
 * @file    LedPatternScheduler.hpp
 * @brief   Portable LED pattern selection + deterministic pattern timing on
 *          top of IStatusLed.
 *
 * The dispatcher chooses the pattern (highest-priority alert, or the normal
 * status pattern); this class forwards CHANGES to the backend
 * (setPattern only when it changes), animates it (backend tick), and
 * computes the reference on/off level itself (levelAt), which is the timing
 * specification a future GPIO / PWM backend must reproduce. No pins are
 * assumed; the firmware default backend is UnconfiguredStatusLed.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_OUTPUT_LED_PATTERN_SCHEDULER_HPP
#define PARAGUARDIAN_OUTPUT_LED_PATTERN_SCHEDULER_HPP

#include <cstdint>

#include "Algorithms/Safety/AlertTypes.hpp"
#include "Output/LED/IStatusLed.hpp"

namespace pg {

class LedPatternScheduler {
public:
    explicit LedPatternScheduler(IStatusLed* backend) : backend_(backend) {}

    void setDesired(LedPattern pattern, LedColor color);
    void update(uint32_t nowMs);

    /// Reference timing: LED on/off @p phaseMs after the pattern started.
    static bool levelAt(LedPattern pattern, uint32_t phaseMs);

    bool level() const { return level_; }
    LedPattern pattern() const { return current_; }
    LedColor color() const { return color_; }
    OutputState state() const { return state_; }
    uint32_t changes() const { return changes_; }

private:
    IStatusLed* backend_;
    LedPattern desired_ = LedPattern::SlowBlink;
    LedColor desiredColor_ = LedColor::Green;
    LedPattern current_ = LedPattern::Off;
    LedColor color_ = LedColor::Default;
    bool started_ = false;
    uint32_t startMs_ = 0u;
    bool level_ = false;
    OutputState state_ = OutputState::Idle;
    uint32_t changes_ = 0u;
};

/// Firmware default: LED count / colour / pins unknown (no schematic).
class UnconfiguredStatusLed final : public IStatusLed {
public:
    void setPattern(LedPattern, LedColor) override {}
    void tick(uint32_t) override {}
    bool available() const override { return false; }
};

}  // namespace pg

#endif  // PARAGUARDIAN_OUTPUT_LED_PATTERN_SCHEDULER_HPP
