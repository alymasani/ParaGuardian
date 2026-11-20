/**
 ******************************************************************************
 * @file    GpioStatusLed.cpp
 * @brief   See GpioStatusLed.hpp.
 ******************************************************************************
 */
#include "Output/LED/GpioStatusLed.hpp"

#include "Output/LED/LedPatternScheduler.hpp"

namespace pg {

uint8_t GpioStatusLed::maskFor(LedColor c) const
{
    uint8_t m = 0u;
    switch (c) {
        case LedColor::Red: m = 1u; break;
        case LedColor::Green: m = 2u; break;
        case LedColor::Blue: m = 4u; break;
        case LedColor::Amber: m = 3u; break;
        case LedColor::Default:
        default: m = 0u; break;
    }
    uint8_t avail = 0u;
    for (uint8_t i = 0; i < 3u; ++i) avail = static_cast<uint8_t>(avail | (out_[i] != nullptr ? (1u << i) : 0u));
    m &= avail;
    if (m == 0u) {   // colour not wired: use the first configured output
        for (uint8_t i = 0; i < 3u; ++i) {
            if (out_[i] != nullptr) return static_cast<uint8_t>(1u << i);
        }
    }
    return m;
}

void GpioStatusLed::setPattern(LedPattern pattern, LedColor color)
{
    if (pattern == pattern_ && color == color_) return;
    pattern_ = pattern;
    color_ = color;
    restart_ = true;
}

void GpioStatusLed::tick(uint32_t nowMs)
{
    if (!available()) return;
    if (restart_) {
        restart_ = false;
        startMs_ = nowMs;
    }
    const bool on = LedPatternScheduler::levelAt(pattern_, nowMs - startMs_);
    const uint8_t mask = on ? maskFor(color_) : 0u;
    if (written_ && mask == lastMask_) return;
    const uint8_t changed = written_ ? static_cast<uint8_t>(mask ^ lastMask_) : 0x07u;
    written_ = true;
    for (uint8_t i = 0; i < 3u; ++i) {
        if (out_[i] != nullptr && (changed & (1u << i)) != 0u) {
            out_[i]->write((mask & (1u << i)) != 0u);
            ++writes_;
        }
    }
    lastMask_ = mask;
}

}  // namespace pg
