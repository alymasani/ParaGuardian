/**
 ******************************************************************************
 * @file    LedPatternScheduler.cpp
 * @brief   See LedPatternScheduler.hpp.
 ******************************************************************************
 */
#include "Output/LED/LedPatternScheduler.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
void inc(uint32_t& c)
{
    if (c != UINT32_MAX) ++c;
}

struct Segment {
    uint16_t ms;
    bool on;
};
/// "... --- ..." with a 150 ms unit, then a pause: 3.6 s period.
const Segment kSos[] = {{150, true}, {150, false}, {150, true}, {150, false}, {150, true}, {300, false},
                        {450, true}, {150, false}, {450, true}, {150, false}, {450, true}, {300, false},
                        {150, true}, {150, false}, {150, true}, {150, false}, {150, true}, {750, false}};

bool segmentLevel(const Segment* seg, size_t n, uint32_t phaseMs)
{
    uint32_t period = 0u;
    for (size_t i = 0; i < n; ++i) period += seg[i].ms;
    uint32_t t = phaseMs % period;
    for (size_t i = 0; i < n; ++i) {
        if (t < seg[i].ms) return seg[i].on;
        t -= seg[i].ms;
    }
    return false;
}
}  // namespace

bool LedPatternScheduler::levelAt(LedPattern pattern, uint32_t phaseMs)
{
    switch (pattern) {
        case LedPattern::Off: return false;
        case LedPattern::SolidOn: return true;
        case LedPattern::SlowBlink: return (phaseMs % 1000u) < 500u;     // 1 Hz
        case LedPattern::FastBlink: return (phaseMs % 200u) < 100u;      // 5 Hz
        case LedPattern::DoubleBlink: {
            const uint32_t t = phaseMs % 1000u;
            return t < 100u || (t >= 200u && t < 300u);
        }
        case LedPattern::SosBlink: return segmentLevel(kSos, sizeof(kSos) / sizeof(kSos[0]), phaseMs);
        default: return false;
    }
}

void LedPatternScheduler::setDesired(LedPattern pattern, LedColor color)
{
    desired_ = pattern;
    desiredColor_ = color;
}

void LedPatternScheduler::update(uint32_t nowMs)
{
    const bool changed = !started_ || desired_ != current_ || desiredColor_ != color_;
    if (changed) {
        current_ = desired_;
        color_ = desiredColor_;
        startMs_ = nowMs;
        started_ = true;
    }
    level_ = levelAt(current_, timebase::ageMs(startMs_, nowMs));
    if (backend_ == nullptr || !backend_->available()) {
        state_ = OutputState::Unavailable;   // computed, not shown: no LED hardware
        return;
    }
    if (changed || state_ == OutputState::Unavailable) {
        backend_->setPattern(current_, color_);
        inc(changes_);
    }
    backend_->tick(nowMs);
    state_ = current_ == LedPattern::Off ? OutputState::Idle : OutputState::Active;
}

}  // namespace pg
