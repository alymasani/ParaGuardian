/**
 ******************************************************************************
 * @file    AlertPriorityPolicy.hpp
 * @brief   ONE table that decides how important each alert is and how it
 *          sounds / looks. Configurable by editing this table only.
 *
 * ORDER (highest first), and why:
 *   100 SosCountdown        the pilot must be able to cancel a false alarm NOW
 *    95 CrashSuspected      (normally covered by the countdown that follows it)
 *    92 SosCommFailure      help may NOT be on its way: the pilot must know
 *    90 SosActive           help is being requested
 *  80-89 landing Critical    imminent terrain contact / extreme descent
 *  60-69 landing Warning
 *  40-49 landing Caution
 *  20-29 landing Advisory, SosAccepted (information)
 * So a long-running advisory can never starve an SOS countdown, and any SOS
 * state outranks every landing warning (after a suspected crash, terrain
 * warnings from a device lying on the ground are noise).
 * The numbers are design choices, not validated human-factors results.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_ALERT_PRIORITY_POLICY_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_ALERT_PRIORITY_POLICY_HPP

#include "Algorithms/Safety/AlertTypes.hpp"

namespace pg {

inline AlertStyle alertStyleFor(AlertKind kind, SafetySeverity sev)
{
    switch (kind) {
        case AlertKind::SosCountdown:
            return AlertStyle{100u, AudioPatternId::CountdownTick, LedPattern::SosBlink, LedColor::Red, false};
        case AlertKind::CrashSuspected:
            return AlertStyle{95u, AudioPatternId::CrashAlarm, LedPattern::FastBlink, LedColor::Red, false};
        case AlertKind::SosCommFailure:
            return AlertStyle{92u, AudioPatternId::CommFailure, LedPattern::DoubleBlink, LedColor::Red, true};
        case AlertKind::SosActive:
            return AlertStyle{90u, AudioPatternId::SosConfirm, LedPattern::SosBlink, LedColor::Amber, true};
        case AlertKind::SosAccepted:
            return AlertStyle{25u, AudioPatternId::InfoChime, LedPattern::SlowBlink, LedColor::Blue, true};
        case AlertKind::None:
        case AlertKind::Count:
            return AlertStyle{0u, AudioPatternId::None, LedPattern::SlowBlink, LedColor::Green, true};
        default:
            break;
    }
    // Landing / system alerts: severity sets the band, the kind orders within it.
    uint8_t bonus = 0u;
    switch (kind) {
        case AlertKind::HazardousApproach: bonus = 4u; break;
        case AlertKind::ClosureWarning: bonus = 3u; break;
        case AlertKind::DescentWarning: bonus = 2u; break;
        case AlertKind::LowClearance: bonus = 1u; break;
        default: bonus = 0u; break;   // SystemDegraded
    }
    switch (sev) {
        case SafetySeverity::Critical:
            return AlertStyle{static_cast<uint8_t>(80u + bonus), AudioPatternId::CriticalFast, LedPattern::FastBlink,
                              LedColor::Red, false};
        case SafetySeverity::Warning:
            return AlertStyle{static_cast<uint8_t>(60u + bonus), AudioPatternId::WarningTriple, LedPattern::FastBlink,
                              LedColor::Amber, true};
        case SafetySeverity::Caution:
            return AlertStyle{static_cast<uint8_t>(40u + bonus), AudioPatternId::CautionDouble,
                              LedPattern::DoubleBlink, LedColor::Amber, true};
        case SafetySeverity::Advisory:
            return AlertStyle{static_cast<uint8_t>(20u + bonus), AudioPatternId::AdvisoryChime,
                              kind == AlertKind::SystemDegraded ? LedPattern::DoubleBlink : LedPattern::SlowBlink,
                              LedColor::Amber, true};
        case SafetySeverity::Info:
        default:
            return AlertStyle{0u, AudioPatternId::None, LedPattern::SlowBlink, LedColor::Green, true};
    }
}

/// Which pilot alert a landing / flight warning becomes.
inline AlertKind alertKindForLanding(SafetyEventType t)
{
    switch (t) {
        case SafetyEventType::ExcessiveDescent: return AlertKind::DescentWarning;
        case SafetyEventType::LowGroundClearance: return AlertKind::LowClearance;
        case SafetyEventType::RapidGroundClosure: return AlertKind::ClosureWarning;
        case SafetyEventType::HazardousApproach: return AlertKind::HazardousApproach;
        case SafetyEventType::TerrainClearanceUnavailable:
        case SafetyEventType::NavigationDegraded: return AlertKind::SystemDegraded;
        default: return AlertKind::None;
    }
}

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_ALERT_PRIORITY_POLICY_HPP
