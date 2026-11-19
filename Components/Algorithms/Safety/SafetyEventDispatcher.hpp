/**
 ******************************************************************************
 * @file    SafetyEventDispatcher.hpp
 * @brief   The consumer of the safety events: keeps the set of active pilot
 *          alerts, resolves priorities, handles acknowledgment, and drives
 *          the audio and LED schedulers. Portable, no heap, never blocks.
 *
 * TWO KINDS OF INPUT, ON PURPOSE
 *   - EVENTS (SafetyEventQueue, transitions): edges. A Raised / Escalated
 *     warning restarts its sound and clears an acknowledgment; a warning
 *     raised and cleared between two snapshots is still announced once
 *     (transient). Event-id gaps (events lost upstream) are counted.
 *   - STATE (SafetySnapshot, CrashStatus, EmergencyStatus, latest values):
 *     authoritative "what is active now". A Cleared event lost to queue
 *     overflow therefore cannot leave an alert stuck, and the SOS state is
 *     never lost because an advisory queue filled: it comes from the
 *     EmergencyManager's status directly, not from events.
 *
 * MISSING INPUT: if the safety snapshot stops updating (processing stalled),
 * landing alerts are held for staleHoldMs, then replaced by SystemDegraded.
 * Emergency alerts do not depend on sensor frames at all.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_DISPATCHER_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_DISPATCHER_HPP

#include <cstdint>

#include "Algorithms/Crash/CrashTypes.hpp"
#include "Algorithms/Emergency/EmergencyTypes.hpp"
#include "Algorithms/Landing/LandingSafetyTypes.hpp"
#include "Algorithms/Safety/AlertPriorityPolicy.hpp"
#include "Algorithms/Safety/AlertTypes.hpp"
#include "Output/Audio/AudioAlertScheduler.hpp"
#include "Output/LED/LedPatternScheduler.hpp"

namespace pg {

struct DispatcherConfig {
    uint32_t snapshotMaxAgeMs = 1000u;   ///< safety snapshot older than this = stale
    uint32_t staleHoldMs = 3000u;        ///< landing alerts held this long after the snapshot went stale
    uint32_t transientHoldMs = 2000u;    ///< a raised-then-cleared warning still sounds this long
    uint32_t countdownFinalMs = 10000u;  ///< faster countdown beeps in the last seconds
    uint32_t acceptedShowMs = 30000u;    ///< SosAccepted information shown this long
    bool isValid() const { return snapshotMaxAgeMs > 0u && staleHoldMs > 0u && transientHoldMs > 0u; }
};

/// Latest-value inputs of one dispatcher update (pointers may be null = not available).
struct DispatcherInputs {
    const SafetySnapshot* safety;
    const CrashStatus* crash;
    const EmergencyStatus* emergency;
};

class SafetyEventDispatcher {
public:
    SafetyEventDispatcher(const DispatcherConfig& cfg, IAudioAlert* audio, IStatusLed* led);

    bool configValid() const { return cfg_.isValid(); }

    /// One event (popped from a queue). Bounded work.
    void consume(const SafetyEvent& e, uint32_t nowMs);
    /// Pilot acknowledgment: silences the audio of the current alert episode if allowed.
    void acknowledge(uint32_t nowMs);
    /// Re-evaluates alerts and advances the output schedulers.
    const DispatchStatus& update(uint32_t nowMs, const DispatcherInputs& in);

    const DispatchStatus& status() const { return st_; }
    const AudioAlertScheduler& audio() const { return audio_; }
    const LedPatternScheduler& led() const { return led_; }

private:
    struct Alert {
        bool active;
        SafetySeverity severity;
        uint32_t sinceMs;
        uint32_t episode;          ///< restarts the sound when it changes
        bool acknowledged;
        bool transient;            ///< kept alive by an event only
        bool transientCounted;
        uint32_t transientUntilMs;
    };
    void set(AlertKind k, bool active, SafetySeverity sev, uint32_t episode, uint32_t nowMs);
    bool trackGap(const SafetyEvent& e);   ///< false = duplicate / older event

    DispatcherConfig cfg_;
    AudioAlertScheduler audio_;
    LedPatternScheduler led_;
    DispatchStatus st_{};
    Alert alerts_[kAlertKindCount] = {};
    uint32_t edgeSerial_[kAlertKindCount] = {};   ///< bumped by Raised / Escalated events
    bool haveEventId_[2] = {};
    uint32_t lastEventId_[2] = {};                ///< [0] processing emitter, [1] emergency emitter
    uint32_t lastSafetyChangeMs_ = 0u;
    bool safetyEverSeen_ = false;
    AlertKind lastAudioAlert_ = AlertKind::None;
    uint8_t lastAudioPriority_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_SAFETY_EVENT_DISPATCHER_HPP
