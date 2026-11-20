/**
 ******************************************************************************
 * @file    EmergencyConfig.hpp
 * @brief   Parameters of the EmergencyManager.
 *
 * ENGINEERING PLACEHOLDERS, NOT VALIDATED SAFETY PARAMETERS. In particular the
 * countdown durations trade "time for a conscious pilot to cancel a false
 * alarm" against "delay before help is requested"; the right values need
 * user studies and the rescue organisation's input (HARDWARE_INTEGRATION_TODO.md
 * section 16). Times in ms (MCU clock).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_CONFIG_HPP
#define PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_CONFIG_HPP

#include <cstdint>

namespace pg {

struct EmergencyConfig {
    /// Countdown after a suspected crash before the SOS request is created (placeholder: 60 s).
    uint32_t autoCountdownMs = 60000u;
    /// Countdown after a manual SOS (guards an accidental press; 0 = immediate). Placeholder.
    uint32_t manualCountdownMs = 5000u;
    /// The crash detector withdrew its suspicion (flight resumed) during the countdown: cancel.
    bool autoCancelOnSuspicionCleared = true;
    /// A cancelled incident (nothing sent) re-arms after this long.
    uint32_t cancelRearmMs = 10000u;

    // ---- transport / retry (firmware -> phone) ----
    uint32_t ackTimeoutMs = 10000u;         ///< after a submit: no phone acceptance this long = attempt failed
    uint32_t retryBaseMs = 2000u;           ///< backoff after the 1st failed attempt; doubles each time ...
    uint32_t retryMaxMs = 60000u;           ///< ... up to this
    uint8_t maxAttempts = 8u;               ///< attempts per cycle; then DeliveryFailed (RetriesExhausted)
    uint8_t maxCycles = 3u;                 ///< a reconnect / manual SOS may start a new cycle this many times
    uint32_t busyRetryMs = 250u;            ///< transport busy: try again this soon (not an attempt)
    uint32_t linkPollMs = 1000u;            ///< transport down: check again this often (not an attempt)
    /// Phone accepted but reported no outbound outcome this long: DeliveryUnknown (may still change).
    uint32_t deliveryTimeoutMs = 120000u;

    // ---- crash-detector supervision ----
    /// The detector had a STRONG impact in flight under observation and then stopped
    /// reporting (processing stalled) this long: open an incident (explicit policy).
    uint32_t detectorSilenceMs = 5000u;
    bool escalateOnDetectorSilence = true;

    // ---- persistence / identity (Milestone 8) ----
    /// The first transmission of a request waits at most this long for the record holding its
    /// incident id to be COMMITTED (so a reset cannot hand the same id to a different incident).
    /// After it, the request is sent anyway, marked IncidentIdVolatile: help is never withheld
    /// because flash is slow or broken.
    uint32_t identityHoldMs = 2000u;
    /// Resets that may restart the SAME countdown. On the next one, restore() creates the SOS
    /// request immediately (a brown-out loop must not postpone help forever).
    uint8_t maxCountdownRestarts = 2u;

    bool isValid() const
    {
        return autoCountdownMs <= 600000u && manualCountdownMs <= autoCountdownMs && ackTimeoutMs > 0u &&
               retryBaseMs > 0u && retryMaxMs >= retryBaseMs && maxAttempts > 0u && maxCycles > 0u &&
               busyRetryMs > 0u && linkPollMs > 0u && deliveryTimeoutMs > ackTimeoutMs && detectorSilenceMs > 0u &&
               autoCountdownMs < 0x7FFFFFFFu && deliveryTimeoutMs < 0x7FFFFFFFu && identityHoldMs <= 10000u;
    }
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_EMERGENCY_EMERGENCY_CONFIG_HPP
