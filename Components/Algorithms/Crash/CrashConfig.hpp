/**
 ******************************************************************************
 * @file    CrashConfig.hpp
 * @brief   Parameters of the CrashDetector.
 *
 * EVERY VALUE HERE IS A PROVISIONAL DEVELOPMENT PARAMETER. None has been
 * validated against real paraglider incidents, hard landings or device drops.
 * The reasoning behind each initial value is given next to it and in
 * Docs/CRASH_EMERGENCY_DESIGN.md section 5; the synthetic sensitivity sweeps
 * are in Docs/CRASH_EMERGENCY_VALIDATION_REPORT.md.
 *
 * Units: specific force in m/s^2 (1 g = 9.80665 m/s^2), rates in rad/s,
 * times in ms (MCU clock).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_CONFIG_HPP
#define PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_CONFIG_HPP

#include <cstdint>

namespace pg {

struct CrashConfig {
    static constexpr float kG = 9.80665f;
    static constexpr float kDeg = 0.0174532925f;

    // ---- impact candidate (corroborated IMU peaks, before any smoothing) ----
    /// |specific force| for an impact candidate. 4 g: above turbulence (~1.5-2.5 g) and
    /// normal landings (~1.5-3 g), below typical hard-impact pulses (5-20 g, short).
    /// A hard spiral can SUSTAIN 3-4 g, which is why impulseMin_mps2 is also required.
    float impactAccel_mps2 = 4.0f * kG;
    /// |specific force| for a "strong" impact (alone sufficient for the strong class).
    float severeAccel_mps2 = 8.0f * kG;
    /// The peak must exceed the recent baseline (EMA of |f|) by this much: a slowly
    /// built-up load (spiral) is not an impulse.
    float impulseMin_mps2 = 2.0f * kG;
    float baselineTau_s = 0.5f;             ///< EMA time constant of the |f| baseline
    /// |angular rate| for an impact candidate. 400 deg/s: spirals ~100-200 deg/s;
    /// tumbling / cartwheeling impacts are usually far above.
    float impactRate_radps = 400.0f * kDeg;
    float severeRate_radps = 800.0f * kDeg;
    /// The rate must also exceed its recent baseline (EMA, same tau) by this much:
    /// a sustained spiral rotation (~150 deg/s) is not an impulse.
    float rateImpulseMin_radps = 250.0f * kDeg;
    /// Sudden changes between consecutive frames (reported as observations).
    float forceJump_mps2 = 3.0f * kG;
    float rateJump_radps = 300.0f * kDeg;
    /// Candidate collection: further peaks within this window belong to the same impact.
    uint32_t candidateWindowMs = 500u;

    // ---- pre-impact context (from the bounded history) ----
    uint32_t preWindowMs = 3000u;           ///< "just before the impact"
    uint32_t recentAirborneMs = 30000u;     ///< airborne phase this recently = eligible context
    float fastDescent_mps = 4.0f;           ///< navigation sink rate before impact (trim sink ~1-1.5 m/s)
    float closureClearance_m = 10.0f;       ///< LiDAR: close to terrain ...
    float closureRate_mps = 3.0f;           ///< ... and closing at least this fast
    float highGroundSpeed_mps = 13.0f;      ///< GNSS ground speed (above normal trim ~8-11 m/s)
    float freeFall_mps2 = 0.35f * kG;       ///< |f| below this = (near) free fall
    uint32_t freeFallMinMs = 150u;          ///< free fall at least this long ...
    uint32_t freeFallGapMs = 300u;          ///< ... ending at most this long before the impact

    // ---- post-impact observation ----
    uint32_t minObservationMs = 2000u;      ///< earliest decision after the impact (lets data settle)
    uint32_t observationMs = 10000u;        ///< longest observation; undecided at the end = Rejected
    float quietForce_mps2 = 0.8f;           ///< ||f| - g| below this ...
    float quietRate_radps = 0.20f;          ///< ... and |rate| below this = quiet sample
    uint32_t quietToleranceMs = 200u;       ///< short non-quiet blips do not restart the quiet period
    uint32_t inactivityMs = 4000u;          ///< quiet this long = Inactivity
    float abnormalTilt_rad = 60.0f * kDeg;  ///< body z axis this far from vertical ...
    float tiltMaxRate_radps = 0.5f;         ///< ... while not rotating fast (not a banked turn) ...
    uint32_t abnormalTiltMs = 2000u;        ///< ... for this long = AbnormalOrientation
    float activity_mps2 = 1.5f;             ///< ||f| - g| above this = pilot / device moving
    uint32_t activityMs = 3000u;            ///< accumulated activity this long = PilotActivity (reported)
    uint8_t repeatedImpacts = 2u;           ///< further candidates in the observation = RepeatedImpacts
    float resumedFlightSpeed_mps = 6.0f;    ///< GNSS speed this high ...
    uint32_t resumedFlightMs = 3000u;       ///< ... for this long, while the phase is airborne = flying again
                                            ///< (shorter than inactivityMs: a turbulence hit in calm air is
                                            ///< recognised as flight before it can look like inactivity)

    /// Strong impact with airborne context whose post-impact IMU data are missing for at
    /// least this fraction of the observation: decided on the pre-impact evidence alone
    /// (explicit policy: missing data after a strong impact is NOT proof of safety).
    float dataLossFraction = 0.5f;
    bool suspectOnPostImpactDataLoss = true;

    // ---- after a decision ----
    uint32_t rejectedHoldMs = 1000u;        ///< Rejected is shown this long, then Monitoring
    uint32_t suspectedHoldMs = 60000u;      ///< CrashSuspected is held (the emergency manager has latched it)
    uint32_t cooldownMs = 30000u;           ///< after a suspicion: impacts counted, no new suspicion
    uint32_t imuUnavailableMs = 1000u;      ///< no IMU data this long (outside an incident) = Unavailable

    // ---- history ----
    uint32_t historySlotMs = 250u;          ///< one history slot per 250 ms (32 slots = 8 s)

    bool isValid() const
    {
        return impactAccel_mps2 > 1.5f * kG && severeAccel_mps2 > impactAccel_mps2 && impulseMin_mps2 > 0.0f &&
               baselineTau_s > 0.0f && impactRate_radps > 0.0f && severeRate_radps > impactRate_radps && rateImpulseMin_radps > 0.0f &&
               candidateWindowMs > 0u && candidateWindowMs < minObservationMs && minObservationMs < observationMs &&
               preWindowMs > 0u && recentAirborneMs >= preWindowMs && fastDescent_mps > 0.0f &&
               freeFall_mps2 > 0.0f && freeFall_mps2 < kG && quietForce_mps2 > 0.0f && quietRate_radps > 0.0f &&
               inactivityMs > 0u && inactivityMs < observationMs && abnormalTilt_rad > 0.0f &&
               abnormalTilt_rad < 3.1416f && dataLossFraction > 0.0f && dataLossFraction <= 1.0f &&
               historySlotMs >= 50u && historySlotMs * 32u >= preWindowMs + candidateWindowMs && cooldownMs > 0u;
    }
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_CONFIG_HPP
