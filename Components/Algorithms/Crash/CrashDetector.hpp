/**
 ******************************************************************************
 * @file    CrashDetector.hpp
 * @brief   Multi-evidence, time-windowed possible-impact / suspected-crash
 *          detector (portable: no HAL, no FreeRTOS, no heap).
 *
 * WHAT IT DOES
 *   1. IMPULSE: every frame it takes the largest |specific force| and |rate|
 *      of each IMU since the previous frame (ImuChannelInfo peaks: BEFORE the
 *      voter, Madgwick or the UKF smooth anything). The second-largest of the
 *      IMUs is used ("corroborated"), so one faulty IMU can neither create
 *      nor hide an impact when 3 IMUs work. A peak must also stand out from
 *      the recent |f| baseline (an impulse, not a sustained spiral load).
 *   2. CONTEXT: a bounded 8 s history (32 slots x 250 ms) of flight phase,
 *      navigation sink rate, LiDAR clearance / closure and GNSS speed gives
 *      what happened JUST BEFORE the impulse.
 *   3. OBSERVATION: for up to observationMs after the impulse it watches for
 *      inactivity, abnormal orientation, repeated impacts, resumed flight and
 *      missing data.
 *   4. DECISION: CrashSuspected only when the impulse class plus the counted
 *      independent indicators satisfy an explicit rule (CRASH_EMERGENCY_DESIGN.md
 *      section 6.4); otherwise Rejected. A single accelerometer threshold can
 *      never produce a suspicion.
 *
 * TIME: every interval uses wrap-safe uint32 differences of real timestamps
 * (never frame counts). Frames older than the previous one are ignored.
 *
 * Events (through the SafetyEventEmitter of the same task): ImpactCandidate
 * Raised / Cleared, CrashSuspected Raised / Cleared.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_DETECTOR_HPP
#define PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_DETECTOR_HPP

#include <cstdint>

#include "Algorithms/Crash/CrashConfig.hpp"
#include "Algorithms/Crash/CrashTypes.hpp"
#include "Algorithms/Safety/SafetyEventEmitter.hpp"

namespace pg {

class CrashDetector {
public:
    static constexpr uint8_t kHistorySlots = 32u;

    explicit CrashDetector(const CrashConfig& cfg = CrashConfig{});

    bool configValid() const { return configValid_; }
    const CrashConfig& config() const { return cfg_; }

    /// One update per processing frame. Bounded work (~32-slot scans at most).
    const CrashStatus& update(const CrashInput& in, SafetyEventEmitter& emitter);
    const CrashStatus& latest() const { return st_; }

private:
    struct Slot {
        uint32_t startMs;
        bool used;
        bool airborne;          ///< phase airborne during the slot
        bool takeoffCandidate;
        bool vzValid;
        float minVz;
        bool clearanceValid;
        float minClearance;
        bool closureValid;
        float maxClosure;
        bool speedValid;
        float maxSpeed;
    };
    struct Impulse {
        bool any;               ///< at least one IMU had new data
        uint8_t channels;
        float accel;            ///< corroborated
        float rate;
        float maxAccel;         ///< largest single IMU
        uint32_t timestampMs;
        bool saturated;         ///< corroborated saturation (>= 2 IMUs, or the only IMU)
        bool uncorroborated;    ///< one IMU alone above the impact threshold
    };

    Impulse readImpulse(const CrashInput& in);
    void recordHistory(const CrashInput& in);
    void evaluatePreContext(uint32_t t0, uint32_t nowMs);
    void trackPostImpact(const CrashInput& in, const Impulse& imp, float f, float w, uint32_t dtMs);
    bool isImpulse(const Impulse& imp) const;
    ImpactClass classify(const Impulse& imp) const;
    void startCandidate(const CrashInput& in, const Impulse& imp, SafetyEventEmitter& em);
    void decide(const CrashInput& in, SafetyEventEmitter& em, bool windowEnd);
    void suspect(const CrashInput& in, CrashDecision why, SafetyEventEmitter& em);
    void reject(const CrashInput& in, CrashDecision why, SafetyEventEmitter& em);
    void enter(CrashState s, uint32_t nowMs);
    void emit(const CrashInput& in, SafetyEventEmitter& em, SafetyEventType type, SafetyTransition tr,
              SafetySeverity sev, SafetySeverity prev);
    uint8_t preCount() const;
    uint8_t postCount() const;

    CrashConfig cfg_;
    bool configValid_;
    CrashStatus st_{};

    // time / duplicate protection
    bool started_ = false;
    uint32_t lastNowMs_ = 0u;
    uint32_t firstUpdateMs_ = 0u;
    bool gapSinceEpisode_ = false;
    bool jumpForce_ = false;   ///< this frame: sudden |f| change
    bool jumpRate_ = false;
    bool peakSeen_[kImuCount] = {};
    uint32_t lastPeakMs_[kImuCount] = {};
    uint32_t lastImuDataMs_ = 0u;
    bool imuEverSeen_ = false;

    // signal trackers
    bool baselineValid_ = false;
    float baseline_ = 0.0f;       ///< EMA of |f| over non-impulse frames
    float rateBaseline_ = 0.0f;   ///< EMA of |rate| over non-impulse frames
    bool prevValid_ = false;
    float prevForce_ = 0.0f;
    float prevRate_ = 0.0f;
    bool inFreeFall_ = false;
    uint32_t freeFallStartMs_ = 0u;
    uint32_t freeFallEndMs_ = 0u;
    uint32_t freeFallDurationMs_ = 0u;
    bool everAirborne_ = false;
    uint32_t lastAirborneMs_ = 0u;

    // history
    Slot hist_[kHistorySlots] = {};
    uint8_t head_ = 0u;

    // episode
    uint32_t impactMs_ = 0u;          ///< first corroborated peak
    uint32_t lastImpulseMs_ = 0u;
    bool strong_ = false;
    uint32_t quietMs_ = 0u;
    uint32_t notQuietMs_ = 0u;
    uint32_t tiltMs_ = 0u;
    uint32_t notTiltMs_ = 0u;
    uint32_t activityMs_ = 0u;
    uint32_t fastMovingMs_ = 0u;
    uint32_t dataMissingMs_ = 0u;
    uint32_t observedTotalMs_ = 0u;
    uint8_t extraImpacts_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_DETECTOR_HPP
