/**
 ******************************************************************************
 * @file    CrashTypes.hpp
 * @brief   Input, evidence and output types of the CrashDetector.
 *
 * TERMINOLOGY (deliberate): the detector reports a POSSIBLE IMPACT
 * (ImpactCandidate) and, with enough supporting evidence, a SUSPECTED CRASH.
 * It never "confirms" a crash and says nothing about injury: the sensors
 * observe forces, rotation, motion and orientation of a box on a harness,
 * not the pilot's condition. Every evidence bit below is an OBSERVATION.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_TYPES_HPP
#define PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_TYPES_HPP

#include <cstdint>
#include <type_traits>

#include "Algorithms/Flight/FlightStateTypes.hpp"
#include "Processing/ProcessingTypes.hpp"   // kImuCount

namespace pg {

/// Crash detector state (see CRASH_EMERGENCY_DESIGN.md section 6).
enum class CrashState : uint8_t {
    Monitoring = 0,         ///< armed, watching for impact candidates
    ImpactCandidate,        ///< a corroborated impulse was seen; collecting the rest of the pulse
    PostImpactObservation,  ///< watching what happens after the impact
    CrashSuspected,         ///< enough independent evidence: escalation requested (NOT a confirmed crash)
    Rejected,               ///< the candidate did not gather enough evidence (shown briefly)
    Cooldown,               ///< after a suspicion: further impacts counted, no new suspicion
    Unavailable,            ///< no IMU data: impacts cannot be observed
};

/// Strength of the impulse itself (IMU only).
enum class ImpactClass : uint8_t { None = 0, Moderate, Strong };

/// Why a candidate ended the way it did.
enum class CrashDecision : uint8_t {
    None = 0,
    // suspected because ...
    StrongImpactWithContext,      ///< strong impulse + >= 2 pre-impact indicators
    StrongImpactWithSupport,      ///< strong impulse + pre- and post-impact indicators
    ImpactWithPostSupport,        ///< >= 2 post-impact indicators (+ context)
    StrongImpactFlightEnded,      ///< strong impulse in flight, flight never resumed (pilot may be moving)
    StrongImpactDataLoss,         ///< strong impulse in flight, post-impact data missing (explicit policy)
    // rejected because ...
    NoFlightContext,              ///< device not recently airborne (handling / drop / bump on the ground)
    DropSignature,                ///< free fall then impact while not airborne (dropped device)
    InsufficientEvidence,         ///< observation ended without enough support
    FlightResumed,                ///< flying again after the impact
};

namespace crashev {
/// Evidence bits of the current episode (CrashStatus::evidence). Observations, not proof.
enum : uint32_t {
    // impulse (IMU, corroborated by >= 2 IMUs when available)
    HighSpecificForce     = 1u << 0,
    SevereSpecificForce   = 1u << 1,
    HighAngularRate       = 1u << 2,
    SevereAngularRate     = 1u << 3,
    SuddenForceChange     = 1u << 4,
    SuddenRateChange      = 1u << 5,
    ImuSaturation         = 1u << 6,    ///< full scale reached: the true peak is LARGER than reported
    // pre-impact context
    AirborneAtImpact      = 1u << 8,    ///< phase airborne at / just before the impact
    RecentlyAirborne      = 1u << 9,    ///< airborne within recentAirborneMs
    TakeoffAttempt        = 1u << 10,   ///< TakeoffCandidate just before the impact
    FastDescent           = 1u << 11,   ///< navigation sink rate above fastDescent_mps before the impact
    GroundClosure         = 1u << 12,   ///< LiDAR: near terrain and closing fast before the impact
    HighGroundSpeed       = 1u << 13,   ///< GNSS ground speed high before the impact
    FreeFallBeforeImpact  = 1u << 14,   ///< |f| near zero just before the impact
    // post-impact
    Inactivity            = 1u << 16,   ///< quiet for inactivityMs
    AbnormalOrientation   = 1u << 17,   ///< lying far from upright, not turning
    RepeatedImpacts       = 1u << 18,   ///< further impacts (tumbling)
    PilotActivity         = 1u << 19,   ///< sustained movement (reported; a conscious pilot may move)
    ResumedFlight         = 1u << 20,   ///< flying again (counter-evidence)
    PostImpactDataMissing = 1u << 21,   ///< IMU data missing for a large part of the observation
};
constexpr uint32_t kImpulseMask = 0x7Fu;
constexpr uint32_t kPreMask = 0x7F00u;
constexpr uint32_t kPostMask = 0x3F0000u;
}  // namespace crashev

namespace crashflag {
/// Quality flags (CrashStatus::flags).
enum : uint32_t {
    SingleImuOnly        = 1u << 0,   ///< only one IMU delivered data: the peak is NOT corroborated
    UncorroboratedPeak   = 1u << 1,   ///< one IMU showed an impulse the others did not (ignored)
    NavigationMissing    = 1u << 2,   ///< no valid vertical speed in the pre-impact window
    LidarMissing         = 1u << 3,
    GnssMissing          = 1u << 4,
    AttitudeMissing      = 1u << 5,   ///< orientation evidence unavailable
    PhaseUnknown         = 1u << 6,   ///< flight phase Unknown at the impact
    HistoryGap           = 1u << 7,   ///< the processing stopped for longer than a history slot
    Unvalidated          = 1u << 8,   ///< thresholds are provisional development values (always, today)
    SampleRateLimited    = 1u << 9,   ///< 100 Hz polling: pulses < ~10 ms can be missed (always, today)
};
}  // namespace crashflag

/// One IMU's impact evidence of this frame (from ImuChannelInfo, before any smoothing).
struct CrashImuChannel {
    bool present;                 ///< the IMU has delivered data since boot
    uint16_t samples;             ///< samples since the previous frame (0 = nothing new)
    float peakAccel_mps2;         ///< largest |specific force| (lower bound if saturated)
    float peakRate_radps;
    uint32_t peakTimestampMs;
    bool accelSaturated;
    bool gyroSaturated;
};

/// The bounded input of the crash detector (~110 B). Each source keeps its own validity.
struct CrashInput {
    uint32_t nowMs;
    CrashImuChannel imu[kImuCount];
    // voted (redundancy-checked) values of the same frame
    bool forceValid;
    float forceNorm_mps2;         ///< |specific force| (gravity included: ~9.81 at rest)
    bool rateValid;
    float rateNorm_radps;
    bool newImuData;              ///< the frame contains IMU data newer than the previous frame
    // attitude
    bool attitudeValid;
    float tilt_rad;               ///< angle of the body z (down) axis from the vertical
    // safety layer (flight phase, LiDAR) of the same frame
    FlightPhase phase;
    bool clearanceValid;
    float clearance_m;
    bool closureValid;
    float closureRate_mps;        ///< positive = approaching terrain
    // navigation (UKF; smoothed, used only for CONTEXT, never for the impulse)
    bool vzValid;
    float vz_mps;                 ///< positive up
    // GNSS
    bool gnssValid;
    float groundSpeed_mps;
    uint32_t gnssTimestampMs;
};
static_assert(std::is_trivially_copyable<CrashInput>::value, "crash input must be memcpy-safe");

struct ImpactSummary {
    uint32_t timestampMs;         ///< time of the first corroborated peak
    float peakAccel_mps2;         ///< corroborated (second-largest IMU, or the only IMU) peak
    float peakRate_radps;
    float maxChannelAccel_mps2;   ///< largest single-IMU peak (uncorroborated, diagnostic)
    float baselineAccel_mps2;     ///< |f| baseline before the impulse
    float vzBefore_mps;           ///< lowest (most negative) navigation vz in the pre-window; NaN = none
    float clearanceBefore_m;      ///< lowest LiDAR clearance in the pre-window; NaN = none
    float groundSpeedBefore_mps;  ///< highest GNSS speed in the pre-window; NaN = none
    ImpactClass cls;
    uint8_t channels;             ///< IMUs with data at the peak
    uint8_t impacts;              ///< candidates in this episode (1 = single impact)
    bool saturated;
};

/// The detector's output (latest-value; also summarised in SafetyEvents).
struct CrashStatus {
    uint32_t sequence;
    uint32_t timestampMs;
    CrashState state;
    uint32_t stateSinceMs;
    uint32_t episodeId;           ///< SafetyEvent episode of the current / last candidate (0 = none yet)
    uint32_t suspicionSerial;     ///< increments at every new CrashSuspected (consumers latch on change)
    uint32_t evidence;            ///< crashev::* of the current / last episode
    uint32_t flags;               ///< crashflag::*
    CrashDecision decision;       ///< of the last finished candidate
    uint8_t preCount;             ///< pre-impact indicators (context)
    uint8_t postCount;            ///< post-impact indicators
    uint32_t observedMs;          ///< time since the impact (in observation / suspected)
    ImpactSummary impact;
    // lifetime counters (saturating)
    uint32_t impactCandidates;
    uint32_t suspicions;
    uint32_t rejections;
    uint32_t suspicionsCleared;   ///< suspected, then flight resumed
    uint32_t uncorroboratedPeaks;
    uint32_t saturationEvents;
    uint32_t unavailableEpisodes;
    uint32_t staleInputs;         ///< frames ignored (old / duplicate IMU data)
};
static_assert(std::is_trivially_copyable<CrashStatus>::value, "crash status must be memcpy-safe");

/// Last known position / navigation for an SOS request (never invented, always with its age).
struct IncidentContext {
    bool gnssFixValid;            ///< a valid 2D/3D fix has been seen
    int32_t latitude_1e7deg;
    int32_t longitude_1e7deg;
    float heightMsl_m;            ///< GNSS height above mean sea level (NOT the barometric altitude)
    float horizontalAccuracy_m;
    uint8_t fixType;              ///< pg::GnssFixType
    uint8_t numSatellites;
    uint32_t fixTimestampMs;      ///< MCU time of that fix (age = now - this)
    bool navAltitudeValid;
    float navAltitude_m;          ///< RELATIVE to the barometric reference (not MSL, not above ground)
    uint32_t navTimestampMs;
    bool navVelocityValid;
    float navVerticalVelocity_mps;
    uint32_t frameQualityFlags;   ///< quality::* of the newest frame (device health summary)
};
static_assert(std::is_trivially_copyable<IncidentContext>::value, "context must be memcpy-safe");

/// What SensorProcessingTask publishes for the emergency task.
struct CrashReport {
    CrashStatus status;
    IncidentContext context;
    uint32_t publishedMs;         ///< MCU time of publication (heartbeat: a stale report = detector silent)
};
static_assert(std::is_trivially_copyable<CrashReport>::value, "report must be memcpy-safe");

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_CRASH_CRASH_TYPES_HPP
