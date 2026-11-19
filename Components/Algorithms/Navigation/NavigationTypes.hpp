/**
 ******************************************************************************
 * @file    NavigationTypes.hpp
 * @brief   Input and output types of the vertical navigation subsystem.
 *
 * NavigationInput     what an estimator consumes (built from a
 *                     ProcessedSensorFrame + AttitudeEstimate by
 *                     navigationInputFrom(), or directly by simulation tests).
 * NavigationEstimate  what every vertical estimator produces. THIS is the
 *                     interface future safety algorithms (flight state,
 *                     landing, crash detection) consume. It does not expose
 *                     filter matrices or the process model: a consumer
 *                     cannot tell (and must not care) whether a
 *                     constant-acceleration UKF, an aerodynamic UKF or the
 *                     barometer-only fallback produced it, except through
 *                     the explicit `estimator` / `processModel` fields.
 *
 * Conventions:
 *   state x = [h, vz, az]: relative altitude (m), vertical velocity (m/s),
 *   vertical linear acceleration (m/s^2). POSITIVE UP everywhere.
 *   h is relative to the barometric reference (where the barometers were
 *   calibrated), NOT height above ground. Terrain clearance is the LiDAR's job.
 *   Times are MCU milliseconds (HAL tick), compared wrap-safely.
 *
 * RULE (same as ProcessedSensorFrame and AttitudeEstimate): a number means
 * nothing unless its validity flag says so. Invalid numbers are NaN, never 0.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_TYPES_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_TYPES_HPP

#include <cstdint>
#include <type_traits>

#include "Algorithms/Attitude/AttitudeTypes.hpp"
#include "Processing/ProcessingTypes.hpp"
#include "Sensors/Common/SensorTypes.hpp"

namespace pg {

/// Indices of the vertical state vector.
namespace vstate {
constexpr unsigned kAltitude = 0u;
constexpr unsigned kVelocity = 1u;
constexpr unsigned kAcceleration = 2u;
constexpr unsigned kDim = 3u;
}  // namespace vstate

/// Navigation state machine (see VERTICAL_UKF_DESIGN.md section 9).
enum class NavState : uint8_t {
    Uninitialized = 0,  ///< no barometric altitude seen yet
    Initializing,       ///< collecting a barometer window to set h, vz and their uncertainty
    Tracking,           ///< barometer (and normally acceleration) updates being accepted
    Degraded,           ///< estimate propagated, but a primary source is missing / rejected for long
    Recovering,         ///< a source came back after an outage or persistent rejection; confirming
    Unavailable,        ///< uncertainty or outage too large: no usable output; re-initialises when data return
    NumericalFault,     ///< the filter produced a non-finite / non-positive-definite result this update
};

/// Which algorithm produced a NavigationEstimate.
enum class NavEstimatorId : uint8_t {
    None = 0,
    VerticalUkf,     ///< VerticalNavigationEstimator (UKF + injected process model)
    BaroAlphaBeta,   ///< BaroAlphaBetaEstimator (barometer only; comparison / possible fallback)
};

/// Which process model the UKF used. New models add an id here.
enum class ProcessModelId : uint8_t {
    None = 0,
    ConstantAcceleration = 1,   ///< temporary placeholder (white-jerk constant acceleration)
    Test = 200,                 ///< test-only models (e.g. the mock used to prove replaceability)
};

struct ProcessModelInfo {
    ProcessModelId id;
    uint16_t version;
};

/// What happened to one measurement source in one update.
enum class MeasurementOutcome : uint8_t {
    NoData = 0,         ///< source invalid / absent this update
    Accepted,
    AcceptedInflated,   ///< accepted with extra uncertainty (late sample, or recovery covariance inflation)
    NotNew,             ///< same sample as already assimilated (republished frame): skipped
    OutOfOrder,         ///< older than the last assimilated sample of this source: skipped
    TooLate,            ///< older than the filter time by more than maxLateMs: skipped
    Stale,              ///< older than the source's maximum age at processing time
    Ambiguous,          ///< redundant sensors disagree (barometers): skipped
    RejectedGate,       ///< innovation outside the gate
    RejectedQuality,    ///< source quality too poor to use (e.g. attitude invalid / recovering from a gap)
    Disabled,           ///< configuration disables the source
    UsedForInit,        ///< consumed by initialisation
    NumericalFault,     ///< the update produced an invalid result and was not applied
    FutureTimestamp,    ///< sample time after the processing time (clock/producer error): skipped
};

namespace navflag {
/// NavigationEstimate::qualityFlags bits. Set = something the consumer must know.
enum : uint32_t {
    NotInitialized         = 1u << 0,
    BaroUnavailable        = 1u << 1,   ///< no barometer accepted for baroDegradedMs
    AccelUnavailable       = 1u << 2,   ///< no acceleration accepted for accelDegradedMs
    GnssUnused             = 1u << 3,   ///< no GNSS velocity accepted for gnssUnusedMs (normal without GNSS)
    BaroRejected           = 1u << 4,   ///< this update's barometer sample was gated out
    AccelRejected          = 1u << 5,
    GnssRejected           = 1u << 6,
    AccelInflated          = 1u << 7,   ///< acceleration noise inflated for attitude quality this update
    PredictionOnly         = 1u << 8,   ///< no measurement assimilated this update (prediction / outage)
    LateMeasurement        = 1u << 9,   ///< a late sample was assimilated with inflated noise
    Recovering             = 1u << 10,
    ReferenceChanged       = 1u << 11,  ///< barometer reference pressure changed: altitude re-anchored
    BaroSourceChanged      = 1u << 12,  ///< contributing barometers changed
    BaroRedundancyDegraded = 1u << 13,  ///< only one barometer contributes
    AttitudeDegraded       = 1u << 14,  ///< acceleration came from a non-Tracking attitude
    AccelNotCalibrated     = 1u << 15,  ///< inherited: accelerometer bias not estimated (biases velocity)
    NumericalFault         = 1u << 16,  ///< a numerical failure occurred this update
    InvalidConfig          = 1u << 17,
    UncertaintyLarge       = 1u << 18,  ///< a sigma exceeded its validity limit
    ModelPlaceholder       = 1u << 19,  ///< process model is the unvalidated placeholder
};
}  // namespace navflag

/// Optional inputs a process model may use (the constant-acceleration model uses none).
/// A future aerodynamic model would read e.g. bank angle here; extending this
/// struct does not change the UKF engine or any consumer.
struct ProcessModelInput {
    uint32_t timestampMs;
    float bankAngle_rad;        ///< from attitude (display Euler roll), NaN if unknown
    float pitchAngle_rad;
    bool attitudeValid;
};

/// One update's worth of input. Positive UP. MCU-ms timestamps.
struct NavigationInput {
    uint32_t nowMs;

    // ---- barometric relative altitude (voted by preprocessing) ----
    bool baroValid;                         ///< preprocessing produced a voted altitude
    float baroAltitude_m;
    uint32_t baroTimestampMs;               ///< newest contributing sample
    uint32_t baroSpreadMs;                  ///< newest - oldest contributing sample
    RedundancyStatus baroStatus;            ///< Healthy / Degraded / Ambiguous / Unavailable
    uint8_t baroContributingMask;           ///< bit j = barometer j in the vote
    uint32_t baroChannelTimestampMs[kBaroCount];
    float baroReferencePressure_pa[kBaroCount];
    bool baroSourceChanged;

    // ---- gravity-compensated vertical acceleration (from AttitudeEstimate) ----
    bool accelValid;                        ///< linearAccel.valid && orientation valid
    bool accelReliable;                     ///< linearAccel.reliable (attitude in a Tracking state)
    float accelUp_mps2;
    uint32_t accelTimestampMs;              ///< accelerometer sample time
    AttitudeState attitudeState;
    CorrectionStatus attitudeAccelCorrection;
    uint32_t attitudeFlags;                 ///< attitude::* bits
    RedundancyStatus imuAccelStatus;
    RedundancyStatus imuGyroStatus;

    // ---- GNSS (optional) ----
    bool gnssValid;                         ///< fresh, accepted by preprocessing
    float gnssVerticalVelocity_mps;         ///< POSITIVE UP (NAV-PVT -velD, converted by the driver)
    float gnssSpeedAccuracy_mps;            ///< NAV-PVT sAcc (3-D speed accuracy estimate)
    uint32_t gnssTimestampMs;               ///< MCU time the solution was parsed
    uint32_t gnssTimeOfWeekMs;              ///< receiver iTOW (identifies the solution)
    GnssFixType gnssFixType;
    uint8_t gnssNumSatellites;

    ProcessModelInput model;
};

/// Per-source diagnostics in the estimate.
struct MeasurementDiagnostics {
    MeasurementOutcome last;       ///< outcome of this update
    float innovation;              ///< z - predicted z of the last evaluated sample (NaN = none)
    float innovationSigma;         ///< sqrt(S) of that sample
    float nis;                     ///< innovation^2 / S (NaN = none)
    float measurementSigma;        ///< sqrt(R) used (after inflation)
    uint32_t sampleAgeMs;          ///< age of the newest sample at nowMs (UINT32_MAX = never)
    uint32_t sinceAcceptedMs;      ///< UINT32_MAX = never accepted
    uint32_t accepted;             ///< lifetime counters (saturating)
    uint32_t rejectedGate;
    uint32_t rejectedQuality;
    uint32_t notNew;
    uint32_t outOfOrder;
    uint32_t tooLate;
    uint32_t inflated;
    uint32_t recoveries;           ///< persistent-rejection / post-outage covariance inflations
};

struct NavigationEstimate {
    uint32_t sequence;             ///< increments per update
    uint32_t timestampMs;          ///< time the state refers to (filter time)
    uint32_t computedAtMs;         ///< MCU time of the update
    uint32_t predictionAgeMs;      ///< computedAtMs - timestampMs (extrapolate with vz/az if needed)

    NavEstimatorId estimator;
    ProcessModelInfo processModel;
    NavState state;

    float altitude_m;              ///< relative to the barometric reference, positive up
    float verticalVelocity_mps;    ///< positive up
    float verticalAccel_mps2;      ///< linear (gravity-free), positive up
    bool altitudeValid;
    bool velocityValid;
    bool accelerationValid;

    float covariance[3][3];        ///< P (state order h, vz, az); NaN when not initialised
    float altitudeSigma_m;
    float velocitySigma_mps;
    float accelSigma_mps2;

    MeasurementDiagnostics baro;
    MeasurementDiagnostics accel;
    MeasurementDiagnostics gnss;

    uint32_t sinceAnyUpdateMs;     ///< time since any measurement was assimilated (outage duration)
    uint32_t outageMs;             ///< time since the last accepted barometer sample (UINT32_MAX = never)
    uint32_t qualityFlags;         ///< navflag::* bits

    // Lifetime counters (saturating)
    uint32_t initializations;
    uint32_t reinitializations;    ///< initialisations after the first (outage / fault / reference)
    uint32_t referenceResets;      ///< barometer reference changed -> altitude re-anchored
    uint32_t predictions;          ///< prediction steps
    uint32_t numericalFaults;
    uint32_t lastFaultCode;        ///< UkfStatus of the last numerical fault (0 = none)
    uint32_t lastFaultAtMs;
    uint32_t unavailableEpisodes;

    bool has(uint32_t flags) const { return (qualityFlags & flags) != 0u; }
};

static_assert(std::is_trivially_copyable<NavigationInput>::value, "input must be memcpy-safe");
static_assert(std::is_trivially_copyable<NavigationEstimate>::value, "estimate must be memcpy-safe");

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAVIGATION_TYPES_HPP
