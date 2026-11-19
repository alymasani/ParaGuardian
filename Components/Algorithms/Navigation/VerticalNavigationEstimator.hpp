/**
 ******************************************************************************
 * @file    VerticalNavigationEstimator.hpp
 * @brief   Vertical navigation: UKF on x = [h, vz, az] with an injected,
 *          replaceable process model. Produces NavigationEstimate.
 *
 * RESPONSIBILITIES (and what is NOT here)
 *   here:   choosing which samples are new and usable, their noise, timing
 *           (variable dt, wrap-around, duplicates, out-of-order, late
 *           samples, outages), initialisation, recovery, the state machine,
 *           diagnostics, the output.
 *   engine: UnscentedKalmanFilter<3> (sigma points, predict, update, checks).
 *   model:  IVerticalProcessModel (how the state evolves; replaceable).
 *
 * TIME: the filter state refers to the "filter time" t_f. Each new sample is
 * applied AT ITS OWN TIMESTAMP: the state is predicted from t_f to the sample
 * time, then updated. Samples of one update are processed in time order. A
 * sample older than t_f (late) by at most maxLateMs (GNSS: gnssMaxLateMs) is
 * applied at t_f with its noise inflated by how much the state can change in
 * that interval (never treated as current); later than that it is skipped
 * (TooLate). With no new data for idlePredictMs, the state is predicted to
 * "now" so its uncertainty grows during outages. NavigationEstimate::timestampMs
 * is t_f; predictionAgeMs says how old that is at the update time.
 *
 * Memory: one object (~0.9 KB on the Cortex-M7), no heap, bounded work per
 * update (at most 3 samples; prediction split into <= maxPredictionMs /
 * maxPredictStepMs steps).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_VERTICAL_NAVIGATION_ESTIMATOR_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_VERTICAL_NAVIGATION_ESTIMATOR_HPP

#include <cstdint>

#include "Algorithms/Navigation/BaroSampleTracker.hpp"
#include "Algorithms/Navigation/IVerticalProcessModel.hpp"
#include "Algorithms/Navigation/NavigationConfig.hpp"
#include "Algorithms/Navigation/NavigationTypes.hpp"
#include "Algorithms/Navigation/UnscentedKalmanFilter.hpp"

namespace pg {

class VerticalNavigationEstimator {
public:
    using Ukf = nav::UnscentedKalmanFilter<vstate::kDim>;

    /// @p model must outlive the estimator (it is referenced, not copied).
    VerticalNavigationEstimator(const NavigationConfig& cfg, const IVerticalProcessModel& model);

    /// False if the configuration, UKF parameters or model parameters are unusable
    /// (the estimator then outputs Unavailable + InvalidConfig and does nothing).
    bool configValid() const { return configValid_; }

    const NavigationEstimate& update(const NavigationInput& in);
    const NavigationEstimate& latest() const { return est_; }
    void reset();

    // ---- read-only access for tests and diagnostics ----
    const Ukf& filter() const { return ukf_; }
    uint32_t filterTimeMs() const { return tf_; }
    const NavigationConfig& config() const { return cfg_; }

private:
    enum class Source : uint8_t { Baro = 0, Accel, Gnss };

    struct Candidate {
        Source src;
        uint32_t ts;
        float z;
        float sigma;
    };

    struct SourceTrack {
        uint32_t lastTs;             ///< last assimilated / consumed sample time
        bool haveLast;
        uint32_t lastAcceptedAt;     ///< time of the last accepted sample
        bool everAccepted;
        uint32_t rejectRun;          ///< consecutive gate rejections
        uint32_t rejectRunStart;
        uint32_t lastCandidateTs;    ///< last sample that reached the filter (accepted or not)
        bool haveCandidate;
    };

    void beginUpdate(const NavigationInput& in);
    void classifyBaro(const NavigationInput& in, Candidate* c, uint32_t& n);
    void classifyAccel(const NavigationInput& in, Candidate* c, uint32_t& n);
    void classifyGnss(const NavigationInput& in, Candidate* c, uint32_t& n);
    void initializationStep(const NavigationInput& in, Candidate* c, uint32_t n);
    bool tryFitAndStart(const NavigationInput& in, const Candidate* accel);
    void processCandidate(const NavigationInput& in, const Candidate& c);
    bool predictTo(uint32_t targetMs, const NavigationInput& in);
    void numericalFault(nav::UkfStatus s, uint32_t nowMs);
    void reanchorAltitude();
    void updateState(uint32_t nowMs);
    void finalize(const NavigationInput& in);
    void restartInit();
    MeasurementDiagnostics& diag(Source s);
    SourceTrack& track(Source s) { return tracks_[static_cast<uint8_t>(s)]; }
    bool tracking() const;
    float lateVariance(Source s, uint32_t lateMs, const ProcessModelInput& u) const;

    NavigationConfig cfg_;
    const IVerticalProcessModel& model_;
    Ukf ukf_;
    bool configValid_;

    NavState state_ = NavState::Uninitialized;
    uint32_t tf_ = 0u;               ///< filter time
    uint32_t lastAnyUpdateAt_ = 0u;
    bool haveAnyUpdate_ = false;
    bool assimilated_ = false;       ///< a measurement (or initialisation) was assimilated this update
    bool recovering_ = false;
    uint32_t recoveryConfirm_ = 0u;
    bool referenceChangedPending_ = false;
    SourceTrack tracks_[3] = {};
    BaroSampleTracker baroTracker_;
    float baroRef_[kBaroCount] = {};
    bool baroRefKnown_[kBaroCount] = {};
    uint32_t lastGnssTow_ = 0u;
    bool haveGnssTow_ = false;

    // initialisation window
    uint32_t initTs_[NavigationConfig::kInitCapacity] = {};
    float initH_[NavigationConfig::kInitCapacity] = {};
    uint32_t initCount_ = 0u;

    NavigationEstimate est_{};
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_VERTICAL_NAVIGATION_ESTIMATOR_HPP
