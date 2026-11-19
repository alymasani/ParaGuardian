/**
 ******************************************************************************
 * @file    IVerticalProcessModel.hpp
 * @brief   THE replacement boundary: how the vertical state evolves over dt.
 *
 * The UKF engine (UnscentedKalmanFilter<N>) never contains physics. It calls
 * exactly two things on its process model:
 *
 *   propagate(x, dt, u)        -> state after dt seconds (called once per
 *                                 sigma point, so it may be NONLINEAR)
 *   processNoise(dt, x, u, Q)  -> covariance of what the model cannot predict
 *
 * The production estimator (VerticalNavigationEstimator) holds a reference to
 * this 3-state interface, so a different model (e.g. a paraglider polar /
 * sink-rate model) is swapped in by constructing the estimator with another
 * object: no change to the engine, the sensor code, the measurement models or
 * any NavigationEstimate consumer. A test-only mock model proves this
 * (test_navigation_ukf.cpp: nav_process_model_swap_*).
 *
 * WHAT A 3-STATE INTERFACE CANNOT DO: a model that needs MORE states
 * (airspeed, wind, a barometer-bias state, aerodynamic coefficients) changes
 * the state dimension. The engine and maths are templated on N and do not
 * change; this interface, VerticalNavigationEstimator's state handling and
 * the covariance fields of NavigationEstimate would. See
 * VERTICAL_UKF_DESIGN.md section 6 for the exact list.
 *
 * Virtual dispatch costs 2N+1 = 7 indirect calls per prediction: negligible.
 * The destructor is protected and non-virtual: models are never deleted
 * through this interface (no heap), and a virtual destructor would pull
 * operator delete into the firmware.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_I_VERTICAL_PROCESS_MODEL_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_I_VERTICAL_PROCESS_MODEL_HPP

#include "Algorithms/Navigation/NavMath.hpp"
#include "Algorithms/Navigation/NavigationTypes.hpp"

namespace pg {

using VerticalStateVec = nav::Vec<vstate::kDim>;
using VerticalCov = nav::Mat<vstate::kDim, vstate::kDim>;

class IVerticalProcessModel {
public:
    /// Identity reported in every NavigationEstimate.
    virtual ProcessModelInfo info() const = 0;

    /// State after @p dt_s seconds (dt_s > 0). Must be finite for finite input.
    virtual VerticalStateVec propagate(const VerticalStateVec& x, float dt_s, const ProcessModelInput& u) const = 0;

    /// Process-noise covariance added over @p dt_s (symmetric, positive semi-definite).
    virtual void processNoise(float dt_s, const VerticalStateVec& x, const ProcessModelInput& u, VerticalCov& Q) const = 0;

    /// False if the model's own parameters are unusable (checked at start-up).
    virtual bool parametersValid() const = 0;

protected:
    IVerticalProcessModel() = default;
    IVerticalProcessModel(const IVerticalProcessModel&) = default;
    IVerticalProcessModel& operator=(const IVerticalProcessModel&) = default;
    ~IVerticalProcessModel() = default;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_I_VERTICAL_PROCESS_MODEL_HPP
