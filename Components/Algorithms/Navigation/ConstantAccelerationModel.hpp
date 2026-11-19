/**
 ******************************************************************************
 * @file    ConstantAccelerationModel.hpp
 * @brief   TEMPORARY placeholder process model: constant acceleration driven
 *          by continuous-time white jerk.
 *
 *   h'  = h + vz dt + az dt^2 / 2
 *   vz' = vz + az dt
 *   az' = az
 *
 *   Q(dt) = qj * [ dt^5/20  dt^4/8  dt^3/6 ]
 *                [ dt^4/8   dt^3/3  dt^2/2 ]
 *                [ dt^3/6   dt^2/2  dt     ]
 *
 * qj is the power spectral density of the jerk (m^2/s^5): over one second
 * with no measurements the acceleration's standard deviation grows by
 * sqrt(qj) m/s^2. Derivation (and an independent numerical check in
 * test_navigation_math.cpp): Q = integral over [0, dt] of
 * F(t) G qj G^T F(t)^T with G = [0 0 1]^T and F(t)G = [t^2/2, t, 1]^T.
 *
 * SECOND TERM (unmodelled acceleration, qa in m^2/s^3, may be 0):
 *
 *   Q_a(dt) = qa * [ dt^3/3  dt^2/2  0 ]
 *                  [ dt^2/2  dt      0 ]
 *                  [ 0       0       0 ]
 *
 * = the same integral with G = [0 1 0]^T: white acceleration acting on
 * h and vz that the az state does NOT contain. Why it is needed: az is
 * measured at 100 Hz, so with qa = 0 the filter treats the accelerometer
 * error as white noise that averages out, and believes vz to a few cm/s.
 * Real errors are not white (uncalibrated accelerometer bias up to ~0.2 m/s^2,
 * attitude error, model error): vz then drifts while the filter stays
 * confident and gates out the barometer that would correct it (found in
 * simulation: VERTICAL_UKF_VALIDATION_REPORT.md section 6). qa keeps vz
 * correctable by the barometer. It is a model-specific tuning, so it lives
 * in the model, not in the engine.
 *
 * WHY THIS MODEL: no paraglider dynamics (polar, sink rate, lift/drag)
 * exists anywhere in this repository (searched; see VERTICAL_UKF_DESIGN.md
 * section 3). It is NOT an aerodynamic model: it knows nothing about wings,
 * sink rate or air mass. It only says "acceleration changes randomly and
 * smoothly", which is enough to fuse a barometer with an accelerometer.
 * Because it is linear, an ordinary Kalman filter is mathematically
 * equivalent; tests use that to verify the UKF.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_CONSTANT_ACCELERATION_MODEL_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_CONSTANT_ACCELERATION_MODEL_HPP

#include "Algorithms/Navigation/IVerticalProcessModel.hpp"
#include "Algorithms/Navigation/NavigationConfig.hpp"

namespace pg {

class ConstantAccelerationModel final : public IVerticalProcessModel {
public:
    static constexpr uint16_t kVersion = 1u;

    /// @param jerkPsd_m2ps5   qj, white-jerk power spectral density (m^2/s^5), > 0.
    /// @param accelPsd_m2ps3  qa, unmodelled white acceleration on h / vz (m^2/s^3), >= 0.
    explicit ConstantAccelerationModel(float jerkPsd_m2ps5, float accelPsd_m2ps3 = 0.0f)
        : qj_(jerkPsd_m2ps5), qa_(accelPsd_m2ps3)
    {
    }

    /// The model with the noise parameters of @p cfg.
    static ConstantAccelerationModel fromConfig(const NavigationConfig& cfg)
    {
        return ConstantAccelerationModel(cfg.jerkPsd_m2ps5, cfg.accelErrorPsd_m2ps3);
    }

    ProcessModelInfo info() const override { return ProcessModelInfo{ProcessModelId::ConstantAcceleration, kVersion}; }
    VerticalStateVec propagate(const VerticalStateVec& x, float dt_s, const ProcessModelInput& u) const override;
    void processNoise(float dt_s, const VerticalStateVec& x, const ProcessModelInput& u, VerticalCov& Q) const override;
    bool parametersValid() const override;

    float jerkPsd() const { return qj_; }
    float accelPsd() const { return qa_; }

private:
    float qj_;
    float qa_;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_CONSTANT_ACCELERATION_MODEL_HPP
