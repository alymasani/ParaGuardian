/**
 ******************************************************************************
 * @file    LinearKalmanReference.hpp
 * @brief   Independent reference: a textbook LINEAR Kalman filter in DOUBLE
 *          precision for the constant-acceleration model (host tests only).
 *
 * Shares no code with the UKF engine (own matrix code, own F, own Q written
 * out explicitly, Joseph-form covariance update). Because the placeholder
 * process model and every measurement model are linear, the UKF must
 * reproduce this filter's mean and covariance up to float rounding.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_SIM_LINEAR_KALMAN_REFERENCE_HPP
#define PARAGUARDIAN_TESTS_SIM_LINEAR_KALMAN_REFERENCE_HPP

#include <cmath>

namespace lkf {

struct M3 {
    double a[3][3];
};

inline M3 mul(const M3& A, const M3& B)
{
    M3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k) C.a[i][j] += A.a[i][k] * B.a[k][j];
    return C;
}

inline M3 tr(const M3& A)
{
    M3 C{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) C.a[i][j] = A.a[j][i];
    return C;
}

class LinearKalmanReference {
public:
    double x[3] = {0, 0, 0};
    M3 P{};
    double qj = 1.0;
    double qa = 0.0;   ///< unmodelled white acceleration on h / vz (m^2/s^3)

    void init(const double x0[3], const M3& P0)
    {
        for (int i = 0; i < 3; ++i) x[i] = x0[i];
        P = P0;
    }

    void predict(double dt)
    {
        const M3 F{{{1, dt, 0.5 * dt * dt}, {0, 1, dt}, {0, 0, 1}}};
        const double xn[3] = {x[0] + x[1] * dt + 0.5 * x[2] * dt * dt, x[1] + x[2] * dt, x[2]};
        for (int i = 0; i < 3; ++i) x[i] = xn[i];
        M3 Q{{{std::pow(dt, 5) / 20, std::pow(dt, 4) / 8, std::pow(dt, 3) / 6},
              {std::pow(dt, 4) / 8, std::pow(dt, 3) / 3, dt * dt / 2},
              {std::pow(dt, 3) / 6, dt * dt / 2, dt}}};
        for (auto& r : Q.a)
            for (double& v : r) v *= qj;
        Q.a[0][0] += qa * dt * dt * dt / 3;
        Q.a[0][1] += qa * dt * dt / 2;
        Q.a[1][0] += qa * dt * dt / 2;
        Q.a[1][1] += qa * dt;
        P = mul(mul(F, P), tr(F));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) P.a[i][j] += Q.a[i][j];
    }

    /// Scalar update of state component @p idx (H = e_idx). Returns NIS.
    double update(int idx, double z, double R)
    {
        const double S = P.a[idx][idx] + R;
        double K[3];
        for (int i = 0; i < 3; ++i) K[i] = P.a[i][idx] / S;
        const double nu = z - x[idx];
        for (int i = 0; i < 3; ++i) x[i] += K[i] * nu;
        // Joseph form: P = (I - K H) P (I - K H)^T + K R K^T
        M3 IKH{};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) IKH.a[i][j] = (i == j ? 1.0 : 0.0);
            IKH.a[i][idx] -= K[i];
        }
        M3 Pn = mul(mul(IKH, P), tr(IKH));
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) Pn.a[i][j] += K[i] * R * K[j];
        P = Pn;
        return nu * nu / S;
    }
};

}  // namespace lkf

#endif  // PARAGUARDIAN_TESTS_SIM_LINEAR_KALMAN_REFERENCE_HPP
