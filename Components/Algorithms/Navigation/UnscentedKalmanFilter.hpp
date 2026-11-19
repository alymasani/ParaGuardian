/**
 ******************************************************************************
 * @file    UnscentedKalmanFilter.hpp
 * @brief   Generic, allocation-free Unscented Kalman Filter engine.
 *
 * Scaled unscented transform (Julier 2002; Wan & van der Merwe 2000):
 *
 *   c      = alpha^2 (n + kappa)          (= n + lambda)
 *   lambda = c - n
 *   sigma points  X0 = x,  X(i) = x + sqrt(c) L_i,  X(i+n) = x - sqrt(c) L_i
 *                 where P = L L^T (Cholesky), L_i = column i of L
 *   weights       Wm0 = lambda / c,  Wc0 = lambda / c + (1 - alpha^2 + beta)
 *                 Wm(i) = Wc(i) = 1 / (2c)   for i = 1..2n
 *
 * PREDICT  Y(i) = f(X(i), dt);  x- = sum Wm Y;  P- = sum Wc (Y - x-)(Y - x-)^T + Q(dt)
 * UPDATE   sigma points are REDRAWN from (x-, P-); Z(i) = h(X(i));
 *          z^ = sum Wm Z;  S = sum Wc (Z - z^)(Z - z^)^T + R;
 *          Pxz = sum Wc (X - x-)(Z - z^)^T;  K = Pxz S^-1;
 *          x+ = x- + K (z - z^);  P+ = P- - K S K^T
 *
 * The engine knows nothing about altitude, barometers or paragliders. The
 * process model is any object with
 *     Vec<N> propagate(const Vec<N>& x, float dt, const Input& u) const
 *     void   processNoise(float dt, const Vec<N>& x, const Input& u, Mat<N,N>& Q) const
 * (IVerticalProcessModel satisfies this for N = 3), and a measurement model is
 * any callable  Vec<M> h(const Vec<N>& x).
 *
 * Numerical policy: every result (sigma points, mean, covariance) is checked
 * for finiteness, and the covariance for positive definiteness by an actual
 * Cholesky factorisation, BEFORE it replaces the current state. On failure
 * the previous state is kept, the call returns the reason, and the CALLER
 * decides what to do (the navigation estimator reports NumericalFault). The
 * engine never silently resets or "repairs" the covariance; the only
 * correction applied is symmetrisation (P + P^T)/2, which removes float
 * rounding asymmetry and cannot hide a real fault.
 *
 * Means are accumulated as Y0 + sum Wm (Y(i) - Y0) (the weights sum to 1),
 * which avoids cancellation when the state is large and the spread small.
 *
 * Header-only because it is a template; memory = O(N^2 + N(2N+1)) floats,
 * all inside the object (N = 3: ~170 bytes). No heap, no exceptions.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_UNSCENTED_KALMAN_FILTER_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_UNSCENTED_KALMAN_FILTER_HPP

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "Algorithms/Navigation/NavMath.hpp"

namespace pg {
namespace nav {

/// Scaled-unscented-transform parameters. NOT the Madgwick gain (also called beta).
struct UkfParams {
    float alpha = 1.0f;   ///< sigma-point spread (0 < alpha <= 1)
    float beta = 2.0f;    ///< prior-distribution term (2 is optimal for a Gaussian)
    float kappa = 0.0f;   ///< secondary scaling (n + kappa = 3 for n = 3 matches Gaussian kurtosis)
};

enum class UkfStatus : uint8_t {
    Ok = 0,
    NotInitialized,
    InvalidParameters,          ///< n + lambda <= 0 or non-finite weights
    InvalidInput,               ///< dt <= 0 / non-finite measurement or noise
    CovarianceNotPositiveDefinite,  ///< Cholesky of the CURRENT covariance failed (cannot draw sigma points)
    NonFiniteSigmaPoint,        ///< the model returned NaN/Inf
    NonFiniteResult,            ///< mean or covariance not finite
    ResultNotPositiveDefinite,  ///< the NEW covariance failed Cholesky (not applied)
    InnovationNotPositiveDefinite,
};

/// Result of evaluating a measurement against the current state (nothing applied yet).
template <size_t N, size_t M>
struct Innovation {
    Vec<M> predicted;   ///< z^
    Vec<M> residual;    ///< nu = z - z^
    Mat<M, M> S;        ///< innovation covariance
    Mat<M, M> Sinv;
    Mat<N, M> Pxz;      ///< state-measurement cross-covariance
    float nis;          ///< nu^T S^-1 nu (normalised innovation squared)
};

template <size_t N>
class UnscentedKalmanFilter {
public:
    static constexpr size_t kStates = N;
    static constexpr size_t kSigmaPoints = 2u * N + 1u;

    UnscentedKalmanFilter() { configure(UkfParams{}); }

    /// Computes the weights. Returns false (and refuses to run) for unusable parameters.
    bool configure(const UkfParams& p)
    {
        params_ = p;
        const float n = static_cast<float>(N);
        c_ = p.alpha * p.alpha * (n + p.kappa);
        const float lambda = c_ - n;
        paramsValid_ = std::isfinite(c_) && c_ > 0.0f && std::isfinite(p.beta) && p.alpha > 0.0f;
        if (!paramsValid_) return false;
        wm0_ = lambda / c_;
        wc0_ = wm0_ + (1.0f - p.alpha * p.alpha + p.beta);
        wi_ = 1.0f / (2.0f * c_);
        sqrtC_ = std::sqrt(c_);
        paramsValid_ = std::isfinite(wm0_) && std::isfinite(wc0_) && std::isfinite(wi_);
        return paramsValid_;
    }

    bool parametersValid() const { return paramsValid_; }
    const UkfParams& params() const { return params_; }
    float weightMean0() const { return wm0_; }
    float weightCov0() const { return wc0_; }
    float weightI() const { return wi_; }

    /// Sets the state. Returns false (state unchanged) if P is not finite and positive definite.
    bool initialize(const Vec<N>& x, const Mat<N, N>& P)
    {
        Mat<N, N> Ps = P;
        symmetrize(Ps);
        Mat<N, N> L{};
        if (!allFinite(x) || !allFinite(Ps) || !cholesky(Ps, L)) return false;
        x_ = x;
        P_ = Ps;
        initialized_ = true;
        return true;
    }

    void clear() { initialized_ = false; }
    bool initialized() const { return initialized_; }
    const Vec<N>& state() const { return x_; }
    const Mat<N, N>& covariance() const { return P_; }

    /// Generates the 2N+1 sigma points of (x, P). False if P is not positive definite.
    bool sigmaPoints(Vec<N> (&X)[kSigmaPoints]) const { return makeSigma(x_, P_, X); }

    template <class Model, class Input>
    UkfStatus predict(const Model& model, float dt, const Input& u)
    {
        if (!initialized_) return UkfStatus::NotInitialized;
        if (!paramsValid_) return UkfStatus::InvalidParameters;
        if (!(dt > 0.0f) || !std::isfinite(dt)) return UkfStatus::InvalidInput;

        Vec<N> X[kSigmaPoints];
        if (!makeSigma(x_, P_, X)) return UkfStatus::CovarianceNotPositiveDefinite;
        Vec<N> Y[kSigmaPoints];
        for (size_t i = 0; i < kSigmaPoints; ++i) {
            Y[i] = model.propagate(X[i], dt, u);
            if (!allFinite(Y[i])) return UkfStatus::NonFiniteSigmaPoint;
        }
        const Vec<N> mean = weightedMean(Y);
        Mat<N, N> P{};
        for (size_t i = 0; i < kSigmaPoints; ++i) {
            const Vec<N> d = sub(Y[i], mean);
            accumulateOuter(P, d, d, (i == 0) ? wc0_ : wi_);
        }
        Mat<N, N> Q{};
        model.processNoise(dt, X[0], u, Q);
        if (!allFinite(Q)) return UkfStatus::NonFiniteResult;
        P = add(P, Q);
        symmetrize(P);
        return commit(mean, P);
    }

    /**
     * Evaluates measurement @p z (noise covariance @p R, model @p h) against
     * the current state WITHOUT changing it. The caller gates on out.nis and
     * then calls apply(out), or discards it.
     */
    template <size_t M, class MeasFn>
    UkfStatus evaluate(const Vec<M>& z, const Mat<M, M>& R, const MeasFn& h, Innovation<N, M>& out) const
    {
        if (!initialized_) return UkfStatus::NotInitialized;
        if (!paramsValid_) return UkfStatus::InvalidParameters;
        if (!allFinite(z) || !allFinite(R)) return UkfStatus::InvalidInput;

        Vec<N> X[kSigmaPoints];
        if (!makeSigma(x_, P_, X)) return UkfStatus::CovarianceNotPositiveDefinite;
        Vec<M> Z[kSigmaPoints];
        for (size_t i = 0; i < kSigmaPoints; ++i) {
            Z[i] = h(X[i]);
            if (!allFinite(Z[i])) return UkfStatus::NonFiniteSigmaPoint;
        }
        const Vec<M> zhat = weightedMean(Z);
        Mat<M, M> S{};
        Mat<N, M> Pxz{};
        for (size_t i = 0; i < kSigmaPoints; ++i) {
            const float w = (i == 0) ? wc0_ : wi_;
            const Vec<M> dz = sub(Z[i], zhat);
            const Vec<N> dx = sub(X[i], x_);
            accumulateOuter(S, dz, dz, w);
            accumulateOuter(Pxz, dx, dz, w);
        }
        S = add(S, R);
        symmetrize(S);
        Mat<M, M> Sinv{};
        if (!spdInverse(S, Sinv)) return UkfStatus::InnovationNotPositiveDefinite;
        out.predicted = zhat;
        out.residual = sub(z, zhat);
        out.S = S;
        out.Sinv = Sinv;
        out.Pxz = Pxz;
        const Vec<M> sn = mul(Sinv, out.residual);
        float nis = 0.0f;
        for (size_t i = 0; i < M; ++i) nis += out.residual[i] * sn[i];
        out.nis = nis;
        return std::isfinite(nis) ? UkfStatus::Ok : UkfStatus::NonFiniteResult;
    }

    /// Applies an innovation produced by evaluate() on the CURRENT state.
    template <size_t M>
    UkfStatus apply(const Innovation<N, M>& in)
    {
        if (!initialized_) return UkfStatus::NotInitialized;
        const Mat<N, M> K = mul(in.Pxz, in.Sinv);
        const Vec<N> x = add(x_, mul(K, in.residual));
        const Mat<N, N> KSKt = mul(mul(K, in.S), transpose(K));
        Mat<N, N> P{};
        for (size_t r = 0; r < N; ++r)
            for (size_t c = 0; c < N; ++c) P.a[r][c] = P_.a[r][c] - KSKt.a[r][c];
        symmetrize(P);
        return commit(x, P);
    }

    /**
     * Adds @p variance to diagonal element @p index. Used ONLY by the explicit,
     * counted recovery policy of the caller (e.g. re-anchoring altitude after a
     * persistent barometer disagreement). Returns false for invalid arguments.
     */
    bool inflateVariance(size_t index, float variance)
    {
        if (!initialized_ || index >= N || !(variance >= 0.0f) || !std::isfinite(variance)) return false;
        P_.a[index][index] += variance;
        return true;
    }

private:
    bool makeSigma(const Vec<N>& x, const Mat<N, N>& P, Vec<N> (&X)[kSigmaPoints]) const
    {
        Mat<N, N> L{};
        if (!cholesky(P, L)) return false;
        X[0] = x;
        for (size_t i = 0; i < N; ++i) {
            for (size_t r = 0; r < N; ++r) {
                const float d = sqrtC_ * L.a[r][i];
                X[1 + i].v[r] = x.v[r] + d;
                X[1 + N + i].v[r] = x.v[r] - d;
            }
        }
        for (size_t i = 0; i < kSigmaPoints; ++i)
            if (!allFinite(X[i])) return false;
        return true;
    }

    template <size_t D>
    Vec<D> weightedMean(const Vec<D> (&Y)[kSigmaPoints]) const
    {
        Vec<D> m = Y[0];
        for (size_t r = 0; r < D; ++r) {
            float s = 0.0f;  // the Wm0 term contributes Wm0 * (Y0 - Y0) = 0
            for (size_t i = 1; i < kSigmaPoints; ++i) s += wi_ * (Y[i].v[r] - Y[0].v[r]);
            m.v[r] = Y[0].v[r] + s;
        }
        return m;
    }

    template <size_t A, size_t B>
    static void accumulateOuter(Mat<A, B>& acc, const Vec<A>& a, const Vec<B>& b, float w)
    {
        for (size_t r = 0; r < A; ++r)
            for (size_t c = 0; c < B; ++c) acc.a[r][c] += w * a.v[r] * b.v[c];
    }

    UkfStatus commit(const Vec<N>& x, const Mat<N, N>& P)
    {
        if (!allFinite(x) || !allFinite(P)) return UkfStatus::NonFiniteResult;
        Mat<N, N> L{};
        if (!cholesky(P, L)) return UkfStatus::ResultNotPositiveDefinite;
        x_ = x;
        P_ = P;
        return UkfStatus::Ok;
    }

    UkfParams params_{};
    float c_ = 3.0f;
    float sqrtC_ = 1.7320508f;
    float wm0_ = 0.0f;
    float wc0_ = 2.0f;
    float wi_ = 1.0f / 6.0f;
    bool paramsValid_ = false;
    bool initialized_ = false;
    Vec<N> x_{};
    Mat<N, N> P_{};
};

}  // namespace nav
}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_UNSCENTED_KALMAN_FILTER_HPP
