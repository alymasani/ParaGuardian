/**
 ******************************************************************************
 * @file    NavMath.hpp
 * @brief   Tiny fixed-size matrix maths for the navigation filter.
 *
 * Why not Eigen: the project has no Eigen, the filter needs a handful of
 * operations on matrices no larger than N x N with N = 3 today, and the
 * firmware rules forbid heap use. Everything here is a template on the
 * dimensions, stored in plain arrays (no heap, trivially copyable), so a
 * future filter with more states (e.g. an aerodynamic model with airspeed
 * and wind) reuses the same code with a different N.
 *
 * Storage is row-major: m(r, c) = a[r][c].
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAV_MATH_HPP
#define PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAV_MATH_HPP

#include <cmath>
#include <cstddef>

namespace pg {
namespace nav {

template <size_t R, size_t C>
struct Mat {
    float a[R][C];

    float& operator()(size_t r, size_t c) { return a[r][c]; }
    float operator()(size_t r, size_t c) const { return a[r][c]; }

    static Mat zero()
    {
        Mat m{};
        return m;
    }
    static Mat identity()
    {
        Mat m{};
        for (size_t i = 0; i < R && i < C; ++i) m.a[i][i] = 1.0f;
        return m;
    }
};

/// Column vector.
template <size_t N>
struct Vec {
    float v[N];

    float& operator[](size_t i) { return v[i]; }
    float operator[](size_t i) const { return v[i]; }

    static Vec zero()
    {
        Vec x{};
        return x;
    }
};

template <size_t R, size_t K, size_t C>
inline Mat<R, C> mul(const Mat<R, K>& A, const Mat<K, C>& B)
{
    Mat<R, C> out{};
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c) {
            float s = 0.0f;
            for (size_t k = 0; k < K; ++k) s += A.a[r][k] * B.a[k][c];
            out.a[r][c] = s;
        }
    return out;
}

template <size_t R, size_t C>
inline Vec<R> mul(const Mat<R, C>& A, const Vec<C>& x)
{
    Vec<R> out{};
    for (size_t r = 0; r < R; ++r) {
        float s = 0.0f;
        for (size_t c = 0; c < C; ++c) s += A.a[r][c] * x.v[c];
        out.v[r] = s;
    }
    return out;
}

template <size_t R, size_t C>
inline Mat<C, R> transpose(const Mat<R, C>& A)
{
    Mat<C, R> out{};
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c) out.a[c][r] = A.a[r][c];
    return out;
}

template <size_t R, size_t C>
inline Mat<R, C> add(const Mat<R, C>& A, const Mat<R, C>& B)
{
    Mat<R, C> out{};
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c) out.a[r][c] = A.a[r][c] + B.a[r][c];
    return out;
}

template <size_t N>
inline Vec<N> add(const Vec<N>& a, const Vec<N>& b)
{
    Vec<N> out{};
    for (size_t i = 0; i < N; ++i) out.v[i] = a.v[i] + b.v[i];
    return out;
}

template <size_t N>
inline Vec<N> sub(const Vec<N>& a, const Vec<N>& b)
{
    Vec<N> out{};
    for (size_t i = 0; i < N; ++i) out.v[i] = a.v[i] - b.v[i];
    return out;
}

/// P = (P + P^T) / 2 (removes the asymmetry that float rounding creates).
template <size_t N>
inline void symmetrize(Mat<N, N>& P)
{
    for (size_t r = 0; r < N; ++r)
        for (size_t c = r + 1; c < N; ++c) {
            const float m = 0.5f * (P.a[r][c] + P.a[c][r]);
            P.a[r][c] = m;
            P.a[c][r] = m;
        }
}

template <size_t R, size_t C>
inline bool allFinite(const Mat<R, C>& A)
{
    for (size_t r = 0; r < R; ++r)
        for (size_t c = 0; c < C; ++c)
            if (!std::isfinite(A.a[r][c])) return false;
    return true;
}

template <size_t N>
inline bool allFinite(const Vec<N>& x)
{
    for (size_t i = 0; i < N; ++i)
        if (!std::isfinite(x.v[i])) return false;
    return true;
}

/// Largest |P(r,c) - P(c,r)| relative to the largest diagonal element.
template <size_t N>
inline float relativeAsymmetry(const Mat<N, N>& P)
{
    float maxDiag = 0.0f;
    float maxAsym = 0.0f;
    for (size_t r = 0; r < N; ++r) {
        maxDiag = std::fmax(maxDiag, std::fabs(P.a[r][r]));
        for (size_t c = r + 1; c < N; ++c) maxAsym = std::fmax(maxAsym, std::fabs(P.a[r][c] - P.a[c][r]));
    }
    return (maxDiag > 0.0f) ? maxAsym / maxDiag : maxAsym;
}

/**
 * Cholesky factorisation P = L L^T (L lower triangular).
 * Returns false (and leaves L unspecified) if P is not symmetric positive
 * definite to working precision: a pivot <= 0 or non-finite. The caller
 * decides what that means; nothing is "fixed" here.
 */
template <size_t N>
inline bool cholesky(const Mat<N, N>& P, Mat<N, N>& L)
{
    L = Mat<N, N>::zero();
    for (size_t j = 0; j < N; ++j) {
        float d = P.a[j][j];
        for (size_t k = 0; k < j; ++k) d -= L.a[j][k] * L.a[j][k];
        if (!(d > 0.0f) || !std::isfinite(d)) return false;
        const float ljj = std::sqrt(d);
        L.a[j][j] = ljj;
        for (size_t i = j + 1; i < N; ++i) {
            float s = P.a[i][j];
            for (size_t k = 0; k < j; ++k) s -= L.a[i][k] * L.a[j][k];
            L.a[i][j] = s / ljj;
        }
    }
    return true;
}

/**
 * Inverse of a symmetric positive-definite M x M matrix via Cholesky.
 * Used for the innovation covariance S (M = 1 for every measurement today,
 * where it reduces to 1/S). Returns false if S is not positive definite.
 */
template <size_t M>
inline bool spdInverse(const Mat<M, M>& S, Mat<M, M>& Sinv)
{
    Mat<M, M> L{};
    if (!cholesky(S, L)) return false;
    // Solve L Y = I, then L^T X = Y, column by column.
    for (size_t col = 0; col < M; ++col) {
        float y[M];
        for (size_t i = 0; i < M; ++i) {
            float s = (i == col) ? 1.0f : 0.0f;
            for (size_t k = 0; k < i; ++k) s -= L.a[i][k] * y[k];
            y[i] = s / L.a[i][i];
        }
        for (size_t ii = M; ii-- > 0;) {
            float s = y[ii];
            for (size_t k = ii + 1; k < M; ++k) s -= L.a[k][ii] * Sinv.a[k][col];
            Sinv.a[ii][col] = s / L.a[ii][ii];
        }
    }
    symmetrize(Sinv);
    return allFinite(Sinv);
}

}  // namespace nav
}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_NAVIGATION_NAV_MATH_HPP
