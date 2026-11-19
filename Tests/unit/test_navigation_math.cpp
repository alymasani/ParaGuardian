/**
 * Vertical UKF milestone - mathematics:
 *   fixed-size matrix code, white-jerk process noise (checked against an
 *   independent numerical integration), unscented-transform weights, and the
 *   MANDATORY equivalence of the UKF with an independent double-precision
 *   linear Kalman filter (sim/LinearKalmanReference.hpp) for prediction,
 *   barometer / acceleration / GNSS-velocity updates and long mixed sequences.
 *   Also: numerical-fault detection and the alpha-beta baseline gains.
 */
#include <cmath>
#include <cstdio>

#include "TestFramework.hpp"
#include "sim/LinearKalmanReference.hpp"
#include "sim/SensorSimulator.hpp"   // sim::Rng

#include "Algorithms/Navigation/BaroAlphaBetaEstimator.hpp"
#include "Algorithms/Navigation/ConstantAccelerationModel.hpp"
#include "Algorithms/Navigation/NavMath.hpp"
#include "Algorithms/Navigation/UnscentedKalmanFilter.hpp"

using namespace pg;
using pg::nav::Mat;
using pg::nav::UkfStatus;
using pg::nav::Vec;
using Ukf3 = pg::nav::UnscentedKalmanFilter<3>;

namespace {

double relDiff(double a, double b, double scale) { return std::fabs(a - b) / std::fmax(scale, 1e-12); }

/// max relative difference between UKF (float) and reference (double), scaled per element by sqrt(Pii Pjj).
double compare(const Ukf3& u, const lkf::LinearKalmanReference& r, double* stateDiff = nullptr)
{
    double worst = 0.0;
    double worstX = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double si = std::sqrt(r.P.a[i][i]);
        worstX = std::fmax(worstX, relDiff(u.state()[static_cast<size_t>(i)], r.x[i], si));
        for (int j = 0; j < 3; ++j) {
            const double sc = std::sqrt(r.P.a[i][i] * r.P.a[j][j]);
            worst = std::fmax(worst, relDiff(u.covariance()(static_cast<size_t>(i), static_cast<size_t>(j)), r.P.a[i][j], sc));
        }
    }
    if (stateDiff) *stateDiff = worstX;
    return worst;
}

struct Pair {
    explicit Pair(float qj, const nav::UkfParams& p = nav::UkfParams{}, float qa = 0.02f) : model(qj, qa)
    {
        ukf.configure(p);
        ref.qj = qj;
        ref.qa = qa;
        Vec<3> x{{12.0f, -1.5f, 0.3f}};
        Mat<3, 3> P{{{4.0f, 0.5f, 0.0f}, {0.5f, 1.0f, 0.1f}, {0.0f, 0.1f, 0.5f}}};
        ukf.initialize(x, P);
        const double xd[3] = {12.0, -1.5, 0.3};
        lkf::M3 Pd{{{4.0, 0.5, 0.0}, {0.5, 1.0, 0.1}, {0.0, 0.1, 0.5}}};
        ref.init(xd, Pd);
    }
    ConstantAccelerationModel model;
    Ukf3 ukf;
    lkf::LinearKalmanReference ref;
    ProcessModelInput u{};

    UkfStatus update(unsigned idx, float z, float R)
    {
        const auto h = [idx](const Vec<3>& x) { Vec<1> o{}; o[0] = x[idx]; return o; };
        Vec<1> zz{{z}};
        Mat<1, 1> RR{{{R}}};
        nav::Innovation<3, 1> in{};
        UkfStatus s = ukf.evaluate(zz, RR, h, in);
        if (s != UkfStatus::Ok) return s;
        s = ukf.apply(in);
        ref.update(static_cast<int>(idx), z, R);
        return s;
    }
};

/// Model that returns NaN (to prove faults are detected, not hidden).
struct NanModel {
    Vec<3> propagate(const Vec<3>& x, float, const ProcessModelInput&) const
    {
        Vec<3> o = x;
        o[1] = std::nanf("");
        return o;
    }
    void processNoise(float, const Vec<3>&, const ProcessModelInput&, Mat<3, 3>& Q) const { Q = Mat<3, 3>::identity(); }
};

}  // namespace

TEST(nav_cholesky_reconstructs_matrix)
{
    const Mat<3, 3> P{{{4.0f, 2.0f, 0.4f}, {2.0f, 10.0f, 1.0f}, {0.4f, 1.0f, 3.0f}}};
    Mat<3, 3> L{};
    CHECK(nav::cholesky(P, L));
    const Mat<3, 3> R = nav::mul(L, nav::transpose(L));
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j) {
            CHECK_NEAR(R(i, j), P(i, j), 1e-5);
            if (j > i) CHECK(L(i, j) == 0.0f);
        }
}

TEST(nav_cholesky_rejects_indefinite_singular_and_nan)
{
    Mat<2, 2> L2{};
    CHECK(!nav::cholesky(Mat<2, 2>{{{1.0f, 2.0f}, {2.0f, 1.0f}}}, L2));    // indefinite
    CHECK(!nav::cholesky(Mat<2, 2>{{{1.0f, 1.0f}, {1.0f, 1.0f}}}, L2));    // singular
    CHECK(!nav::cholesky(Mat<2, 2>{{{std::nanf(""), 0.0f}, {0.0f, 1.0f}}}, L2));
    CHECK(!nav::cholesky(Mat<2, 2>{{{-1.0f, 0.0f}, {0.0f, 1.0f}}}, L2));
}

TEST(nav_spd_inverse_times_matrix_is_identity)
{
    const Mat<3, 3> S{{{4.0f, 2.0f, 0.4f}, {2.0f, 10.0f, 1.0f}, {0.4f, 1.0f, 3.0f}}};
    Mat<3, 3> Si{};
    CHECK(nav::spdInverse(S, Si));
    const Mat<3, 3> I = nav::mul(S, Si);
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j) CHECK_NEAR(I(i, j), i == j ? 1.0 : 0.0, 1e-5);
    Mat<1, 1> s1{{{0.25f}}}, s1i{};
    CHECK(nav::spdInverse(s1, s1i));
    CHECK_NEAR(s1i(0, 0), 4.0, 1e-6);
}

TEST(nav_white_jerk_Q_matches_independent_numerical_integration)
{
    // Q(dt) = integral_0^dt Phi(t) G q G^T Phi(t)^T dt, Phi(t) G = [t^2/2, t, 1] (white jerk, qj)
    // plus the same integral with G = [0 1 0], Phi(t) G = [t, 1, 0] (unmodelled acceleration, qa).
    // Simpson, 2000 intervals, double.
    for (double dt : {0.001, 0.01, 0.02, 0.1, 0.37, 1.0}) {
        const double q = 2.5, qa = 0.07;
        double Qn[3][3] = {};
        const int n = 2000;
        for (int k = 0; k <= n; ++k) {
            const double t = dt * k / n;
            const double w = (k == 0 || k == n) ? 1.0 : (k % 2 ? 4.0 : 2.0);
            const double g[3] = {t * t / 2, t, 1.0};
            const double ga[3] = {t, 1.0, 0.0};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) Qn[i][j] += w * (q * g[i] * g[j] + qa * ga[i] * ga[j]);
        }
        for (auto& r : Qn)
            for (double& v : r) v *= dt / (3.0 * n);
        ConstantAccelerationModel m(static_cast<float>(q), static_cast<float>(qa));
        Mat<3, 3> Q{};
        m.processNoise(static_cast<float>(dt), Vec<3>{}, ProcessModelInput{}, Q);
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j) CHECK(relDiff(Q(i, j), Qn[i][j], std::fabs(Qn[i][j])) < 2e-5);
        Mat<3, 3> L{};
        CHECK(nav::cholesky(Q, L));   // positive definite for dt > 0
    }
}

TEST(nav_constant_acceleration_propagation_is_exact)
{
    ConstantAccelerationModel m(1.0f);
    const Vec<3> x{{100.0f, 2.0f, -0.5f}};
    const Vec<3> y = m.propagate(x, 3.0f, ProcessModelInput{});
    CHECK_NEAR(y[0], 100.0 + 2.0 * 3.0 - 0.5 * 0.5 * 9.0, 1e-4);
    CHECK_NEAR(y[1], 2.0 - 0.5 * 3.0, 1e-6);
    CHECK_NEAR(y[2], -0.5, 1e-7);
    CHECK(m.parametersValid());
    CHECK(!ConstantAccelerationModel(0.0f).parametersValid());
    CHECK(!ConstantAccelerationModel(std::nanf("")).parametersValid());
    CHECK(!ConstantAccelerationModel(1.0f, -0.1f).parametersValid());
    CHECK(ConstantAccelerationModel(1.0f, 0.0f).parametersValid());
    CHECK(m.info().id == ProcessModelId::ConstantAcceleration);
}

TEST(nav_ukf_weights_sum_to_one_and_match_formulas)
{
    for (float alpha : {1.0f, 0.5f, 0.1f, 0.001f}) {
        for (float kappa : {0.0f, 1.0f}) {
            Ukf3 u;
            CHECK(u.configure(nav::UkfParams{alpha, 2.0f, kappa}));
            const double c = double(alpha) * double(alpha) * (3.0 + double(kappa));
            const double sum = double(u.weightMean0()) + 6.0 * double(u.weightI());
            CHECK(std::fabs(sum - 1.0) < 1e-3 * std::fmax(1.0, std::fabs(double(u.weightMean0()))));
            CHECK(relDiff(u.weightI(), 1.0 / (2.0 * c), 1.0 / (2.0 * c)) < 1e-5);
            CHECK(relDiff(u.weightCov0(), (c - 3.0) / c + (1.0 - double(alpha) * double(alpha) + 2.0),
                          std::fmax(1.0, std::fabs((c - 3.0) / c))) < 1e-5);
        }
    }
    Ukf3 bad;
    CHECK(!bad.configure(nav::UkfParams{0.0f, 2.0f, 0.0f}));
    CHECK(!bad.configure(nav::UkfParams{1.0f, 2.0f, -3.0f}));   // n + kappa = 0
    CHECK(!bad.parametersValid());
    ConstantAccelerationModel m(1.0f);
    Vec<3> x{};
    CHECK(!bad.initialize(x, Mat<3, 3>::identity()) || bad.predict(m, 0.01f, ProcessModelInput{}) == UkfStatus::InvalidParameters);
}

TEST(nav_sigma_points_reproduce_mean_and_covariance)
{
    Ukf3 u;
    const Vec<3> x{{5.0f, -1.0f, 0.2f}};
    const Mat<3, 3> P{{{2.0f, 0.3f, 0.1f}, {0.3f, 0.5f, 0.05f}, {0.1f, 0.05f, 0.2f}}};
    CHECK(u.initialize(x, P));
    Vec<3> X[7];
    CHECK(u.sigmaPoints(X));
    double m[3] = {};
    for (int i = 0; i < 7; ++i)
        for (int r = 0; r < 3; ++r) m[r] += (i == 0 ? u.weightMean0() : u.weightI()) * X[i][static_cast<size_t>(r)];
    double C[3][3] = {};
    for (int i = 0; i < 7; ++i)
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                C[r][c] += (i == 0 ? u.weightCov0() : u.weightI()) * (X[i][static_cast<size_t>(r)] - m[r]) *
                           (X[i][static_cast<size_t>(c)] - m[c]);
    for (int r = 0; r < 3; ++r) {
        CHECK_NEAR(m[r], x[static_cast<size_t>(r)], 1e-5);
        for (int c = 0; c < 3; ++c) CHECK_NEAR(C[r][c], P(static_cast<size_t>(r), static_cast<size_t>(c)), 1e-5);
    }
}

TEST(nav_ukf_prediction_matches_linear_kf)
{
    for (double dt : {0.001, 0.01, 0.02, 0.1, 0.5}) {
        Pair p(1.0f);
        CHECK(p.ukf.predict(p.model, static_cast<float>(dt), p.u) == UkfStatus::Ok);
        p.ref.predict(dt);
        double dx = 0.0;
        const double dP = compare(p.ukf, p.ref, &dx);
        CHECK(dP < 1e-5);
        CHECK(dx < 1e-5);
    }
}

TEST(nav_ukf_each_measurement_update_matches_linear_kf)
{
    struct Case { unsigned idx; float z; float R; const char* name; };
    const Case cases[] = {{0u, 13.1f, 0.25f, "barometer altitude"},
                          {2u, -0.4f, 0.1225f, "vertical acceleration"},
                          {1u, -1.1f, 0.09f, "GNSS vertical velocity"}};
    for (const Case& k : cases) {
        Pair p(1.0f);
        p.ukf.predict(p.model, 0.02f, p.u);
        p.ref.predict(0.02);
        CHECK(p.update(k.idx, k.z, k.R) == UkfStatus::Ok);
        double dx = 0.0;
        const double dP = compare(p.ukf, p.ref, &dx);
        std::printf("    [metric] UKF vs linear KF, %s update: state diff %.2e sigma, covariance diff %.2e (relative)\n",
                    k.name, dx, dP);
        CHECK(dP < 1e-5);
        CHECK(dx < 1e-5);
    }
}

TEST(nav_ukf_long_mixed_sequence_matches_linear_kf)
{
    // 20 000 steps of random dt (1..40 ms) with barometer (50 Hz), acceleration (100 Hz)
    // and GNSS velocity (5 Hz) updates, for several unscented-transform parameter sets.
    const nav::UkfParams sets[] = {{1.0f, 2.0f, 0.0f}, {0.5f, 2.0f, 0.0f}, {1.0f, 0.0f, 1.0f}, {0.1f, 2.0f, 0.0f},
                                   {0.001f, 2.0f, 0.0f}};
    double xDiff[5] = {};
    UkfStatus st[5] = {};
    int s = 0;
    for (const nav::UkfParams& prm : sets) {
        Pair p(0.8f, prm);
        sim::Rng rng(77);
        double worstP = 0.0, worstX = 0.0;
        UkfStatus last = UkfStatus::Ok;
        for (int k = 0; k < 20000 && last == UkfStatus::Ok; ++k) {
            const double dt = 0.001 * double(1 + rng.next() % 40);
            last = p.ukf.predict(p.model, static_cast<float>(dt), p.u);
            p.ref.predict(dt);
            const double t = 0.01 * k;
            if (k % 2 == 0 && last == UkfStatus::Ok) last = p.update(0u, float(20.0 * std::sin(0.05 * t) + rng.gaussian() * 0.5), 0.25f);
            if (last == UkfStatus::Ok) last = p.update(2u, float(0.05 * std::cos(0.05 * t) + rng.gaussian() * 0.35), 0.1225f);
            if (k % 20 == 0 && last == UkfStatus::Ok) last = p.update(1u, float(std::cos(0.05 * t) + rng.gaussian() * 0.2), 0.04f);
            double dx = 0.0;
            worstP = std::fmax(worstP, compare(p.ukf, p.ref, &dx));
            worstX = std::fmax(worstX, dx);
        }
        std::printf("    [metric] UKF(alpha=%.3g beta=%.3g kappa=%.3g) vs linear KF, 20000 mixed steps: status %d, "
                    "max state diff %.2e sigma, max covariance diff %.2e\n",
                    double(prm.alpha), double(prm.beta), double(prm.kappa), int(last), worstX, worstP);
        xDiff[s] = worstX;
        st[s] = last;
        ++s;
        if (prm.alpha >= 0.5f) {   // the configurations recommended for float
            CHECK(last == UkfStatus::Ok);
            CHECK(worstX < 2e-3);
            CHECK(worstP < 2e-3);
        }
    }
    // alpha = 0.1: Wm0 = -99. Same mathematics, but each mean is a difference of
    // large weighted terms, so float rounding grows ~|Wm0|. The deviation must
    // still be small and must GROW as alpha shrinks (precision, not a maths error).
    CHECK(st[3] == UkfStatus::Ok);
    CHECK(xDiff[3] < 0.05);
    CHECK(xDiff[3] > xDiff[1] && xDiff[1] > 0.0);
}

TEST(nav_ukf_numerical_faults_detected_and_state_kept)
{
    Pair p(1.0f);
    const Vec<3> x0 = p.ukf.state();
    const Mat<3, 3> P0 = p.ukf.covariance();
    CHECK(p.ukf.predict(NanModel{}, 0.01f, p.u) == UkfStatus::NonFiniteSigmaPoint);
    CHECK(p.ukf.state()[0] == x0[0] && p.ukf.covariance()(1, 1) == P0(1, 1));   // nothing committed
    CHECK(p.ukf.predict(p.model, 0.0f, p.u) == UkfStatus::InvalidInput);
    CHECK(p.ukf.predict(p.model, -0.01f, p.u) == UkfStatus::InvalidInput);

    // Negative measurement variance -> innovation covariance not positive definite.
    const auto h = [](const Vec<3>& x) { Vec<1> o{}; o[0] = x[0]; return o; };
    nav::Innovation<3, 1> in{};
    CHECK(p.ukf.evaluate(Vec<1>{{1.0f}}, Mat<1, 1>{{{-100.0f}}}, h, in) == UkfStatus::InnovationNotPositiveDefinite);
    CHECK(p.ukf.evaluate(Vec<1>{{std::nanf("")}}, Mat<1, 1>{{{1.0f}}}, h, in) == UkfStatus::InvalidInput);

    // A crafted innovation that would make P indefinite is refused, not applied.
    CHECK(p.ukf.evaluate(Vec<1>{{12.0f}}, Mat<1, 1>{{{0.25f}}}, h, in) == UkfStatus::Ok);
    in.S(0, 0) = 1e-6f;
    in.Sinv(0, 0) = 1e6f;
    CHECK(p.ukf.apply(in) == UkfStatus::ResultNotPositiveDefinite);
    CHECK(p.ukf.covariance()(0, 0) == P0(0, 0));

    // Initialisation refuses a non-positive-definite covariance.
    Ukf3 u;
    CHECK(!u.initialize(Vec<3>{}, Mat<3, 3>{{{1, 2, 0}, {2, 1, 0}, {0, 0, 1}}}));
    CHECK(!u.initialized());
    CHECK(u.predict(p.model, 0.01f, p.u) == UkfStatus::NotInitialized);
}

TEST(nav_alpha_beta_gains_equal_kalman_steady_state)
{
    // Kalata's closed form vs iterating the Riccati equation of the 2-state
    // constant-velocity Kalman filter (piecewise-constant white acceleration).
    for (double T : {0.01, 0.02, 0.1}) {
        const double sa = 1.0, sz = 0.5;
        double P[2][2] = {{100, 0}, {0, 100}};
        double K0 = 0, K1 = 0;
        for (int k = 0; k < 20000; ++k) {
            const double Pp00 = P[0][0] + 2 * T * P[0][1] + T * T * P[1][1] + sa * sa * T * T * T * T / 4;
            const double Pp01 = P[0][1] + T * P[1][1] + sa * sa * T * T * T / 2;
            const double Pp11 = P[1][1] + sa * sa * T * T;
            const double S = Pp00 + sz * sz;
            K0 = Pp00 / S;
            K1 = Pp01 / S;
            P[0][0] = (1 - K0) * Pp00;
            P[0][1] = (1 - K0) * Pp01;
            P[1][1] = Pp11 - K1 * Pp01;
        }
        float a = 0, b = 0;
        BaroAlphaBetaEstimator::gains(float(sa), float(sz), float(T), a, b);
        CHECK(relDiff(a, K0, K0) < 1e-3);
        CHECK(relDiff(b / T, K1, K1) < 1e-3);
        // Posterior covariance formulas used for the baseline's reported sigmas.
        CHECK(relDiff(double(a) * sz * sz, P[0][0], P[0][0]) < 1e-3);
        const double pvv = double(b) * (double(a) - double(b) / 2) / (1 - double(a)) * sz * sz / (T * T);
        CHECK(relDiff(pvv, P[1][1], P[1][1]) < 2e-3);
    }
}
