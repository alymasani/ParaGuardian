/**
 ******************************************************************************
 * @file    VerticalSimulator.hpp
 * @brief   Deterministic vertical-flight simulation for the navigation tests
 *          (host only; heap and std::function are fine here).
 *
 * INDEPENDENT TRUTH: the true vertical acceleration a(t) is a scenario
 * function (smooth transitions, steps, sinusoids, band-limited "turbulence",
 * or a random-jerk process). Altitude and velocity are integrated from it with
 * RK4 at 0.5 ms in DOUBLE precision. This shares nothing with the filter's
 * constant-acceleration model: except in the explicit "matched" consistency
 * scenario, the truth deliberately violates the model's assumptions.
 *
 * SENSORS (all MCU-ms timestamps, like the real acquisition):
 *   barometers  2 channels, 20 ms, read in different 10 ms cycles; white
 *               noise, optional correlated (Gauss-Markov) airflow error,
 *               common weather drift, per-channel faults, presence windows.
 *               A simple stand-in for the preprocessing vote (mean if both
 *               agree within 2 m, single = Degraded, disagreement = Ambiguous).
 *               The real voter is exercised in test_navigation_integration.cpp.
 *   accel       gravity-compensated vertical acceleration as the attitude
 *               estimator would deliver it: truth + bias + white noise +
 *               attitude-related error (scenario function / Gauss-Markov),
 *               quality states (turn gate, degraded, gap), saturation,
 *               timestamp jitter.
 *   GNSS        vertical velocity (positive up) at its own rate, latency,
 *               noise, reported sAcc, outliers, dropouts.
 * Frames are produced every framePeriodMs (optionally irregular) with
 * nowMs = sample time + 1, exactly like navigationInputFrom() would.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_SIM_VERTICAL_SIMULATOR_HPP
#define PARAGUARDIAN_TESTS_SIM_VERTICAL_SIMULATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

#include "sim/SensorSimulator.hpp"   // sim::Rng

#include "Algorithms/Navigation/BaroAlphaBetaEstimator.hpp"
#include "Algorithms/Navigation/ConstantAccelerationModel.hpp"
#include "Algorithms/Navigation/NavigationConfig.hpp"
#include "Algorithms/Navigation/VerticalNavigationEstimator.hpp"

namespace vsim {

constexpr double kPi = 3.14159265358979323846;

/// Smooth 0->1 step over [t1, t1 + T]: s = 3x^2 - 2x^3; returns ds/dt.
inline double smoothRateOfChange(double t, double t1, double T)
{
    if (t <= t1 || t >= t1 + T) return 0.0;
    const double x = (t - t1) / T;
    return (6.0 * x - 6.0 * x * x) / T;
}

/// Acceleration that changes the velocity by dv smoothly between t1 and t1 + T.
inline double velocityChange(double t, double t1, double T, double dv) { return dv * smoothRateOfChange(t, t1, T); }

struct TruthPoint {
    double h, v, a;
};

/// RK4 integration of h' = v, v' = a(t) at 0.5 ms; 1 ms samples kept in a ring (last ~16 s).
class VerticalTruth {
public:
    std::function<double(double)> accel = [](double) { return 0.0; };

    void start(double h0, double v0)
    {
        h_ = h0;
        v_ = v0;
        tMs_ = 0;
        ring_.assign(kRing, TruthPoint{0, 0, 0});
        ring_[0] = TruthPoint{h_, v_, accel(0.0)};
    }

    /// Truth at @p ms since the scenario start (must be within the last 16 s and <= the newest).
    TruthPoint at(int64_t ms)
    {
        while (tMs_ < ms) step();
        if (ms < 0) ms = 0;
        return ring_[static_cast<size_t>(ms % kRing)];
    }

private:
    static constexpr int64_t kRing = 16384;
    void step()
    {
        for (int sub = 0; sub < 2; ++sub) {
            const double t = tMs_ * 1e-3 + sub * 0.0005;
            const double dt = 0.0005;
            const double a1 = accel(t), a2 = accel(t + dt / 2), a4 = accel(t + dt);
            const double k1h = v_, k1v = a1;
            const double k2h = v_ + dt / 2 * k1v, k2v = a2;
            const double k3h = v_ + dt / 2 * k2v, k3v = a2;
            const double k4h = v_ + dt * k3v, k4v = a4;
            h_ += dt / 6 * (k1h + 2 * k2h + 2 * k3h + k4h);
            v_ += dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
        }
        ++tMs_;
        ring_[static_cast<size_t>(tMs_ % kRing)] = TruthPoint{h_, v_, accel(tMs_ * 1e-3)};
    }
    double h_ = 0, v_ = 0;
    int64_t tMs_ = 0;
    std::vector<TruthPoint> ring_;
};

/// First-order Gauss-Markov error (correlated noise) with stationary sigma and time constant tau.
struct GaussMarkov {
    double sigma = 0.0, tau = 1.0, x = 0.0;
    double step(sim::Rng& r, double dt)
    {
        if (sigma <= 0.0) return 0.0;
        const double phi = std::exp(-dt / tau);
        x = phi * x + sigma * std::sqrt(1.0 - phi * phi) * r.gaussian();
        return x;
    }
};

enum class AttQuality : uint8_t { Good = 0, TurnGate, Degraded, GapDetected, Invalid };

struct Scenario {
    const char* name = "";
    double durationS = 60.0;
    uint32_t t0Ms = 100000u;            ///< MCU time of the scenario start
    double h0 = 500.0, v0 = 0.0;
    std::function<double(double)> accel = [](double) { return 0.0; };   ///< TRUE linear vertical acceleration

    // barometers
    double baroNoise = 0.4;             ///< per channel, 1 sigma (m)
    double baroCorrSigma = 0.0, baroCorrTau = 3.0;   ///< common correlated airflow error
    double baroDrift_mps = 0.0;         ///< common weather drift
    uint32_t baroPeriodMs[2] = {20u, 20u};
    uint32_t baroPhaseMs[2] = {0u, 10u};
    std::function<double(double, int)> baroFault = [](double, int) { return 0.0; };
    std::function<bool(double, int)> baroPresent = [](double, int) { return true; };

    // acceleration (as delivered by the attitude estimator)
    double accelNoise = 0.15;
    double accelBias = 0.0;
    double accelCorrSigma = 0.0, accelCorrTau = 5.0;
    std::function<double(double)> accelError = [](double) { return 0.0; };
    std::function<AttQuality(double)> attitude = [](double) { return AttQuality::Good; };
    std::function<bool(double)> accelPresent = [](double) { return true; };
    double accelSaturation_mps2 = 1e9;  ///< |a| above this: sample invalid (saturated)
    uint32_t accelJitterMs = 0u;

    // GNSS
    bool gnss = false;
    uint32_t gnssPeriodMs = 200u;
    uint32_t gnssLatencyMs = 0u;
    double gnssNoise = 0.15;
    float gnssSAcc = 0.3f;
    double gnssOutlierProb = 0.0, gnssOutlierMag = 5.0;
    std::function<bool(double)> gnssPresent = [](double) { return true; };

    // frames
    uint32_t framePeriodMs = 10u;
    uint32_t frameJitterMs = 0u;        ///< irregular frames: period + 0..J - J/2
    uint32_t republish = 0u;            ///< extra identical frames after each frame (duplicates)
    double outOfOrderProb = 0.0;        ///< replay an OLD frame occasionally
};

/// Error statistics of one quantity.
struct ErrStats {
    std::vector<double> e;
    double sum = 0, sumSq = 0, sumSigned = 0, maxAbs = 0;
    void add(double err)
    {
        const double a = std::fabs(err);
        e.push_back(a);
        sum += a;
        sumSq += err * err;
        sumSigned += err;
        maxAbs = std::max(maxAbs, a);
    }
    size_t n() const { return e.size(); }
    double mae() const { return e.empty() ? NAN : sum / double(e.size()); }
    double rmse() const { return e.empty() ? NAN : std::sqrt(sumSq / double(e.size())); }
    double bias() const { return e.empty() ? NAN : sumSigned / double(e.size()); }
    double p95() const
    {
        if (e.empty()) return NAN;
        std::vector<double> c = e;
        const size_t k = static_cast<size_t>(0.95 * double(c.size() - 1));
        std::nth_element(c.begin(), c.begin() + static_cast<long>(k), c.end());
        return c[k];
    }
};

struct Metrics {
    ErrStats h, v, a;
    uint32_t frames = 0, usableFrames = 0;
    double neesSum = 0;
    uint32_t neesN = 0, neesIn95 = 0;
    uint32_t cover[3] = {0, 0, 0}, coverN[3] = {0, 0, 0};
    double nisSum[3] = {0, 0, 0};
    uint32_t nisN[3] = {0, 0, 0};
    double maxAsymmetry = 0.0, maxSigmaH = 0.0, minEigProxy = 1e30;
    pg::NavigationEstimate last{};
    double usable() const { return frames ? double(usableFrames) / double(frames) : 0.0; }
    double nees() const { return neesN ? neesSum / double(neesN) : NAN; }
    double nis(int i) const { return nisN[i] ? nisSum[i] / double(nisN[i]) : NAN; }
    double coverage(int i) const { return coverN[i] ? double(cover[i]) / double(coverN[i]) : NAN; }
};

/// e^T P^-1 e over the first @p n states (n = 2 or 3), P from the estimate. Returns NaN if P is singular.
inline double nees(const pg::NavigationEstimate& est, const double* e, int n)
{
    double P[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) P[i][j] = est.covariance[i][j];
    if (n == 2) {
        const double det = P[0][0] * P[1][1] - P[0][1] * P[1][0];
        if (!(det > 0)) return NAN;
        return (e[0] * (P[1][1] * e[0] - P[0][1] * e[1]) + e[1] * (-P[1][0] * e[0] + P[0][0] * e[1])) / det;
    }
    const double c00 = P[1][1] * P[2][2] - P[1][2] * P[2][1];
    const double c01 = P[0][2] * P[2][1] - P[0][1] * P[2][2];
    const double c02 = P[0][1] * P[1][2] - P[0][2] * P[1][1];
    const double det = P[0][0] * c00 + P[1][0] * c01 + P[2][0] * c02;
    if (!(det > 0)) return NAN;
    const double inv[3][3] = {{c00, c01, c02},
                              {P[1][2] * P[2][0] - P[1][0] * P[2][2], P[0][0] * P[2][2] - P[0][2] * P[2][0],
                               P[0][2] * P[1][0] - P[0][0] * P[1][2]},
                              {P[1][0] * P[2][1] - P[1][1] * P[2][0], P[0][1] * P[2][0] - P[0][0] * P[2][1],
                               P[0][0] * P[1][1] - P[0][1] * P[1][0]}};
    double s = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) s += e[i] * inv[i][j] * e[j] / det;
    return s;
}

/// Runs one scenario through the UKF (and optionally the baseline) on identical inputs.
class Runner {
public:
    Runner(const Scenario& sc, uint64_t seed, const pg::NavigationConfig& ncfg = pg::NavigationConfig{},
           const pg::BaroAlphaBetaConfig& bcfg = pg::BaroAlphaBetaConfig{})
        : sc_(sc), rng_(seed), model_(pg::ConstantAccelerationModel::fromConfig(ncfg)), ukf_(ncfg, model_), base_(bcfg)
    {
        truth_.accel = sc.accel;
        truth_.start(sc.h0, sc.v0);
        baroCorr_.sigma = sc.baroCorrSigma;
        baroCorr_.tau = sc.baroCorrTau;
        accelCorr_.sigma = sc.accelCorrSigma;
        accelCorr_.tau = sc.accelCorrTau;
    }

    /// Optional per-frame hook (after both estimators updated).
    std::function<void(double t, const pg::NavigationInput&, const pg::NavigationEstimate& u,
                       const pg::NavigationEstimate& b, const TruthPoint&)>
        each;
    double warmupS = 2.0;   ///< metrics ignore the first warmupS seconds (initialisation)
    bool runBaseline = true;

    Metrics ukf, base;

    void run()
    {
        const int64_t endMs = static_cast<int64_t>(sc_.durationS * 1000.0);
        int64_t tMs = 0;
        std::vector<pg::NavigationInput> history;
        while (tMs <= endMs) {
            pg::NavigationInput in = makeInput(tMs);
            feed(in, tMs);
            for (uint32_t r = 0; r < sc_.republish; ++r) {
                pg::NavigationInput dup = in;
                dup.nowMs += 1u + r;
                feed(dup, tMs);
            }
            if (sc_.outOfOrderProb > 0.0) {
                history.push_back(in);
                if (history.size() > 8u) history.erase(history.begin());
                if (history.size() > 4u && rng_.uniform() < sc_.outOfOrderProb) {
                    pg::NavigationInput old = history[history.size() - 4u];
                    old.nowMs = in.nowMs + 2u;   // delivered NOW, data from 30 ms ago
                    feed(old, tMs);
                    ++replayed;
                }
            }
            uint32_t period = sc_.framePeriodMs;
            if (sc_.frameJitterMs) period = period - sc_.frameJitterMs / 2u + uint32_t(rng_.next() % (sc_.frameJitterMs + 1u));
            tMs += period;
        }
    }

    uint32_t replayed = 0;
    pg::VerticalNavigationEstimator& estimator() { return ukf_; }
    pg::BaroAlphaBetaEstimator& baseline() { return base_; }
    VerticalTruth& truth() { return truth_; }
    uint32_t mcu(int64_t ms) const { return sc_.t0Ms + static_cast<uint32_t>(ms); }
    int64_t simMs(uint32_t mcuMs) const { return static_cast<int64_t>(static_cast<int32_t>(mcuMs - sc_.t0Ms)); }

private:
    struct BaroCh {
        bool have = false;
        int64_t tsMs = 0;
        double value = 0;
    };

    pg::NavigationInput makeInput(int64_t tMs)
    {
        const double t = tMs * 1e-3;
        truth_.at(tMs);   // advance the truth to this frame
        pg::NavigationInput in{};
        in.nowMs = mcu(tMs) + 1u;
        const double dtFrame = (tMs - lastFrameMs_) * 1e-3;
        lastFrameMs_ = tMs;
        const double corr = baroCorr_.step(rng_, std::max(dtFrame, 1e-3));

        // barometers: sampled on their OWN schedule (independent of frame times);
        // the newest sample due in (previous frame, this frame] is the one the frame sees.
        for (int j = 0; j < 2; ++j) {
            const int64_t per = sc_.baroPeriodMs[j];
            const int64_t ph = sc_.baroPhaseMs[j] % std::max<int64_t>(per, 1);
            if (per <= 0 || tMs < ph) continue;
            const int64_t s = tMs - ((tMs - ph) % per);
            if (s > prevFrameMs_ && sc_.baroPresent(s * 1e-3, j)) {
                const double ts = s * 1e-3;
                ch_[j].have = true;
                ch_[j].tsMs = s;
                ch_[j].value = truth_.at(s).h + sc_.baroDrift_mps * ts + corr + rng_.gaussian() * sc_.baroNoise +
                               sc_.baroFault(ts, j);
            }
        }
        uint8_t mask = 0;
        for (int j = 0; j < 2; ++j)
            if (ch_[j].have && tMs - ch_[j].tsMs <= 60 && sc_.baroPresent(t, j)) mask |= uint8_t(1u << j);
        for (int j = 0; j < 2; ++j) {
            in.baroChannelTimestampMs[j] = mcu(ch_[j].tsMs);
            in.baroReferencePressure_pa[j] = 101325.0f;
        }
        if (mask == 3u && std::fabs(ch_[0].value - ch_[1].value) > 2.0) {
            in.baroValid = false;
            in.baroStatus = pg::RedundancyStatus::Ambiguous;
        } else if (mask != 0u) {
            double sum = 0;
            int n = 0;
            int64_t newest = -1000000, oldest = 1000000000;
            for (int j = 0; j < 2; ++j)
                if (mask & (1u << j)) {
                    sum += ch_[j].value;
                    ++n;
                    newest = std::max(newest, ch_[j].tsMs);
                    oldest = std::min(oldest, ch_[j].tsMs);
                }
            in.baroValid = true;
            in.baroAltitude_m = static_cast<float>(sum / n);
            in.baroTimestampMs = mcu(newest);
            in.baroSpreadMs = static_cast<uint32_t>(newest - oldest);
            in.baroStatus = (n == 2) ? pg::RedundancyStatus::Healthy : pg::RedundancyStatus::Degraded;
        } else {
            in.baroStatus = pg::RedundancyStatus::Unavailable;
        }
        in.baroContributingMask = in.baroValid ? mask : 0u;
        in.baroSourceChanged = in.baroValid && lastMask_ != 0u && in.baroContributingMask != lastMask_;
        if (in.baroValid) lastMask_ = in.baroContributingMask;

        // acceleration (from the attitude estimator)
        const AttQuality q = sc_.attitude(t);
        const uint32_t jitter = sc_.accelJitterMs ? uint32_t(rng_.next() % (sc_.accelJitterMs + 1u)) : 0u;
        const int64_t aTs = tMs - jitter;
        const double aTrue = truth_.at(aTs).a;
        const double aCorr = accelCorr_.step(rng_, std::max(dtFrame, 1e-3));
        in.accelTimestampMs = mcu(aTs);
        in.accelValid = sc_.accelPresent(t) && q != AttQuality::Invalid &&
                        std::fabs(aTrue) <= sc_.accelSaturation_mps2;
        in.accelUp_mps2 = static_cast<float>(aTrue + sc_.accelBias + aCorr + sc_.accelError(t) +
                                             rng_.gaussian() * sc_.accelNoise);
        in.accelReliable = q == AttQuality::Good || q == AttQuality::TurnGate;
        in.attitudeState = (q == AttQuality::Degraded || q == AttQuality::GapDetected) ? pg::AttitudeState::Degraded
                                                                                        : pg::AttitudeState::Tracking6Axis;
        in.attitudeAccelCorrection = (q == AttQuality::TurnGate) ? pg::CorrectionStatus::RejectedRotation
                                                                 : pg::CorrectionStatus::Accepted;
        in.attitudeFlags = pg::attitude::AccelNotCalibrated | pg::attitude::MountingUnverified;
        if (q == AttQuality::GapDetected) in.attitudeFlags |= pg::attitude::GapDetected;
        in.imuAccelStatus = pg::RedundancyStatus::Healthy;
        in.imuGyroStatus = pg::RedundancyStatus::Healthy;
        in.model.attitudeValid = in.accelValid;
        in.model.bankAngle_rad = 0.0f;
        in.model.pitchAngle_rad = 0.0f;

        // GNSS
        if (sc_.gnss) {
            const int64_t gper = sc_.gnssPeriodMs;
            const int64_t gs = tMs - (tMs % gper);   // newest solution due in (previous frame, this frame]
            if (gs > prevFrameMs_ && sc_.gnssPresent(gs * 1e-3)) {
                gnssHave_ = true;
                gnssTs_ = gs;
                ++gnssTow_;
                double vz = truth_.at(std::max<int64_t>(0, gs - sc_.gnssLatencyMs)).v + rng_.gaussian() * sc_.gnssNoise;
                if (sc_.gnssOutlierProb > 0 && rng_.uniform() < sc_.gnssOutlierProb)
                    vz += (rng_.uniform() < 0.5 ? -1 : 1) * sc_.gnssOutlierMag;
                gnssVz_ = vz;
            }
            if (gnssHave_ && tMs - gnssTs_ <= 2000 && sc_.gnssPresent(t)) {
                in.gnssValid = true;
                in.gnssVerticalVelocity_mps = static_cast<float>(gnssVz_);
                in.gnssSpeedAccuracy_mps = sc_.gnssSAcc;
                in.gnssTimestampMs = mcu(gnssTs_);
                in.gnssTimeOfWeekMs = 1000u * gnssTow_;
                in.gnssFixType = pg::GnssFixType::Fix3D;
                in.gnssNumSatellites = 12u;
            }
        }
        prevFrameMs_ = tMs;
        return in;
    }

    void feed(const pg::NavigationInput& in, int64_t tMs)
    {
        const pg::NavigationEstimate& u = ukf_.update(in);
        const pg::NavigationEstimate& b = runBaseline ? base_.update(in) : base_.latest();
        const double t = tMs * 1e-3;
        TruthPoint tu{}, tb{};
        if (u.altitudeValid) tu = truth_.at(simMs(u.timestampMs));
        if (runBaseline && b.altitudeValid) tb = truth_.at(simMs(b.timestampMs));
        if (t >= warmupS) {
            score(ukf, u, tu, 3);
            if (runBaseline) score(base, b, tb, 2);
        }
        ukf.last = u;
        base.last = b;
        if (each) each(t, in, u, b, truth_.at(tMs));
    }

    static void score(Metrics& m, const pg::NavigationEstimate& e, const TruthPoint& tp, int dims)
    {
        ++m.frames;
        const bool usable = e.altitudeValid && e.velocityValid;
        if (usable) ++m.usableFrames;
        const double err[3] = {e.altitude_m - tp.h, e.verticalVelocity_mps - tp.v, e.verticalAccel_mps2 - tp.a};
        if (e.altitudeValid) m.h.add(err[0]);
        if (e.velocityValid) m.v.add(err[1]);
        if (e.accelerationValid) m.a.add(err[2]);
        const double sig[3] = {e.altitudeSigma_m, e.velocitySigma_mps, e.accelSigma_mps2};
        const bool valid[3] = {e.altitudeValid, e.velocityValid, e.accelerationValid};
        for (int i = 0; i < dims; ++i) {
            if (!valid[i] || !std::isfinite(sig[i])) continue;
            ++m.coverN[i];
            if (std::fabs(err[i]) <= 2.0 * sig[i]) ++m.cover[i];
        }
        if (usable && (dims == 2 || e.accelerationValid)) {
            const double x = nees(e, err, dims);
            if (std::isfinite(x)) {
                m.neesSum += x;
                ++m.neesN;
                if (x <= (dims == 3 ? 7.815 : 5.991)) ++m.neesIn95;
            }
        }
        const pg::MeasurementDiagnostics* d[3] = {&e.baro, &e.accel, &e.gnss};
        for (int i = 0; i < 3; ++i) {
            if ((d[i]->last == pg::MeasurementOutcome::Accepted || d[i]->last == pg::MeasurementOutcome::RejectedGate) &&
                std::isfinite(d[i]->nis)) {
                m.nisSum[i] += d[i]->nis;
                ++m.nisN[i];
            }
        }
        if (std::isfinite(e.altitudeSigma_m)) {
            m.maxSigmaH = std::max(m.maxSigmaH, double(e.altitudeSigma_m));
            double asym = 0;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) asym = std::max(asym, std::fabs(double(e.covariance[i][j]) - e.covariance[j][i]));
            m.maxAsymmetry = std::max(m.maxAsymmetry, asym);
            if (dims == 3) m.minEigProxy = std::min(m.minEigProxy, double(e.covariance[2][2]));
        }
    }

    Scenario sc_;
    sim::Rng rng_;
    pg::ConstantAccelerationModel model_;
    pg::VerticalNavigationEstimator ukf_;
    pg::BaroAlphaBetaEstimator base_;
    VerticalTruth truth_;
    GaussMarkov baroCorr_, accelCorr_;
    BaroCh ch_[2];
    uint8_t lastMask_ = 0;
    int64_t lastFrameMs_ = 0;
    int64_t prevFrameMs_ = -1;
    bool gnssHave_ = false;
    int64_t gnssTs_ = 0;
    uint32_t gnssTow_ = 0;
    double gnssVz_ = 0;
};

}  // namespace vsim

#endif  // PARAGUARDIAN_TESTS_SIM_VERTICAL_SIMULATOR_HPP
