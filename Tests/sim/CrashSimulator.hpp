/**
 ******************************************************************************
 * @file    CrashSimulator.hpp
 * @brief   Deterministic incident simulation for the crash / emergency /
 *          alert tests (host only). Built ON TOP of fsim (FlightSafetySimulator),
 *          which already provides independent kinematics, terrain, LiDAR ray
 *          tracing, navigation / GNSS / attitude inputs and the real
 *          SafetyMonitor (flight phase, clearance, landing warnings).
 *
 * WHAT THIS FILE ADDS (independent of the detector code)
 *   - Ground contact is DETECTED from the truth (device reaches the terrain
 *     while descending), not scripted: impact pulses can be placed relative
 *     to the real contact time (*tc), and post-contact orientation / motion
 *     lambdas read the same *tc.
 *   - Three IMU channels sampled at their own read instants inside each
 *     10 ms cycle (3, 2, 1 ms before the frame: an impact pulse shorter than
 *     the cycle can be missed, as on the real 100 Hz polling), with
 *     sustained load factor, motion noise, half-sine impact pulses, rotation
 *     pulses, full-scale clipping (+ saturation flag), per-channel faults
 *     and dropouts.
 *   - The CrashInput in the exact layout crashInputFrom() fills in firmware
 *     (voted force = median of the unsaturated channels).
 * Scenarios are synthetic classes of motion, not recordings of incidents.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_SIM_CRASH_SIMULATOR_HPP
#define PARAGUARDIAN_TESTS_SIM_CRASH_SIMULATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "sim/FlightSafetySimulator.hpp"

#include "Algorithms/Crash/CrashDetector.hpp"
#include "Algorithms/Crash/CrashInputAdapter.hpp"

namespace csim {

constexpr double kG = 9.80665;
constexpr double kNever = 1e12;

struct Pulse {
    double t;          ///< start time (s); relative to contact if rel
    bool rel;
    double g;          ///< added |f| peak, in g
    double durMs;
    double dps;        ///< added |rate| peak
    double rateDurMs;
};

struct Scenario {
    const char* name = "";
    fsim::Scenario fs;
    std::shared_ptr<double> tc = std::make_shared<double>(kNever);   ///< detected ground contact time (s)
    double contactAfter = kNever;             ///< contact detection armed after this time
    std::vector<Pulse> pulses;
    std::function<double(double)> loadG = [](double) { return 1.0; };   ///< sustained |f| / g
    std::function<bool(double)> imuData = [](double) { return true; };
    std::function<bool(double)> chanPresent[3] = {[](double) { return true; }, [](double) { return true; },
                                                  [](double) { return true; }};
    std::function<double(double)> chanFaultG[3] = {[](double) { return 0.0; }, [](double) { return 0.0; },
                                                   [](double) { return 0.0; }};
    double fullScaleG = 16.0;
    double accelNoiseG = 0.02;
    int expectSuspicion = 0;                  ///< 1 = a suspicion is the expected outcome
    bool holdout = false;
};

inline double halfSine(double t, double start, double durS)
{
    if (t < start || t > start + durS || durS <= 0.0) return 0.0;
    return std::sin(fsim::kPi * (t - start) / durS);
}

class Runner {
public:
    Runner(const Scenario& sc, uint64_t seed = 1u, const pg::CrashConfig& cc = pg::CrashConfig{},
           const pg::SafetyConfig& sfc = pg::SafetyConfig{})
        : sc_(sc), fr_(sc.fs, seed, sfc), det_(cc), rng_(seed * 7919u + 3u)
    {
    }

    /// Per frame, after the detector ran (t s, input, status, snapshot, truth).
    std::function<void(double, const pg::CrashInput&, const pg::CrashStatus&, const pg::SafetySnapshot&,
                       const fsim::Truth&, const pg::SafetyInput&)>
        each;

    // results
    double firstCandidateT = -1.0, firstSuspectT = -1.0, contactT = -1.0;
    uint32_t candidates = 0u, suspicions = 0u, rejections = 0u;
    pg::CrashDecision firstDecision = pg::CrashDecision::None;
    pg::CrashStatus atSuspicion{};
    pg::CrashStatus last{};

    fsim::Runner& flight() { return fr_; }
    pg::CrashDetector& detector() { return det_; }

    void run()
    {
        *sc_.tc = kNever;   // a Scenario object may be run several times: contact is re-detected each run
        fr_.each = [this](double t, const pg::SafetyInput& in, const pg::SafetySnapshot& s, const fsim::Truth& tr) {
            frame(t, in, s, tr);
        };
        fr_.run();
    }

private:
    double channelForce(int i, double ts) const
    {
        double f = sc_.loadG(ts) * kG;
        const double tcv = *sc_.tc;
        for (const Pulse& p : sc_.pulses) {
            const double st = p.rel ? (tcv >= kNever ? kNever : tcv + p.t) : p.t;
            f += p.g * kG * halfSine(ts, st, p.durMs * 1e-3);
        }
        f += sc_.chanFaultG[i](ts) * kG;
        return f;
    }
    double channelRate(double ts) const
    {
        double w = 0.0;
        const double tcv = *sc_.tc;
        for (const Pulse& p : sc_.pulses) {
            const double st = p.rel ? (tcv >= kNever ? kNever : tcv + p.t) : p.t;
            w += p.dps * fsim::kDeg * halfSine(ts, st, p.rateDurMs * 1e-3);
        }
        return w;
    }

    void frame(double t, const pg::SafetyInput& in, const pg::SafetySnapshot& snap, const fsim::Truth& tr)
    {
        // Ground contact detection from the truth (independent of the detector).
        if (*sc_.tc >= kNever && t >= sc_.contactAfter && tr.clearanceBelow <= sc_.fs.groundSensorHeight + 0.02) {
            *sc_.tc = t;
            contactT = t;
        }
        pg::CrashInput ci{};
        ci.nowMs = in.nowMs;
        const bool data = sc_.imuData(t);
        float good[3];
        int ng = 0;
        double lastRate = 0.0;
        for (int i = 0; i < 3; ++i) {
            pg::CrashImuChannel& c = ci.imu[i];
            c.present = sc_.chanPresent[i](t) || everPresent_[i];
            if (sc_.chanPresent[i](t)) everPresent_[i] = true;
            if (!data || !sc_.chanPresent[i](t)) continue;
            const double ts = t - (3 - i) * 1e-3;   // own read instant inside the cycle
            double f = channelForce(i, ts) + in.attitude.linearAccelNorm_mps2 * rng_.gaussian() * 0.7 +
                       sc_.accelNoiseG * kG * rng_.gaussian();
            f = std::fabs(f);
            // own rate model: the scenario's true rotation magnitude with 10 % jitter + sensor noise
            double w = std::fabs(sc_.fs.motionRate(ts) * (1.0 + 0.1 * rng_.gaussian())) + channelRate(ts) +
                       0.003 * std::fabs(rng_.gaussian());
            const double fs = sc_.fullScaleG * kG;
            c.accelSaturated = f >= fs;
            if (f > fs) f = fs;   // clipped: a LOWER bound of the true value
            c.gyroSaturated = w >= 2000.0 * fsim::kDeg;
            if (w > 2000.0 * fsim::kDeg) w = 2000.0 * fsim::kDeg;
            c.samples = 1u;
            c.peakAccel_mps2 = static_cast<float>(f);
            c.peakRate_radps = static_cast<float>(w);
            c.peakTimestampMs = in.nowMs - static_cast<uint32_t>(4 - i);
            if (!c.accelSaturated) good[ng++] = static_cast<float>(f);
            lastRate = w;
        }
        std::sort(good, good + ng);
        ci.forceValid = ng > 0;
        ci.forceNorm_mps2 = ng > 0 ? good[ng / 2] : NAN;
        ci.rateValid = data;
        ci.rateNorm_radps = data ? static_cast<float>(lastRate) : NAN;
        ci.newImuData = data;
        ci.attitudeValid = in.attitude.orientationValid && data;
        ci.tilt_rad = ci.attitudeValid ? pg::bodyTiltFromVertical_rad(in.attitude.q) : NAN;
        ci.phase = snap.phase;
        ci.clearanceValid = snap.clearanceValid;
        ci.clearance_m = snap.clearance_m;
        ci.closureValid = snap.closureRateValid;
        ci.closureRate_mps = snap.closureRate_mps;
        ci.vzValid = in.nav.velocityValid;
        ci.vz_mps = in.nav.verticalVelocity_mps;
        ci.gnssValid = in.gnss.valid;
        ci.groundSpeed_mps = in.gnss.groundSpeed_mps;
        ci.gnssTimestampMs = in.gnss.timestampMs;

        const pg::CrashState before = det_.latest().state;
        const pg::CrashStatus& st = det_.update(ci, fr_.monitor().eventEmitter());
        if (st.state != before) {
            if (st.state == pg::CrashState::ImpactCandidate) {
                ++candidates;
                if (firstCandidateT < 0) firstCandidateT = t;
            }
            if (st.state == pg::CrashState::CrashSuspected) {
                ++suspicions;
                if (firstSuspectT < 0) {
                    firstSuspectT = t;
                    atSuspicion = st;
                }
            }
            if (st.state == pg::CrashState::Rejected) ++rejections;
            if (firstDecision == pg::CrashDecision::None &&
                (st.state == pg::CrashState::Rejected || st.state == pg::CrashState::CrashSuspected)) {
                firstDecision = st.decision;
            }
        }
        last = st;
        if (each) each(t, ci, st, snap, tr, in);
    }

    Scenario sc_;
    fsim::Runner fr_;
    pg::CrashDetector det_;
    sim::Rng rng_;
    bool everPresent_[3] = {};
};

// ---------------------------------------------------------------------------
// Scenario library. Standard flight over flat terrain: still on launch until
// `takeoff`, take-off run (4 m/s) for 3 s, climb at climbRate until climbEnd,
// glide (glideSink, glideSpeed). From crashT the crash descent crashVz applies
// until the device reaches the ground (contact detected from the truth).
// After contact: post-contact motion / tilt / walking speed.
// ---------------------------------------------------------------------------
struct Profile {
    double takeoff = 10.0, climbEnd = 33.0, climbRate = 2.5, glideSink = -1.2, glideSpeed = 10.0;
    double crashT = kNever, crashVz = -8.0;
    double flightAccel = 0.4, flightRate = 0.08;     ///< in-flight motion (linear accel m/s^2, rate rad/s)
    double postAccel = 0.05, postRate = 0.01;        ///< after contact (0.05 / 0.01 = lying still)
    double postTiltDeg = 0.0, postWalkSpeed = 0.0;
    double postRiseS = 0.0;                          ///< post motion / tilt start this long after contact
};

inline Scenario baseFlight(const char* name, const Profile& p, double duration)
{
    Scenario s;
    s.name = name;
    s.fs.name = name;
    s.fs.durationS = duration;
    s.fs.h0 = 1.0;
    s.fs.groundSensorHeight = 1.0;
    std::shared_ptr<double> tc = s.tc;
    const Profile P = p;
    auto post = [P, tc](double t) { return t >= *tc + P.postRiseS; };
    s.fs.vz = [P, tc](double t) {
        if (t >= *tc) return 0.0;
        if (t < P.takeoff + 3.0) return 0.0;
        if (t >= P.crashT) return P.crashVz;
        if (t < P.climbEnd) return P.climbRate;
        return P.glideSink;
    };
    s.fs.groundSpeed = [P, tc, post](double t) {
        if (t >= *tc) return post(t) ? P.postWalkSpeed : 0.0;
        if (t < P.takeoff) return 0.0;
        if (t < P.takeoff + 3.0) return 4.0;
        return P.glideSpeed;
    };
    s.fs.motionAccel = [P, tc, post](double t) {
        if (t >= *tc) return post(t) ? P.postAccel : 0.05;
        if (t >= P.takeoff && t < P.takeoff + 3.0) return 1.5;
        return t >= P.takeoff ? P.flightAccel : 0.05;
    };
    s.fs.motionRate = [P, tc, post](double t) {
        if (t >= *tc) return post(t) ? P.postRate : 0.01;
        return t >= P.takeoff ? P.flightRate : 0.01;
    };
    s.fs.rollDeg = [P, post](double t) { return post(t) ? P.postTiltDeg : 0.0; };
    s.contactAfter = p.crashT;
    return s;
}

/// Adds an impact pulse at the detected contact (+ offset).
inline void impactAtContact(Scenario& s, double extraG, double durMs, double dps, double rateDurMs = 120.0,
                            double offset = 0.0)
{
    s.pulses.push_back(Pulse{offset, true, extraG, durMs, dps, rateDurMs});
}

}  // namespace csim

#endif  // PARAGUARDIAN_TESTS_SIM_CRASH_SIMULATOR_HPP
