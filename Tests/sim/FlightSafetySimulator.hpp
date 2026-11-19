/**
 ******************************************************************************
 * @file    FlightSafetySimulator.hpp
 * @brief   Deterministic flight + terrain + sensor simulation for the
 *          flight-state / ground-clearance / landing-safety tests (host only).
 *
 * INDEPENDENT TRUTH
 *   - Kinematics: vertical speed, ground speed, heading, roll, pitch are
 *     scenario functions of time; position is integrated in double at 1 ms.
 *     The device cannot go below the terrain + groundSensorHeight (standing
 *     pilot / harness on the ground).
 *   - Terrain: an arbitrary height function z = terrain(x, y) (flat, slope,
 *     steps, cliffs, vegetation patches).
 *   - LiDAR range: the TRUE beam (body vector rotated by the TRUE attitude
 *     with this file's own Euler-matrix code, NOT the firmware quaternion
 *     library) is marched through the terrain in double (2 cm steps) and the
 *     hit refined by bisection. No hit within maxRange = "out of range"
 *     (rejected sample, like the driver). Range noise follows the TF02-Pro
 *     manual (6 cm below 6 m, 1 % above).
 *   - Navigation / attitude / GNSS / IMU inputs are generated from the truth
 *     with noise, outages and quality states, in the exact SafetyInput layout
 *     that safetyInputFrom() produces in firmware.
 * Synthetic scenarios are classes of motion; they are NOT recordings of real
 * paraglider flights.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_SIM_FLIGHT_SAFETY_SIMULATOR_HPP
#define PARAGUARDIAN_TESTS_SIM_FLIGHT_SAFETY_SIMULATOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

#include "sim/SensorSimulator.hpp"   // sim::Rng

#include "Algorithms/Attitude/Quaternion.hpp"
#include "Algorithms/Safety/SafetyMonitor.hpp"

namespace fsim {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

struct V3 {
    double x, y, z;
};

/// Smooth 0 -> 1 between t1 and t1 + T (value, not rate).
inline double ramp(double t, double t1, double T)
{
    if (t <= t1) return 0.0;
    if (t >= t1 + T) return 1.0;
    const double x = (t - t1) / T;
    return x * x * (3.0 - 2.0 * x);
}

/// TRUE beam direction in ENU from body FRD vector b and Z-Y-X Euler angles vs NED (own matrices, double).
inline V3 beamEnuTruth(const V3& b, double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch),
                 cy = std::cos(yaw), sy = std::sin(yaw);
    // R_ned_from_body = Rz(yaw) Ry(pitch) Rx(roll)
    const double R[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                            {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                            {-sp, cp * sr, cp * cr}};
    const double n[3] = {R[0][0] * b.x + R[0][1] * b.y + R[0][2] * b.z, R[1][0] * b.x + R[1][1] * b.y + R[1][2] * b.z,
                         R[2][0] * b.x + R[2][1] * b.y + R[2][2] * b.z};
    return V3{n[1], n[0], -n[2]};   // NED -> ENU
}

using Terrain = std::function<double(double, double)>;

/// First intersection of the ray P + s D (s > 0, |D| = 1) with z = terrain(x, y); returns s or -1.
inline double rayTerrain(const V3& P, const V3& D, const Terrain& terrain, double maxRange)
{
    auto f = [&](double s) { return P.z + s * D.z - terrain(P.x + s * D.x, P.y + s * D.y); };
    if (f(0.0) <= 0.0) return 0.0;
    double s0 = 0.0;
    const double step = 0.02;
    for (double s = step; s <= maxRange + step; s += step) {
        if (f(s) <= 0.0) {
            double a = s0, b = s;
            for (int i = 0; i < 40; ++i) {
                const double m = 0.5 * (a + b);
                if (f(m) > 0.0) a = m;
                else b = m;
            }
            const double hit = 0.5 * (a + b);
            return hit <= maxRange ? hit : -1.0;
        }
        s0 = s;
    }
    return -1.0;
}

struct Scenario {
    const char* name = "";
    double durationS = 60.0;
    uint32_t t0Ms = 200000u;
    double h0 = 1.0;                  ///< initial device height (terrain at 0 is 0 unless the terrain says otherwise)
    double groundSensorHeight = 1.0;  ///< device height above the surface when on the ground
    // kinematics (true)
    std::function<double(double)> vz = [](double) { return 0.0; };
    std::function<double(double)> groundSpeed = [](double) { return 0.0; };
    std::function<double(double)> heading = [](double) { return 0.0; };   ///< rad from north (x = north)
    std::function<double(double)> rollDeg = [](double) { return 0.0; };
    std::function<double(double)> pitchDeg = [](double) { return 0.0; };
    Terrain terrain = [](double, double) { return 0.0; };
    // motion indicators (true magnitudes of linear acceleration / rotation rate noise)
    std::function<double(double)> motionAccel = [](double) { return 0.05; };
    std::function<double(double)> motionRate = [](double) { return 0.01; };
    // navigation
    double navVzNoise = 0.12, navAltNoise = 0.3, navVzSigma = 0.15, navAltSigma = 0.3;
    std::function<pg::NavState(double)> navState = [](double) { return pg::NavState::Tracking; };
    std::function<double(double)> navVzBias = [](double) { return 0.0; };
    std::function<double(double)> navVzSigmaFn = nullptr;   ///< overrides navVzSigma when set
    std::function<double(double)> navAltOffset = [](double) { return 0.0; };   ///< e.g. a barometric discontinuity
    // attitude
    std::function<bool(double)> attitudeValid = [](double) { return true; };
    std::function<pg::AttitudeState(double)> attitudeState = [](double) { return pg::AttitudeState::Tracking6Axis; };
    double attitudeRollErrorDeg = 0.0, attitudePitchErrorDeg = 0.0;
    // LiDAR
    bool lidar = true;
    V3 beamBodyTruth{0.0, 0.0, 1.0};
    double lidarMaxRange = 40.0;
    std::function<bool(double)> lidarPresent = [](double) { return true; };
    std::function<bool(double)> lidarWeak = [](double) { return false; };
    std::function<double(double)> lidarSpike = [](double) { return 0.0; };   ///< added to the range (outliers)
    // GNSS
    bool gnss = true;
    std::function<bool(double)> gnssPresent = [](double) { return true; };
    double gnssNoise = 0.2;
    // truth labels (for metrics): airborne = device more than airborneMargin above the ground contact height
    double airborneMargin = 0.3;
};

/// Records every published event (test sink).
struct Recorder final : pg::ISafetyEventSink {
    std::vector<pg::SafetyEvent> events;
    bool publish(const pg::SafetyEvent& e) override
    {
        events.push_back(e);
        return true;
    }
    size_t count(pg::SafetyEventType t, pg::SafetyTransition tr) const
    {
        return static_cast<size_t>(std::count_if(events.begin(), events.end(), [&](const pg::SafetyEvent& e) {
            return e.type == t && e.transition == tr;
        }));
    }
    const pg::SafetyEvent* first(pg::SafetyEventType t, pg::SafetyTransition tr) const
    {
        for (const auto& e : events)
            if (e.type == t && e.transition == tr) return &e;
        return nullptr;
    }
    size_t raisedAtLeast(pg::SafetySeverity s) const
    {
        return static_cast<size_t>(std::count_if(events.begin(), events.end(), [&](const pg::SafetyEvent& e) {
            return (e.transition == pg::SafetyTransition::Raised || e.transition == pg::SafetyTransition::Escalated) &&
                   e.severity >= s;
        }));
    }
};

struct Truth {
    double t, x, y, h, vz, terrainBelow, clearanceBelow;   ///< clearanceBelow = h - terrain(x, y)
    double rangeTrue;            ///< -1 = no return within range
    double illuminatedClearance; ///< h - z(hit point) (what a perfect sensor + flat assumption reports); NaN if no hit
    V3 beamEnu;
    bool airborne;
};

struct Stat {
    std::vector<double> e;
    void add(double v) { e.push_back(v); }
    size_t n() const { return e.size(); }
    double maxAbs() const
    {
        double m = 0;
        for (double v : e) m = std::max(m, std::fabs(v));
        return m;
    }
    double mean() const
    {
        double s = 0;
        for (double v : e) s += v;
        return e.empty() ? NAN : s / double(e.size());
    }
    double rms() const
    {
        double s = 0;
        for (double v : e) s += v * v;
        return e.empty() ? NAN : std::sqrt(s / double(e.size()));
    }
    double p95() const
    {
        if (e.empty()) return NAN;
        std::vector<double> a;
        for (double v : e) a.push_back(std::fabs(v));
        std::sort(a.begin(), a.end());
        return a[static_cast<size_t>(0.95 * double(a.size() - 1))];
    }
};

class Runner {
public:
    explicit Runner(const Scenario& sc, uint64_t seed = 1u, const pg::SafetyConfig& cfg = pg::SafetyConfig{})
        : sc_(sc), rng_(seed), mon_(cfg)
    {
        mon_.addSink(&rec);
    }

    Recorder rec;
    std::function<void(double t, const pg::SafetyInput&, const pg::SafetySnapshot&, const Truth&)> each;
    // metrics
    std::vector<std::pair<double, pg::FlightPhase>> phases;   ///< (time, new phase)
    Stat clearanceErrIlluminated;   ///< estimate - truth vertical distance to the illuminated point
    Stat clearanceErrBelow;         ///< estimate - truth clearance directly beneath
    Stat rateErr;                   ///< closure-rate estimate - truth (where valid)
    uint32_t frames = 0, clearanceValidFrames = 0, truthHitFrames = 0;
    pg::SafetySnapshot last{};

    pg::SafetyMonitor& monitor() { return mon_; }

    void run()
    {
        const int64_t endMs = static_cast<int64_t>(sc_.durationS * 1000.0);
        x_ = 0.0;
        y_ = 0.0;
        h_ = std::max(sc_.h0, sc_.terrain(0.0, 0.0) + sc_.groundSensorHeight);
        pg::FlightPhase lastPhase = pg::FlightPhase::Unknown;
        for (int64_t tMs = 0; tMs <= endMs; tMs += 10) {
            advance(tMs);
            const Truth tr = truth(tMs);
            const pg::SafetyInput in = makeInput(tMs, tr);
            const pg::SafetySnapshot& s = mon_.update(in);
            ++frames;
            if (s.phase != lastPhase) {
                phases.emplace_back(tMs * 1e-3, s.phase);
                lastPhase = s.phase;
            }
            if (tr.rangeTrue > 0) ++truthHitFrames;
            const pg::GroundClearance& gc = mon_.clearance();
            if (gc.clearanceValid && (gc.flags & pg::clearflag::NotNew) == 0u) {
                ++clearanceValidFrames;
                if (std::isfinite(lastIlluminated_)) clearanceErrIlluminated.add(gc.clearance_m - lastIlluminated_);
                clearanceErrBelow.add(gc.clearance_m - tr.clearanceBelow);
                if (gc.closureRateValid && std::isfinite(truthRate_)) rateErr.add(gc.closureRate_mps - truthRate_);
            }
            last = s;
            if (each) each(tMs * 1e-3, in, s, tr);
        }
    }

    /// Time of the first entry into @p p (or -1).
    double firstPhase(pg::FlightPhase p, double after = -1.0) const
    {
        for (const auto& e : phases)
            if (e.second == p && e.first > after) return e.first;
        return -1.0;
    }
    size_t phaseChanges() const { return phases.size(); }

private:
    void advance(int64_t tMs)
    {
        while (simMs_ < tMs) {
            const double t = simMs_ * 1e-3, dt = 0.001;
            const double gs = sc_.groundSpeed(t), hd = sc_.heading(t);
            x_ += gs * std::cos(hd) * dt;
            y_ += gs * std::sin(hd) * dt;
            const double floor = sc_.terrain(x_, y_) + sc_.groundSensorHeight;
            const double hNew = std::max(floor, h_ + sc_.vz(t) * dt);
            vzTrue_ = (hNew - h_) / dt;
            h_ = hNew;
            ++simMs_;
        }
    }

    Truth truth(int64_t tMs)
    {
        const double t = tMs * 1e-3;
        Truth tr{};
        tr.t = t;
        tr.x = x_;
        tr.y = y_;
        tr.h = h_;
        tr.vz = vzTrue_;
        tr.terrainBelow = sc_.terrain(x_, y_);
        tr.clearanceBelow = h_ - tr.terrainBelow;
        tr.airborne = tr.clearanceBelow > sc_.groundSensorHeight + sc_.airborneMargin;
        tr.beamEnu = beamEnuTruth(sc_.beamBodyTruth, sc_.rollDeg(t) * kDeg, sc_.pitchDeg(t) * kDeg, sc_.heading(t));
        // ENU frame: x east, y north. Our horizontal x is "north", y "east".
        const V3 P{y_, x_, h_};
        auto terrEnu = [this](double e, double n) { return sc_.terrain(n, e); };
        tr.rangeTrue = rayTerrain(P, tr.beamEnu, terrEnu, sc_.lidarMaxRange);
        tr.illuminatedClearance = tr.rangeTrue > 0 ? -tr.rangeTrue * tr.beamEnu.z : NAN;
        // truth closure rate of the illuminated clearance (finite difference over 50 ms)
        if (std::isfinite(tr.illuminatedClearance) && std::isfinite(prevIllum_) && tMs - prevIllumMs_ <= 60) {
            truthRate_ = -(tr.illuminatedClearance - prevIllum_) / ((tMs - prevIllumMs_) * 1e-3);
        } else {
            truthRate_ = NAN;
        }
        if (tMs % 50 == 0) {
            prevIllum_ = tr.illuminatedClearance;
            prevIllumMs_ = tMs;
        }
        lastIlluminated_ = tr.illuminatedClearance;
        return tr;
    }

    pg::SafetyInput makeInput(int64_t tMs, const Truth& tr)
    {
        const double t = tMs * 1e-3;
        const uint32_t ts = sc_.t0Ms + static_cast<uint32_t>(tMs);
        pg::SafetyInput in{};
        in.nowMs = ts + 1u;
        in.frameQualityFlags = pg::quality::MountingUnverified | pg::quality::AccelNotCalibrated;

        // navigation (UKF-like output)
        const pg::NavState ns = sc_.navState(t);
        const bool navUp = ns == pg::NavState::Tracking || ns == pg::NavState::Degraded || ns == pg::NavState::Recovering;
        in.nav.state = ns;
        in.nav.estimator = pg::NavEstimatorId::VerticalUkf;
        in.nav.timestampMs = ts;
        in.nav.sequence = static_cast<uint32_t>(tMs / 10);
        in.nav.qualityFlags = pg::navflag::ModelPlaceholder | pg::navflag::AccelNotCalibrated;
        const double sigV = sc_.navVzSigmaFn ? sc_.navVzSigmaFn(t) : sc_.navVzSigma;
        in.nav.altitudeValid = navUp;
        in.nav.velocityValid = navUp && sigV <= 2.0;
        in.nav.accelerationValid = navUp;
        in.nav.altitude_m = navUp ? static_cast<float>(tr.h + sc_.navAltOffset(t) + rng_.gaussian() * sc_.navAltNoise) : NAN;
        in.nav.verticalVelocity_mps = navUp ? static_cast<float>(tr.vz + sc_.navVzBias(t) + rng_.gaussian() * sc_.navVzNoise) : NAN;
        in.nav.verticalAccel_mps2 = navUp ? 0.0f : NAN;
        in.nav.altitudeSigma_m = navUp ? static_cast<float>(sc_.navAltSigma) : NAN;
        in.nav.velocitySigma_mps = navUp ? static_cast<float>(sigV) : NAN;

        // attitude (q from the TRUE angles + configured error, via the firmware's conversion)
        const bool attOk = sc_.attitudeValid(t);
        in.attitude.orientationValid = attOk;
        in.attitude.state = attOk ? sc_.attitudeState(t) : pg::AttitudeState::Invalid;
        in.attitude.timestampMs = ts;
        in.attitude.qualityFlags = pg::attitude::AccelNotCalibrated | pg::attitude::MountingUnverified;
        in.attitude.q = pg::quat::fromEulerNedZyx(static_cast<float>((sc_.rollDeg(t) + sc_.attitudeRollErrorDeg) * kDeg),
                                                  static_cast<float>((sc_.pitchDeg(t) + sc_.attitudePitchErrorDeg) * kDeg),
                                                  static_cast<float>(sc_.heading(t)));
        const double ma = sc_.motionAccel(t), mr = sc_.motionRate(t);
        in.attitude.linearAccelValid = attOk;
        in.attitude.linearAccelNorm_mps2 = static_cast<float>(std::fabs(ma * (1.0 + 0.5 * rng_.gaussian())));
        in.attitude.linearAccelTimestampMs = ts;
        in.imu.valid = true;
        in.imu.timestampMs = ts;
        in.imu.rateNorm_radps = static_cast<float>(std::fabs(mr * (1.0 + 0.5 * rng_.gaussian())));

        // LiDAR (driver + preprocessing behaviour: rejected samples do not replace the accepted one)
        if (sc_.lidar && sc_.lidarPresent(t)) {
            if (sc_.lidarWeak(t)) {
                rejReasons_ = pg::reject::LidarWeakSignal;
                rejTs_ = ts;
            } else if (tr.rangeTrue < 0) {
                rejReasons_ = pg::reject::LidarOutOfRange;
                rejTs_ = ts;
            } else {
                const double sig = std::max(0.06, 0.01 * tr.rangeTrue);
                accDist_ = static_cast<float>(tr.rangeTrue + sig * rng_.gaussian() + sc_.lidarSpike(t));
                accTs_ = ts;
                accHave_ = true;
            }
        }
        in.lidar.lastRejectReasons = rejReasons_;
        in.lidar.lastRejectTimestampMs = rejTs_;
        if (accHave_) {
            in.lidar.distance_m = accDist_;
            in.lidar.signalStrength = 900u;
            in.lidar.timestampMs = accTs_;
            in.lidar.valid = (in.nowMs - accTs_) <= 50u;
        }

        // GNSS 5 Hz (ground speed only is used)
        if (sc_.gnss && sc_.gnssPresent(t)) {
            if (tMs % 200 == 0 || !gnssHave_) {
                gnssSpeed_ = std::max(0.0, sc_.groundSpeed(t) + rng_.gaussian() * sc_.gnssNoise);
                gnssTs_ = ts;
                gnssHave_ = true;
            }
            in.gnss.valid = true;
            in.gnss.groundSpeed_mps = static_cast<float>(gnssSpeed_);
            in.gnss.speedAccuracy_mps = 0.3f;
            in.gnss.timestampMs = gnssTs_;
            in.gnss.fixType = pg::GnssFixType::Fix3D;
            in.gnss.numSatellites = 12u;
        } else {
            gnssHave_ = false;
        }
        return in;
    }

    Scenario sc_;
    sim::Rng rng_;
    pg::SafetyMonitor mon_;
    int64_t simMs_ = 0;
    double x_ = 0, y_ = 0, h_ = 0, vzTrue_ = 0;
    double prevIllum_ = NAN, lastIlluminated_ = NAN, truthRate_ = NAN;
    int64_t prevIllumMs_ = -1000;
    uint32_t rejReasons_ = 0u, rejTs_ = 0u, accTs_ = 0u;
    float accDist_ = 0.0f;
    bool accHave_ = false;
    bool gnssHave_ = false;
    double gnssSpeed_ = 0.0;
    uint32_t gnssTs_ = 0u;
};

}  // namespace fsim

#endif  // PARAGUARDIAN_TESTS_SIM_FLIGHT_SAFETY_SIMULATOR_HPP
