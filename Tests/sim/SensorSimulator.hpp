/**
 ******************************************************************************
 * @file    SensorSimulator.hpp
 * @brief   Deterministic simulated sensor suite for preprocessing tests (host only).
 *
 * Produces pg::SensorMessage streams exactly like SensorAcquisitionTask would:
 * per-sensor rates and phases, per-sensor sequence numbers, MCU-ms
 * timestamps. Ground truth is defined in the BODY frame (FRD) as functions
 * of time; each IMU / the magnetometer converts it into its OWN axes with the
 * transpose of its mounting matrix, then adds bias and noise. Pressure is
 * generated from a true altitude with the INVERSE ISA formula
 *     P = P0 * (1 - h / 44330)^(1 / 0.1903)
 * so the pipeline's forward formula is checked against an independent
 * expression.
 *
 * Faults: invalid flag, NaN, saturation flag, extra bias (step fault),
 * "stuck" constant output, missing samples (not produced: no sequence step),
 * lost samples (produced but not delivered: sequence step = gap), delayed
 * timestamps, outliers, absent sensors, recovery (toggle at run time).
 *
 * Noise: own xorshift64* + Box-Muller, NOT <random> distributions, whose
 * output differs between standard libraries - tests must be reproducible.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_SIM_SENSOR_SIMULATOR_HPP
#define PARAGUARDIAN_TESTS_SIM_SENSOR_SIMULATOR_HPP

#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

#include "Acquisition/SensorMessage.hpp"
#include "Processing/VectorMath.hpp"

namespace sim {

using pg::Mat3f;
using pg::Vec3f;

class Rng {
public:
    explicit Rng(uint64_t seed) : s_(seed ? seed : 0x9E3779B97F4A7C15ull) {}
    uint64_t next()
    {
        s_ ^= s_ >> 12;
        s_ ^= s_ << 25;
        s_ ^= s_ >> 27;
        return s_ * 2685821657736338717ull;
    }
    /// Uniform in (0, 1].
    double uniform() { return (static_cast<double>(next() >> 11) + 1.0) * (1.0 / 9007199254740992.0); }
    /// Standard normal (Box-Muller, one value per call).
    double gaussian()
    {
        const double u1 = uniform();
        const double u2 = uniform();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(6.283185307179586 * u2);
    }
    Vec3f gaussian3(float sigma)
    {
        return Vec3f{static_cast<float>(gaussian() * sigma), static_cast<float>(gaussian() * sigma),
                     static_cast<float>(gaussian() * sigma)};
    }

private:
    uint64_t s_;
};

struct ImuModel {
    bool present = true;
    uint32_t periodMs = 10u;
    uint32_t phaseMs = 0u;
    Mat3f bodyFromSensor = pg::mat::identity();
    Vec3f gyroBias = Vec3f{0, 0, 0};       ///< sensor axes, rad/s
    Vec3f accelOffset = Vec3f{0, 0, 0};    ///< sensor axes, m/s^2
    float gyroNoise = 0.0f;                ///< 1-sigma per axis, rad/s
    float accelNoise = 0.0f;               ///< 1-sigma per axis, m/s^2
    // faults
    Vec3f gyroFaultBias = Vec3f{0, 0, 0};  ///< body-independent extra error (sensor axes)
    Vec3f accelFaultBias = Vec3f{0, 0, 0};
    bool invalidFlag = false;
    bool nanOutput = false;
    bool gyroSaturated = false;
    bool accelSaturated = false;
    double missProbability = 0.0;          ///< sample not produced (no sequence step)
    double lossProbability = 0.0;          ///< sample produced but lost (sequence gap)
    uint32_t timestampDelayMs = 0u;        ///< timestamp older than "now" by this much
    uint32_t timestampJitterMs = 0u;       ///< extra random 0..J ms subtracted per sample
    double outlierProbability = 0.0;
    float outlierMagnitude = 0.0f;
};

struct BaroModel {
    bool present = true;
    uint32_t periodMs = 20u;
    uint32_t phaseMs = 0u;
    double offsetPa = 0.0;                 ///< constant sensor offset
    double noisePa = 0.0;
    double faultPa = 0.0;                  ///< extra error (fault injection)
    bool invalidFlag = false;
    double missProbability = 0.0;
    double lossProbability = 0.0;
    uint32_t timestampDelayMs = 0u;
};

struct GenericModel {
    bool present = false;
    uint32_t periodMs = 10u;
    uint32_t phaseMs = 0u;
};

class SensorSimulator {
public:
    explicit SensorSimulator(uint64_t seed = 1u) : rng_(seed)
    {
        imu[1].phaseMs = 0u;
        baro[1].phaseMs = 10u;  // barometers read in different 10 ms cycles (realistic)
        mag.periodMs = 20u;
    }

    ImuModel imu[3];
    BaroModel baro[2];
    GenericModel mag;
    GenericModel lidar;

    /// Ground truth, body frame (FRD). Defaults: device at rest, level.
    std::function<Vec3f(uint32_t)> angularRate = [](uint32_t) { return Vec3f{0, 0, 0}; };
    std::function<Vec3f(uint32_t)> specificForce = [](uint32_t) {
        return Vec3f{0, 0, -pg::units::kStandardGravity_mps2};
    };
    std::function<double(uint32_t)> altitude = [](uint32_t) { return 0.0; };
    std::function<double(uint32_t)> referencePressure = [](uint32_t) { return 101325.0; };  ///< weather
    Vec3f magneticFieldBody = Vec3f{20.0f, 0.0f, 45.0f};  ///< uT, roughly mid-latitude

    /// Independent inverse of the ISA formula.
    static double pressureAt(double altitude_m, double p0_pa)
    {
        return p0_pa * std::pow(1.0 - altitude_m / 44330.0, 1.0 / 0.1903);
    }

    /// All messages due at @p nowMs (in the acquisition-like order IMU, baro, mag, lidar).
    std::vector<pg::SensorMessage> step(uint32_t nowMs)
    {
        if (!haveEpoch_) {
            epoch_ = nowMs;
            haveEpoch_ = true;
        }
        std::vector<pg::SensorMessage> out;
        for (int i = 0; i < 3; ++i) {
            makeImu(i, nowMs, out);
        }
        for (int j = 0; j < 2; ++j) {
            makeBaro(j, nowMs, out);
        }
        if (mag.present && due(nowMs, mag.periodMs, mag.phaseMs)) {
            pg::MagSample s{};
            s.header = header(pg::SensorId::Mag0, nowMs);
            s.field_uT = magneticFieldBody;
            s.offsetCancelled = true;
            out.push_back(pg::SensorMessage::fromMag(s));
        }
        if (lidar.present && due(nowMs, lidar.periodMs, lidar.phaseMs)) {
            pg::LidarSample s{};
            s.header = header(pg::SensorId::Lidar0, nowMs);
            s.distance_m = 12.5f;
            s.signalStrength = 1000u;
            s.temperature_c = 30.0f;
            out.push_back(pg::SensorMessage::fromLidar(s));
        }
        return out;
    }

    uint32_t sequence(pg::SensorId id) const { return seq_[static_cast<int>(id)]; }

private:
    /// Scheduling relative to the first step (wrap-safe: 2^32 is not a multiple of 10).
    bool due(uint32_t now, uint32_t period, uint32_t phase) const
    {
        return period != 0u && ((now - epoch_) % period) == phase;
    }

    pg::SampleHeader header(pg::SensorId id, uint32_t ts)
    {
        pg::SampleHeader h{};
        h.id = id;
        h.timestampMs = ts;
        h.sequence = ++seq_[static_cast<int>(id)];
        h.valid = true;
        return h;
    }

    void makeImu(int i, uint32_t now, std::vector<pg::SensorMessage>& out)
    {
        ImuModel& m = imu[i];
        if (!m.present || !due(now, m.periodMs, m.phaseMs)) {
            return;
        }
        if (m.missProbability > 0.0 && rng_.uniform() <= m.missProbability) {
            return;  // not produced
        }
        const pg::SensorId id = static_cast<pg::SensorId>(i);
        const uint32_t jitter =
            (m.timestampJitterMs != 0u) ? static_cast<uint32_t>(rng_.next() % (m.timestampJitterMs + 1u)) : 0u;
        const uint32_t ts = now - m.timestampDelayMs - jitter;
        pg::ImuSample s{};
        s.header = header(id, ts);
        const Mat3f sensorFromBody = pg::mat::transpose(m.bodyFromSensor);
        Vec3f w = pg::mat::mul(sensorFromBody, angularRate(ts));
        Vec3f f = pg::mat::mul(sensorFromBody, specificForce(ts));
        w = pg::vec::add(pg::vec::add(pg::vec::add(w, m.gyroBias), m.gyroFaultBias), rng_.gaussian3(m.gyroNoise));
        f = pg::vec::add(pg::vec::add(pg::vec::add(f, m.accelOffset), m.accelFaultBias), rng_.gaussian3(m.accelNoise));
        if (m.outlierProbability > 0.0 && rng_.uniform() <= m.outlierProbability) {
            f.x += m.outlierMagnitude;
            w.x += m.outlierMagnitude;
        }
        if (m.nanOutput) {
            w.y = std::nanf("");
        }
        s.gyro_radps = w;
        s.accel_mps2 = f;
        s.temperature_c = 25.0f;
        s.accelFresh = true;
        s.gyroFresh = true;
        s.gyroSaturated = m.gyroSaturated;
        s.accelSaturated = m.accelSaturated;
        s.header.valid = !m.invalidFlag;
        if (m.lossProbability > 0.0 && rng_.uniform() <= m.lossProbability) {
            return;  // produced (sequence consumed) but lost in transit
        }
        out.push_back(pg::SensorMessage::fromImu(s));
    }

    void makeBaro(int j, uint32_t now, std::vector<pg::SensorMessage>& out)
    {
        BaroModel& m = baro[j];
        if (!m.present || !due(now, m.periodMs, m.phaseMs)) {
            return;
        }
        if (m.missProbability > 0.0 && rng_.uniform() <= m.missProbability) {
            return;
        }
        const uint32_t ts = now - m.timestampDelayMs;
        pg::BaroSample s{};
        s.header = header(static_cast<pg::SensorId>(3 + j), ts);
        const double p = pressureAt(altitude(ts), referencePressure(ts)) + m.offsetPa + m.faultPa +
                         rng_.gaussian() * m.noisePa;
        s.pressure_pa = static_cast<float>(p);
        s.temperature_c = 20.0f;
        s.pressureInRange = s.pressure_pa >= 30000.0f && s.pressure_pa <= 125000.0f;
        s.header.valid = s.pressureInRange && !m.invalidFlag;
        if (m.lossProbability > 0.0 && rng_.uniform() <= m.lossProbability) {
            return;
        }
        out.push_back(pg::SensorMessage::fromBaro(s));
    }

    Rng rng_;
    uint32_t seq_[8] = {};
    uint32_t epoch_ = 0u;
    bool haveEpoch_ = false;
};

}  // namespace sim

#endif  // PARAGUARDIAN_TESTS_SIM_SENSOR_SIMULATOR_HPP
