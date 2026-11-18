/**
 ******************************************************************************
 * @file    PreprocessingPipeline.cpp
 * @brief   See PreprocessingPipeline.hpp.
 ******************************************************************************
 */
#include "Processing/PreprocessingPipeline.hpp"

#include <cmath>

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {

void inc(uint32_t& v)
{
    if (v != UINT32_MAX) {
        ++v;
    }
}

void addSat(uint32_t& v, uint32_t d)
{
    v = (UINT32_MAX - v < d) ? UINT32_MAX : v + d;
}

VectorEstimate toVectorEstimate(const VoteResult& r, uint32_t nowMs)
{
    VectorEstimate e{};
    e.valid = r.valid;
    e.status = r.status;
    e.candidateMask = r.candidateMask;
    e.contributingMask = r.contributingMask;
    e.excludedMask = r.excludedMask;
    e.outlierMask = r.outlierMask;
    e.partialAgreement = r.partialAgreement;
    e.maxDisagreement = r.maxPairDifference;
    if (r.valid) {
        e.value = r.value;
        e.timestampMs = r.timestampMs;
        e.ageMs = timebase::ageMs(r.timestampMs, nowMs);
        e.sampleSpreadMs = timebase::ageMs(r.oldestTimestampMs, r.timestampMs);
    }
    return e;
}

}  // namespace

PreprocessingPipeline::PreprocessingPipeline(const PreprocessingConfig& cfg)
    : cfg_(cfg),
      validator_(cfg.validation),
      calibration_(cfg),
      accelVoter_(static_cast<uint8_t>(kImuCount), cfg.accelVoter),
      gyroVoter_(static_cast<uint8_t>(kImuCount), cfg.gyroVoter),
      baroVoter_(static_cast<uint8_t>(kBaroCount), cfg.baroVoter)
{
}

void PreprocessingPipeline::start(uint32_t nowMs)
{
    if (cfg_.session.calibrateAtStartup) {
        calibration_.request(nowMs);
    }
}

void PreprocessingPipeline::setAccelCalibration(size_t imu, const AccelCalibration& c)
{
    if (imu < kImuCount) {
        accelCal_[imu] = c;
    }
}

bool PreprocessingPipeline::fresh(SensorId id, uint32_t timestampMs, uint32_t nowMs) const
{
    // A timestamp "in the future" relative to the frame time cannot be fresh data
    // either; the validator already rejected those beyond the tolerance.
    const int32_t age = timebase::diffMs(nowMs, timestampMs);
    return age >= -static_cast<int32_t>(cfg_.validation.maxFutureMs) &&
           (age <= 0 || static_cast<uint32_t>(age) <= cfg_.validation.maxAgeMs[sensorIndex(id)]);
}

ValidationResult PreprocessingPipeline::ingest(const SensorMessage& msg, uint32_t nowMs)
{
    inc(stats_.messagesIngested);
    const ValidationResult v = validator_.validate(msg, nowMs);
    if (v.sequenceGap != 0u) {
        addSat(lostSinceFrame_, v.sequenceGap);
        addSat(stats_.samplesLost, v.sequenceGap);
    }

    const SampleHeader* h = headerOf(msg);
    const bool knownId = (h != nullptr) && !v.has(reject::UnknownSensor | reject::NoSample);
    if (knownId && v.reasons != 0u) {
        if (isImu(h->id)) imu_[imuIndex(h->id)].lastRejectReasons = v.reasons;
        if (isBaro(h->id)) baro_[baroIndex(h->id)].lastRejectReasons = v.reasons;
    }

    if (!v.accepted) {
        inc(stats_.messagesRejected);
        inc(rejectedSinceFrame_);
        calibration_.poll(nowMs);
        return v;
    }
    inc(stats_.messagesAccepted);

    switch (msg.type) {
        case SensorMessageType::Imu:
            handleImu(msg.imu, v);
            break;
        case SensorMessageType::Baro:
            handleBaro(msg.baro);
            break;
        case SensorMessageType::Mag:
            mag_.present = true;
            mag_.timestampMs = msg.mag.header.timestampMs;
            mag_.fieldBody = alignment_.toBody(SensorId::Mag0, msg.mag.field_uT);
            newOther_ = true;
            break;
        case SensorMessageType::Gnss:
            gnss_.present = true;
            gnss_.sample = msg.gnss;
            newOther_ = true;
            break;
        case SensorMessageType::Lidar:
            lidar_.present = true;
            lidar_.sample = msg.lidar;
            newOther_ = true;
            break;
        case SensorMessageType::None:
            break;
    }
    calibration_.poll(nowMs);
    return v;
}

void PreprocessingPipeline::handleImu(const ImuSample& s, const ValidationResult& v)
{
    const SensorId id = s.header.id;
    const size_t i = imuIndex(id);

    // Calibration works on RAW sensor-axis data: a bias belongs to the chip's axes.
    if (v.accelUsable && v.gyroUsable) {
        calibration_.onImu(i, s.header.timestampMs, s.gyro_radps, s.accel_mps2);
    }

    ImuState& st = imu_[i];
    st.present = true;
    st.timestampMs = s.header.timestampMs;
    st.accelUsable = v.accelUsable;
    st.gyroUsable = v.gyroUsable;

    // 1. correct in sensor axes, 2. rotate the SAME way for both channels.
    const Vec3f accelSensor = applyAccelCalibration(accelCal_[i], s.accel_mps2);
    const GyroBiasCalibrator& gc = calibration_.gyro(i);
    st.gyroCalibrated = gc.active().valid;
    const Vec3f gyroSensor = gc.correct(s.gyro_radps);
    st.accelBody = alignment_.toBody(id, accelSensor);
    st.gyroBody = alignment_.toBody(id, gyroSensor);
    newImu_ = true;
}

void PreprocessingPipeline::handleBaro(const BaroSample& s)
{
    const size_t j = baroIndex(s.header.id);
    BaroState& st = baro_[j];
    st.present = true;
    st.timestampMs = s.header.timestampMs;
    st.pressure_pa = s.pressure_pa;

    // One pow() per barometer sample (2 x 50 Hz), here in the processing task,
    // never in the acquisition task or the driver.
    const BaroReference& ref = calibration_.baro(j).active();
    if (ref.valid) {
        st.relativeAltitude_m = baro::relativeAltitude_m(s.pressure_pa, ref.referencePressure_pa, ref.scaleHeight_m);
        st.altitudeValid = std::isfinite(st.relativeAltitude_m);
        st.referenceSession = ref.sessionId;
    } else {
        st.altitudeValid = false;
    }
    calibration_.onBaro(j, s.header.timestampMs, s.pressure_pa);
    newBaro_ = true;
}

void PreprocessingPipeline::fillImu(uint32_t nowMs, ProcessedSensorFrame& f)
{
    VoteInput acc[RedundancyVoter::kMaxSensors] = {};
    VoteInput gyr[RedundancyVoter::kMaxSensors] = {};
    bool anyGyroCalibrated = false;

    for (size_t i = 0; i < kImuCount; ++i) {
        const ImuState& st = imu_[i];
        const SensorId id = imuId(i);
        ImuChannelInfo& info = f.imu[i];
        info.present = st.present;
        info.lastRejectReasons = st.lastRejectReasons;
        info.gyroCalibration = calibration_.gyro(i).state();
        anyGyroCalibrated = anyGyroCalibrated || calibration_.gyro(i).active().valid;
        if (!st.present) {
            continue;
        }
        info.timestampMs = st.timestampMs;
        info.ageMs = timebase::ageMs(st.timestampMs, nowMs);
        info.fresh = fresh(id, st.timestampMs, nowMs);
        info.accelUsable = st.accelUsable;
        info.gyroUsable = st.gyroUsable;
        info.accelBody_mps2 = st.accelBody;
        info.gyroBody_radps = st.gyroBody;

        acc[i] = VoteInput{info.fresh && st.accelUsable, st.accelBody, st.timestampMs};
        const bool gyroCalOk = st.gyroCalibrated || !cfg_.requireGyroCalibration;
        gyr[i] = VoteInput{info.fresh && st.gyroUsable && gyroCalOk, st.gyroBody, st.timestampMs};
    }

    const VoteResult a = accelVoter_.update(acc, newImu_);
    const VoteResult g = gyroVoter_.update(gyr, newImu_);
    f.specificForce_mps2 = toVectorEstimate(a, nowMs);
    f.angularRate_radps = toVectorEstimate(g, nowMs);

    // Time step for an integrating consumer (Madgwick): only between two valid
    // outputs with genuinely newer data.
    f.gyroDtValid = false;
    f.gyroDtMs = 0u;
    if (g.valid) {
        if (lastGyroValid_ && timebase::isAfter(g.timestampMs, lastGyroTimestampMs_)) {
            f.gyroDtMs = static_cast<uint32_t>(timebase::diffMs(g.timestampMs, lastGyroTimestampMs_));
            f.gyroDtValid = f.gyroDtMs <= kMaxGyroDtMs;
        }
        lastGyroValid_ = true;
        lastGyroTimestampMs_ = g.timestampMs;
    } else {
        lastGyroValid_ = false;
    }

    uint32_t q = 0u;
    if (!anyGyroCalibrated) q |= quality::GyroNotCalibrated;
    if (a.status == RedundancyStatus::Unavailable && g.status == RedundancyStatus::Unavailable) {
        q |= quality::ImuUnavailable;
    }
    if (a.status == RedundancyStatus::Ambiguous || g.status == RedundancyStatus::Ambiguous) {
        q |= quality::ImuAmbiguous;
        inc(stats_.imuAmbiguousFrames);
    }
    if (a.status == RedundancyStatus::Degraded || g.status == RedundancyStatus::Degraded) {
        q |= quality::ImuDegraded;
    }
    if (!a.valid && !g.valid) {
        inc(stats_.framesWithoutImu);
    }
    f.qualityFlags |= q;
}

void PreprocessingPipeline::fillBaro(uint32_t nowMs, ProcessedSensorFrame& f)
{
    // Only altitudes relative to references from the SAME session are compared:
    // they share one physical zero (where and when the references were taken).
    bool haveSession = false;
    uint32_t session = 0u;
    bool anyReference = false;
    for (size_t j = 0; j < kBaroCount; ++j) {
        const BaroReference& ref = calibration_.baro(j).active();
        if (ref.valid) {
            anyReference = true;
            if (!haveSession || ref.sessionId > session) {
                session = ref.sessionId;
                haveSession = true;
            }
        }
    }

    VoteInput in[RedundancyVoter::kMaxSensors] = {};
    for (size_t j = 0; j < kBaroCount; ++j) {
        const BaroState& st = baro_[j];
        const BaroReferenceCalibrator& cal = calibration_.baro(j);
        BaroChannelInfo& info = f.baro[j];
        info.present = st.present;
        info.lastRejectReasons = st.lastRejectReasons;
        info.referenceState = cal.state();
        info.referencePressure_pa = cal.active().valid ? static_cast<float>(cal.active().referencePressure_pa) : 0.0f;
        if (!st.present) {
            continue;
        }
        info.timestampMs = st.timestampMs;
        info.ageMs = timebase::ageMs(st.timestampMs, nowMs);
        info.fresh = fresh(baroId(j), st.timestampMs, nowMs);
        info.pressure_pa = st.pressure_pa;
        const bool refOk = cal.active().valid && st.altitudeValid && st.referenceSession == cal.active().sessionId &&
                           st.referenceSession == session;
        info.altitudeValid = info.fresh && refOk;
        info.relativeAltitude_m = refOk ? static_cast<float>(st.relativeAltitude_m) : 0.0f;
        in[j] = VoteInput{info.altitudeValid, Vec3f{info.relativeAltitude_m, 0.0f, 0.0f}, st.timestampMs};
    }

    const VoteResult r = baroVoter_.update(in, newBaro_);
    ScalarEstimate& e = f.baroAltitude_m;
    e.valid = r.valid;
    e.status = r.status;
    e.candidateMask = r.candidateMask;
    e.contributingMask = r.contributingMask;
    e.excludedMask = r.excludedMask;
    e.maxDisagreement = r.maxPairDifference;
    if (r.valid) {
        e.value = r.value.x;
        e.timestampMs = r.timestampMs;
        e.ageMs = timebase::ageMs(r.timestampMs, nowMs);
        e.sampleSpreadMs = timebase::ageMs(r.oldestTimestampMs, r.timestampMs);
    }

    uint32_t q = 0u;
    if (!anyReference) q |= quality::BaroNoReference;
    if (r.status == RedundancyStatus::Unavailable) q |= quality::BaroUnavailable;
    if (r.status == RedundancyStatus::Ambiguous) {
        q |= quality::BaroAmbiguous;
        inc(stats_.baroAmbiguousFrames);
    }
    if (r.status == RedundancyStatus::Degraded) q |= quality::BaroDegraded;
    if (r.valid && lastBaroValid_ && r.contributingMask != lastBaroContributing_) {
        q |= quality::BaroSourceChanged;  // a step of up to the agreement tolerance is possible
    }
    if (!r.valid) {
        inc(stats_.framesWithoutBaro);
    }
    lastBaroValid_ = r.valid;
    lastBaroContributing_ = r.contributingMask;
    f.qualityFlags |= q;
}

void PreprocessingPipeline::fillOthers(uint32_t nowMs, ProcessedSensorFrame& f)
{
    f.mag.ironCalibrated = false;
    if (mag_.present) {
        f.mag.timestampMs = mag_.timestampMs;
        f.mag.ageMs = timebase::ageMs(mag_.timestampMs, nowMs);
        f.mag.valid = fresh(SensorId::Mag0, mag_.timestampMs, nowMs);
        f.mag.fieldBody_uT = mag_.fieldBody;
    }
    f.qualityFlags |= quality::MagNotIronCalibrated;
    if (!f.mag.valid) f.qualityFlags |= quality::MagUnavailable;

    if (gnss_.present) {
        const GnssSample& s = gnss_.sample;
        GnssInfo& g = f.gnss;
        g.timestampMs = s.header.timestampMs;
        g.ageMs = timebase::ageMs(s.header.timestampMs, nowMs);
        g.valid = fresh(SensorId::Gnss0, s.header.timestampMs, nowMs);
        g.timeOfWeek_ms = s.gnssTimeOfWeek_ms;
        g.latitude_1e7deg = s.latitude_1e7deg;
        g.longitude_1e7deg = s.longitude_1e7deg;
        g.heightMsl_m = s.heightMsl_m;
        g.velocityNorth_mps = s.velocityNorth_mps;
        g.velocityEast_mps = s.velocityEast_mps;
        g.verticalVelocity_mps = s.verticalVelocity_mps;
        g.horizontalAccuracy_m = s.horizontalAccuracy_m;
        g.verticalAccuracy_m = s.verticalAccuracy_m;
        g.numSatellites = s.numSatellites;
        g.fixType = s.fixType;
    }
    if (!f.gnss.valid) f.qualityFlags |= quality::GnssUnavailable;

    if (lidar_.present) {
        const LidarSample& s = lidar_.sample;
        f.lidar.distance_m = s.distance_m;
        f.lidar.signalStrength = s.signalStrength;
        f.lidar.timestampMs = s.header.timestampMs;
        f.lidar.ageMs = timebase::ageMs(s.header.timestampMs, nowMs);
        f.lidar.valid = fresh(SensorId::Lidar0, s.header.timestampMs, nowMs);
    }
    if (!f.lidar.valid) f.qualityFlags |= quality::LidarUnavailable;
}

void PreprocessingPipeline::buildFrame(uint32_t nowMs, ProcessedSensorFrame& f)
{
    f = ProcessedSensorFrame{};
    f.sequence = ++frameSequence_;
    f.timestampMs = nowMs;
    f.bodyFrame = BodyFrame::FrdBody;
    f.timeDomain = TimeDomain::McuMillis;

    fillImu(nowMs, f);
    fillBaro(nowMs, f);
    fillOthers(nowMs, f);

    const CalibrationSessionStatus& cs = calibration_.status();
    f.calibrationState = cs.state;
    f.lastCalibrationFailure = cs.lastFailure;
    f.calibrationSessionId = cs.sessionId;

    if (!alignment_.allVerified()) f.qualityFlags |= quality::MountingUnverified;
    bool allAccelCalibrated = true;
    for (size_t i = 0; i < kImuCount; ++i) allAccelCalibrated = allAccelCalibrated && accelCal_[i].valid;
    if (!allAccelCalibrated) f.qualityFlags |= quality::AccelNotCalibrated;
    if (calibration_.collecting()) f.qualityFlags |= quality::CalibrationInProgress;
    if (lostSinceFrame_ != 0u) f.qualityFlags |= quality::SamplesLost;
    if (rejectedSinceFrame_ != 0u) f.qualityFlags |= quality::SamplesRejected;
    if (!newImu_) f.qualityFlags |= quality::NoNewImuData;
    f.samplesLostSinceLastFrame = lostSinceFrame_;
    f.samplesRejectedSinceLastFrame = rejectedSinceFrame_;

    newImu_ = newBaro_ = newOther_ = false;
    lostSinceFrame_ = 0u;
    rejectedSinceFrame_ = 0u;
    inc(stats_.framesBuilt);
}

}  // namespace pg
