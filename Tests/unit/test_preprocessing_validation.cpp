/**
 * Preprocessing milestone - sample validation and timestamp handling.
 * (Components/Processing/SampleValidator.*, TimeBase.hpp, SequenceTracker)
 */
#include <cmath>
#include <limits>

#include "TestFramework.hpp"

#include "Acquisition/SampleFreshness.hpp"
#include "Processing/SampleValidator.hpp"
#include "Processing/TimeBase.hpp"

using namespace pg;

namespace {

SensorMessage imuMsg(SensorId id, uint32_t ts, uint32_t seq)
{
    ImuSample s{};
    s.header = SampleHeader{id, ts, seq, true};
    s.accel_mps2 = Vec3f{0.0f, 0.0f, -9.80665f};
    s.gyro_radps = Vec3f{0.001f, -0.002f, 0.0f};
    s.temperature_c = 25.0f;
    s.accelFresh = true;
    s.gyroFresh = true;
    return SensorMessage::fromImu(s);
}

SensorMessage baroMsg(SensorId id, uint32_t ts, uint32_t seq, float p = 101325.0f)
{
    BaroSample s{};
    s.header = SampleHeader{id, ts, seq, true};
    s.pressure_pa = p;
    s.temperature_c = 20.0f;
    s.pressureInRange = true;
    return SensorMessage::fromBaro(s);
}

}  // namespace

TEST(prep_validation_good_imu_sample_accepted)
{
    SampleValidator v{ValidationConfig{}};
    const ValidationResult r = v.validate(imuMsg(SensorId::Imu0, 100, 1), 105);
    CHECK(r.accepted);
    CHECK(r.accelUsable);
    CHECK(r.gyroUsable);
    CHECK_EQ(r.reasons, 0u);
    CHECK_EQ(r.ageMs, 5u);
    CHECK(r.sequence == SequenceKind::First);
}

TEST(prep_validation_invalid_flag_rejected_with_reason)
{
    SampleValidator v{ValidationConfig{}};
    SensorMessage m = imuMsg(SensorId::Imu0, 100, 1);
    m.imu.header.valid = false;
    const ValidationResult r = v.validate(m, 100);
    CHECK(!r.accepted);
    CHECK(r.has(reject::FlagInvalid));
    CHECK_EQ(v.rejectedCount(SensorId::Imu0), 1u);
    CHECK_EQ(v.reasonCount(2u), 1u);  // bit 2 = FlagInvalid
    CHECK_EQ(v.lastRejectReasons(SensorId::Imu0) & reject::FlagInvalid, reject::FlagInvalid);
}

TEST(prep_validation_nan_and_inf_rejected)
{
    SampleValidator v{ValidationConfig{}};
    SensorMessage a = imuMsg(SensorId::Imu0, 100, 1);
    a.imu.gyro_radps.y = std::numeric_limits<float>::quiet_NaN();
    CHECK(v.validate(a, 100).has(reject::NonFinite));
    SensorMessage b = baroMsg(SensorId::Baro0, 100, 1, std::numeric_limits<float>::infinity());
    const ValidationResult rb = v.validate(b, 100);
    CHECK(!rb.accepted);
    CHECK(rb.has(reject::NonFinite));
    SensorMessage c = imuMsg(SensorId::Imu1, 100, 1);
    c.imu.temperature_c = std::numeric_limits<float>::quiet_NaN();
    CHECK(!v.validate(c, 100).accepted);
}

TEST(prep_validation_saturation_rejects_only_that_channel)
{
    SampleValidator v{ValidationConfig{}};
    SensorMessage m = imuMsg(SensorId::Imu0, 100, 1);
    m.imu.gyroSaturated = true;
    const ValidationResult r = v.validate(m, 100);
    CHECK(r.accepted);            // the accelerometer data is still good
    CHECK(r.accelUsable);
    CHECK(!r.gyroUsable);
    CHECK(r.has(reject::GyroSaturated));

    SensorMessage both = imuMsg(SensorId::Imu0, 110, 2);
    both.imu.gyroSaturated = true;
    both.imu.accelSaturated = true;
    const ValidationResult r2 = v.validate(both, 110);
    CHECK(!r2.accepted);          // nothing usable left
}

TEST(prep_validation_out_of_range_and_not_new_data)
{
    SampleValidator v{ValidationConfig{}};
    SensorMessage m = imuMsg(SensorId::Imu0, 100, 1);
    m.imu.accel_mps2.x = 200.0f;  // > 16 g
    m.imu.gyroFresh = false;      // status register said "no new gyro sample"
    const ValidationResult r = v.validate(m, 100);
    CHECK(r.has(reject::AccelOutOfRange));
    CHECK(r.has(reject::GyroNotNew));
    CHECK(!r.accepted);
}

TEST(prep_validation_free_fall_is_not_a_fault)
{
    // Near-zero specific force happens in free fall; it must NOT be rejected.
    SampleValidator v{ValidationConfig{}};
    SensorMessage m = imuMsg(SensorId::Imu0, 100, 1);
    m.imu.accel_mps2 = Vec3f{0.02f, -0.01f, 0.03f};
    const ValidationResult r = v.validate(m, 100);
    CHECK(r.accepted);
    CHECK(r.accelUsable);
}

TEST(prep_validation_stale_per_sensor_thresholds)
{
    SampleValidator v{ValidationConfig{}};
    // IMU limit 30 ms, barometer 60 ms: a 45 ms old sample is stale for one, fresh for the other.
    CHECK(v.validate(imuMsg(SensorId::Imu0, 1000, 1), 1045).has(reject::Stale));
    CHECK(v.validate(baroMsg(SensorId::Baro0, 1000, 1), 1045).accepted);
    CHECK(v.validate(baroMsg(SensorId::Baro0, 1100, 2), 1161).has(reject::Stale));
}

TEST(prep_validation_future_and_backwards_timestamps)
{
    SampleValidator v{ValidationConfig{}};
    // 1-2 ms ahead is tick quantisation (tolerated); 10 ms ahead is not.
    CHECK(v.validate(imuMsg(SensorId::Imu0, 102, 1), 100).accepted);
    CHECK(v.validate(imuMsg(SensorId::Imu0, 120, 2), 110).has(reject::TimestampFuture));
    // Newer sequence, but timestamp before the last ACCEPTED one (102).
    const ValidationResult r = v.validate(imuMsg(SensorId::Imu0, 95, 3), 125);
    CHECK(r.has(reject::TimestampBackwards));
    CHECK(!r.accepted);
}

TEST(prep_validation_duplicates_out_of_order_and_gaps)
{
    SampleValidator v{ValidationConfig{}};
    CHECK(v.validate(imuMsg(SensorId::Imu0, 100, 10), 100).accepted);
    const ValidationResult dup = v.validate(imuMsg(SensorId::Imu0, 100, 10), 101);
    CHECK(dup.has(reject::Duplicate));
    CHECK(dup.sequence == SequenceKind::Duplicate);
    const ValidationResult gap = v.validate(imuMsg(SensorId::Imu0, 140, 14), 141);
    CHECK(gap.accepted);
    CHECK(gap.sequence == SequenceKind::Gap);
    CHECK_EQ(gap.sequenceGap, 3u);
    const ValidationResult old = v.validate(imuMsg(SensorId::Imu0, 120, 12), 142);
    CHECK(old.has(reject::OutOfOrder));
    CHECK_EQ(v.lostCount(SensorId::Imu0), 3u);
    // Other sensors are tracked independently.
    CHECK(v.validate(imuMsg(SensorId::Imu1, 141, 10), 142).accepted);
}

TEST(prep_validation_id_type_mismatch_and_empty_message)
{
    SampleValidator v{ValidationConfig{}};
    SensorMessage m = imuMsg(SensorId::Baro0, 100, 1);  // IMU payload claiming to be a barometer
    CHECK(v.validate(m, 100).has(reject::UnknownSensor));
    SensorMessage none{};
    CHECK(v.validate(none, 100).has(reject::NoSample));
    CHECK_EQ(v.totalRejected(), 2u);
}

TEST(prep_validation_baro_lidar_mag_gnss_specific_checks)
{
    SampleValidator v{ValidationConfig{}};
    CHECK(v.validate(baroMsg(SensorId::Baro0, 100, 1, 0.0f), 100).has(reject::PressureRange));
    CHECK(v.validate(baroMsg(SensorId::Baro1, 100, 1, -5.0f), 100).has(reject::PressureRange));

    LidarSample l{};
    l.header = SampleHeader{SensorId::Lidar0, 100, 1, true};
    l.distance_m = 5.0f;
    l.weakSignal = true;
    CHECK(v.validate(SensorMessage::fromLidar(l), 100).has(reject::LidarWeakSignal));

    MagSample mg{};
    mg.header = SampleHeader{SensorId::Mag0, 100, 1, true};
    mg.field_uT = Vec3f{900.0f, 0.0f, 0.0f};
    CHECK(v.validate(SensorMessage::fromMag(mg), 100).has(reject::MagOutOfRange));

    GnssSample g{};
    g.header = SampleHeader{SensorId::Gnss0, 100, 1, true};
    g.gnssFixOk = true;
    g.positionValid = true;
    g.horizontalAccuracy_m = -1.0f;
    CHECK(v.validate(SensorMessage::fromGnss(g), 100).has(reject::GnssBadAccuracy));
}

// ---------------------------------------------------------------- time ------

TEST(prep_time_wraparound_diff_and_age)
{
    const uint32_t before = 0xFFFFFFF0u;  // 16 ms before the 49.7-day wrap
    const uint32_t after = 0x00000010u;   // 16 ms after it
    CHECK_EQ(timebase::diffMs(after, before), 32);
    CHECK_EQ(timebase::diffMs(before, after), -32);
    CHECK(timebase::isAfter(after, before));
    CHECK_EQ(timebase::ageMs(before, after), 32u);
    CHECK_EQ(timebase::ageMs(after, before), 0u);  // future -> 0, never a huge age
}

TEST(prep_time_validator_across_wrap)
{
    SampleValidator v{ValidationConfig{}};
    CHECK(v.validate(imuMsg(SensorId::Imu0, 0xFFFFFFF6u, 1), 0xFFFFFFF8u).accepted);
    const ValidationResult r = v.validate(imuMsg(SensorId::Imu0, 0x00000004u, 2), 0x00000006u);
    CHECK(r.accepted);                       // not "backwards", not stale
    CHECK_EQ(r.ageMs, 2u);
    CHECK(v.validate(imuMsg(SensorId::Imu0, 0xFFFFFFFFu, 3), 0x00000008u).has(reject::TimestampBackwards));
}

TEST(prep_time_fixed_latency_model)
{
    FixedLatencyTimeModel model;
    SampleHeader h{SensorId::Baro0, 1000u, 1u, true};
    CHECK_EQ(model.measurementTimeMs(h), 1000u);  // default: no latency assumed
    model.setLatencyMs(SensorId::Baro0, 7u);
    CHECK_EQ(model.measurementTimeMs(h), 993u);
    SampleHeader wrap{SensorId::Baro0, 3u, 2u, true};
    CHECK_EQ(model.measurementTimeMs(wrap), 0xFFFFFFFCu);  // wraps consistently
}

TEST(prep_sequence_tracker_classification_backward_compatible)
{
    SequenceTracker t;
    SampleHeader h{SensorId::Imu2, 0u, 5u, true};
    CHECK(t.observeHeader(h).kind == SequenceKind::First);
    h.sequence = 6u;
    CHECK(t.observeHeader(h).kind == SequenceKind::InOrder);
    CHECK(t.observeHeader(h).kind == SequenceKind::Duplicate);
    h.sequence = 4u;
    CHECK(t.observeHeader(h).kind == SequenceKind::OutOfOrder);
    CHECK_EQ(t.outOfOrder(SensorId::Imu2), 2u);  // duplicate + older (original meaning)
    CHECK_EQ(t.duplicates(SensorId::Imu2), 1u);
    h.sequence = 0xFFFFFFFFu;                    // far "behind" after signed compare? no: 6 -> 0xFFFFFFFF is older
    CHECK(t.observeHeader(h).kind == SequenceKind::OutOfOrder);
}
