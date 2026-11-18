/**
 * Preprocessing milestone - RedundancyVoter (3 IMUs, 2 barometers).
 */
#include <cmath>

#include "TestFramework.hpp"

#include "Processing/ProcessingConfig.hpp"
#include "Processing/RedundancyVoter.hpp"

using namespace pg;

namespace {

VoterConfig imuCfg() { return PreprocessingConfig{}.accelVoter; }   // 1.0 m/s^2 + 5 %
VoterConfig baroCfg() { return PreprocessingConfig{}.baroVoter; }   // 2.0 m

using In = VoteInput[RedundancyVoter::kMaxSensors];

void set(In& in, float a, float b, float c, uint8_t availableMask = 0x7, uint32_t t = 100)
{
    const float v[3] = {a, b, c};
    for (int i = 0; i < 3; ++i) {
        in[i] = VoteInput{(availableMask & (1u << i)) != 0u, Vec3f{v[i], 0.0f, -9.8f}, t};
    }
}

}  // namespace

TEST(prep_vote_all_three_agree_healthy_mean)
{
    RedundancyVoter v(3, imuCfg());
    In in;
    set(in, 0.1f, 0.2f, 0.3f);
    const VoteResult r = v.update(in, true);
    CHECK(r.valid);
    CHECK(r.status == RedundancyStatus::Healthy);
    CHECK_EQ(r.contributingMask, 0x7u);
    CHECK_NEAR(r.value.x, 0.2, 1e-6);
    CHECK_NEAR(r.maxPairDifference, 0.2, 1e-6);
}

TEST(prep_vote_one_imu_invalid_degraded_two_contribute)
{
    RedundancyVoter v(3, imuCfg());
    In in;
    set(in, 0.1f, 0.3f, 50.0f, 0x3);  // IMU2 not available (invalid / stale)
    const VoteResult r = v.update(in, true);
    CHECK(r.valid);
    CHECK(r.status == RedundancyStatus::Degraded);
    CHECK_EQ(r.contributingMask, 0x3u);
    CHECK_NEAR(r.value.x, 0.2, 1e-6);
}

TEST(prep_vote_biased_imu_is_outlier_then_excluded_with_persistence)
{
    RedundancyVoter v(3, imuCfg());
    In in;
    set(in, 0.0f, 0.1f, 3.0f);  // IMU2 biased by 3 m/s^2
    for (int k = 1; k <= 4; ++k) {
        const VoteResult r = v.update(in, true);
        CHECK(r.valid);
        CHECK_EQ(r.outlierMask, 0x4u);           // left out of THIS sample's value...
        CHECK_EQ(r.contributingMask, 0x3u);
        CHECK_EQ(r.excludedMask, 0u);            // ...but not yet de-selected
        CHECK(r.status == RedundancyStatus::Degraded);
        CHECK_NEAR(r.value.x, 0.05, 1e-6);
    }
    const VoteResult r5 = v.update(in, true);    // 5th consecutive -> excluded
    CHECK_EQ(r5.excludedMask, 0x4u);
    CHECK(v.votedOut(2));
    CHECK_EQ(v.exclusions(2), 1u);
    CHECK_EQ(v.selectionChanges(), 1u);
}

TEST(prep_vote_single_noisy_sample_does_not_change_selection)
{
    RedundancyVoter v(3, imuCfg());
    In good;
    set(good, 0.0f, 0.0f, 0.0f);
    In spike;
    set(spike, 0.0f, 0.0f, 5.0f);
    for (int k = 0; k < 20; ++k) {
        v.update((k % 7 == 3) ? spike : good, true);  // isolated spikes
    }
    CHECK(!v.excluded(2));
    CHECK_EQ(v.selectionChanges(), 0u);
}

TEST(prep_vote_two_disagreeing_imus_ambiguous_no_output)
{
    RedundancyVoter v(3, imuCfg());
    In in;
    set(in, 0.0f, 4.0f, 0.0f, 0x3);  // only two available, and they disagree
    const VoteResult r = v.update(in, true);
    CHECK(!r.valid);
    CHECK(r.status == RedundancyStatus::Ambiguous);
    CHECK_EQ(r.contributingMask, 0u);
    CHECK_EQ(r.outlierMask, 0u);  // nobody can be blamed with two sensors
}

TEST(prep_vote_all_three_disagree_ambiguous)
{
    RedundancyVoter v(3, imuCfg());
    In in;
    set(in, 0.0f, 3.0f, 6.0f);
    const VoteResult r = v.update(in, true);
    CHECK(!r.valid);
    CHECK(r.status == RedundancyStatus::Ambiguous);
    for (int k = 0; k < 10; ++k) v.update(in, true);
    CHECK_EQ(v.selectionChanges(), 0u);  // no exclusion without majority evidence
}

TEST(prep_vote_chain_partial_agreement_uses_median)
{
    RedundancyVoter v(3, imuCfg());
    In in;
    set(in, 0.0f, 0.9f, 1.8f);  // 0-1 and 1-2 agree (<= ~1.49), 0-2 do not
    const VoteResult r = v.update(in, true);
    CHECK(r.valid);
    CHECK(r.partialAgreement);
    CHECK(r.status == RedundancyStatus::Degraded);
    CHECK_NEAR(r.value.x, 0.9, 1e-6);
    CHECK_EQ(r.outlierMask, 0u);
}

TEST(prep_vote_excluded_imu_recovers_after_hysteresis)
{
    RedundancyVoter v(3, imuCfg());
    In bad;
    set(bad, 0.0f, 0.0f, 3.0f);
    for (int k = 0; k < 5; ++k) v.update(bad, true);
    CHECK(v.excluded(2));
    In good;
    set(good, 0.0f, 0.0f, 0.1f);
    for (int k = 0; k < 19; ++k) {
        const VoteResult r = v.update(good, true);
        CHECK_EQ(r.contributingMask, 0x3u);  // still out while re-qualifying
    }
    const VoteResult r = v.update(good, true);  // 20th agreement
    CHECK(!v.excluded(2));
    CHECK_EQ(v.readmissions(2), 1u);
    const VoteResult r2 = v.update(good, true);
    CHECK_EQ(r2.contributingMask, 0x7u);
    CHECK(r2.status == RedundancyStatus::Healthy);
    (void)r;
}

TEST(prep_vote_newdata_false_does_not_count_persistence)
{
    RedundancyVoter v(3, imuCfg());
    In bad;
    set(bad, 0.0f, 0.0f, 3.0f);
    for (int k = 0; k < 50; ++k) v.update(bad, false);  // same sample re-evaluated
    CHECK(!v.excluded(2));
}

TEST(prep_vote_time_skew_excludes_old_sample_from_comparison)
{
    RedundancyVoter v(3, imuCfg());
    In in;
    set(in, 0.0f, 0.0f, 0.0f);
    in[2].timestampMs = 100 - 20;  // 20 ms older than the others (> 15 ms skew)
    const VoteResult r = v.update(in, true);
    CHECK_EQ(r.skewedMask, 0x4u);
    CHECK_EQ(r.candidateMask, 0x3u);
    CHECK(r.status == RedundancyStatus::Degraded);
    CHECK_EQ(r.timestampMs, 100u);
}

TEST(prep_vote_no_candidates_unavailable)
{
    RedundancyVoter v(3, imuCfg());
    In in;
    set(in, 0, 0, 0, 0x0);
    const VoteResult r = v.update(in, true);
    CHECK(!r.valid);
    CHECK(r.status == RedundancyStatus::Unavailable);
}

TEST(prep_vote_relative_tolerance_during_large_signals)
{
    // At 40 m/s^2 (4 g turn), 5 % of magnitude = 2 m/s^2 extra tolerance.
    RedundancyVoter v(3, imuCfg());
    In in;
    for (int i = 0; i < 3; ++i) in[i] = VoteInput{true, Vec3f{40.0f + 1.2f * static_cast<float>(i), 0, 0}, 100};
    const VoteResult r = v.update(in, true);  // pairs differ by 1.2 and 2.4 (< 1 + 0.05*42.4 = 3.1)
    CHECK(r.status == RedundancyStatus::Healthy);
}

// ------------------------------------------------------------- barometers --

TEST(prep_vote_two_baros_agree_healthy)
{
    RedundancyVoter v(2, baroCfg());
    In in = {{true, {10.2f, 0, 0}, 100}, {true, {10.9f, 0, 0}, 90}, {false, {}, 0}};
    const VoteResult r = v.update(in, true);
    CHECK(r.valid);
    CHECK(r.status == RedundancyStatus::Healthy);
    CHECK_NEAR(r.value.x, 10.55, 1e-5);
    CHECK_EQ(r.timestampMs, 100u);
    CHECK_EQ(r.oldestTimestampMs, 90u);
}

TEST(prep_vote_two_baros_disagree_ambiguous_neither_blamed)
{
    RedundancyVoter v(2, baroCfg());
    In in = {{true, {10.0f, 0, 0}, 100}, {true, {15.0f, 0, 0}, 100}, {false, {}, 0}};
    for (int k = 0; k < 30; ++k) {
        const VoteResult r = v.update(in, true);
        CHECK(!r.valid);
        CHECK(r.status == RedundancyStatus::Ambiguous);
    }
    CHECK(!v.excluded(0));
    CHECK(!v.excluded(1));
}

TEST(prep_vote_one_baro_unavailable_degraded)
{
    RedundancyVoter v(2, baroCfg());
    In in = {{true, {5.0f, 0, 0}, 100}, {false, {}, 0}, {false, {}, 0}};
    const VoteResult r = v.update(in, true);
    CHECK(r.valid);
    CHECK(r.status == RedundancyStatus::Degraded);
    CHECK_EQ(r.contributingMask, 0x1u);
}

TEST(prep_vote_returning_baro_requalifies_before_contributing)
{
    RedundancyVoter v(2, baroCfg());
    In both = {{true, {1.0f, 0, 0}, 100}, {true, {1.2f, 0, 0}, 100}, {false, {}, 0}};
    In only0 = {{true, {1.0f, 0, 0}, 100}, {false, {}, 0}, {false, {}, 0}};
    v.update(both, true);
    for (int k = 0; k < 5; ++k) v.update(only0, true);  // absent 5 updates -> must re-qualify
    CHECK(v.excluded(1));
    for (int k = 0; k < 9; ++k) {
        const VoteResult r = v.update(both, true);
        CHECK_EQ(r.contributingMask, 0x1u);
        CHECK(r.status == RedundancyStatus::Degraded);
    }
    v.update(both, true);  // 10th agreement
    const VoteResult r = v.update(both, true);
    CHECK_EQ(r.contributingMask, 0x3u);
    CHECK(r.status == RedundancyStatus::Healthy);
    // A returning barometer that DISAGREES is a 1-vs-1 conflict: no side is picked.
    RedundancyVoter w(2, baroCfg());
    w.update(both, true);
    for (int k = 0; k < 5; ++k) w.update(only0, true);
    In conflict = {{true, {1.0f, 0, 0}, 100}, {true, {9.0f, 0, 0}, 100}, {false, {}, 0}};
    const VoteResult c = w.update(conflict, true);
    CHECK(!c.valid);
    CHECK(c.status == RedundancyStatus::Ambiguous);
}
