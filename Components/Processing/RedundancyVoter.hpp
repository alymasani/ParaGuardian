/**
 ******************************************************************************
 * @file    RedundancyVoter.hpp
 * @brief   Consistency check and selection for 2 or 3 redundant sensors
 *          measuring the same vector (IMU specific force, IMU angular rate)
 *          or scalar (barometric altitude, passed as {h, 0, 0}).
 *
 * ONE UPDATE, STEP BY STEP
 *   1. Candidates  = sensors with a fresh, valid, calibrated, body-frame value.
 *      Candidates whose timestamp is more than maxSkewMs older than the newest
 *      candidate are not compared (they describe a different instant).
 *   2. Pairwise agreement: i and j agree when
 *          |v_i - v_j| <= absTolerance + relTolerance * max(|v_i|, |v_j|)
 *   3. Voters = candidates not EXCLUDED. With 3 voters, a sensor that
 *      disagrees with both others WHILE those two agree with each other is
 *      this update's OUTLIER (majority evidence). With 2 voters nobody can be
 *      blamed: a disagreement is AMBIGUOUS.
 *   4. Contributors = voters minus outliers. Output:
 *        3 contributors, all agree        -> mean,   Healthy
 *        3 contributors, 2 of 3 pairs agree -> component median, Degraded (partial)
 *        3 contributors, no pair agrees   -> none,   Ambiguous
 *        2 contributors agree             -> mean,   Healthy if that is every configured sensor, else Degraded
 *        2 contributors disagree          -> none,   Ambiguous
 *        1 contributor                    -> it,     Degraded (no cross-check), unless a
 *                                            re-qualifying sensor disagrees with it -> Ambiguous
 *        0 candidates                     -> none,   Unavailable
 *   5. Persistence / hysteresis (only when newData, so one sample is never
 *      counted twice): a voter that is the outlier excludeAfter times in a row
 *      is EXCLUDED; an excluded sensor that agrees with the output includeAfter
 *      times in a row is re-included. A sensor absent for requalifyAfterAbsent
 *      updates must re-qualify the same way when it returns (so a recovered
 *      sensor cannot cause a step in the output). Exclusion is what "selection"
 *      means; a one-sample outlier is left out of THAT sample's mean but does
 *      not change the selection.
 *
 * LIMITS (stated, not hidden): majority voting assumes faults are independent.
 * Two sensors that fail the same way (common-mode: shared supply, shared SPI
 * bus fault, same temperature error) out-vote the healthy one. Voting detects
 * inconsistency; it cannot prove correctness.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_PROCESSING_REDUNDANCY_VOTER_HPP
#define PARAGUARDIAN_PROCESSING_REDUNDANCY_VOTER_HPP

#include <cstddef>
#include <cstdint>

#include "Processing/ProcessingConfig.hpp"
#include "Processing/ProcessingTypes.hpp"
#include "Processing/VectorMath.hpp"

namespace pg {

struct VoteInput {
    bool available;        ///< fresh, valid, calibrated, aligned
    Vec3f value;
    uint32_t timestampMs;
};

struct VoteResult {
    RedundancyStatus status;
    bool valid;                 ///< value may be used
    Vec3f value;
    uint32_t timestampMs;       ///< newest contributing sample (MCU ms)
    uint32_t oldestTimestampMs; ///< oldest contributing sample (spread = newest - oldest)
    uint8_t candidateMask;      ///< bit i: sensor i was available and inside the skew window
    uint8_t contributingMask;   ///< bit i: sensor i is in the output
    uint8_t excludedMask;       ///< bit i: sensor i is excluded / re-qualifying (persistent)
    uint8_t outlierMask;        ///< bit i: sensor i was this update's outlier
    uint8_t skewedMask;         ///< bit i: available but too old relative to the others
    bool partialAgreement;      ///< 3 contributors, but not every pair agreed (median used)
    float maxPairDifference;    ///< largest |v_i - v_j| among candidates
};

class RedundancyVoter {
public:
    static constexpr size_t kMaxSensors = 3u;

    RedundancyVoter(uint8_t sensorCount, const VoterConfig& cfg);

    VoteResult update(const VoteInput (&in)[kMaxSensors], bool newData);
    void reset();

    uint8_t sensorCount() const { return n_; }
    bool excluded(size_t i) const { return i < n_ && (track_[i].votedOut || track_[i].requalifying); }
    bool votedOut(size_t i) const { return i < n_ && track_[i].votedOut; }
    uint32_t exclusions(size_t i) const { return i < n_ ? track_[i].exclusions : 0u; }
    uint32_t readmissions(size_t i) const { return i < n_ ? track_[i].readmissions : 0u; }
    uint32_t selectionChanges() const { return selectionChanges_; }
    const VoterConfig& config() const { return cfg_; }

private:
    struct Track {
        bool everSeen;
        bool votedOut;        ///< excluded by majority evidence
        bool requalifying;    ///< returned after an absence; must agree before contributing
        uint16_t disagreeStreak;
        uint16_t agreeStreak;
        uint16_t absentStreak;
        uint32_t exclusions;
        uint32_t readmissions;
    };
    bool agree(const Vec3f& a, const Vec3f& b) const;
    static void bump(uint16_t& v) { if (v != UINT16_MAX) ++v; }

    uint8_t n_;
    VoterConfig cfg_;
    Track track_[kMaxSensors] = {};
    uint32_t selectionChanges_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_PROCESSING_REDUNDANCY_VOTER_HPP
