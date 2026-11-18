/**
 ******************************************************************************
 * @file    RedundancyVoter.cpp
 * @brief   See RedundancyVoter.hpp.
 ******************************************************************************
 */
#include "Processing/RedundancyVoter.hpp"

#include "Processing/TimeBase.hpp"

namespace pg {

namespace {
constexpr uint8_t bit(size_t i) { return static_cast<uint8_t>(1u << i); }
}  // namespace

RedundancyVoter::RedundancyVoter(uint8_t sensorCount, const VoterConfig& cfg)
    : n_((sensorCount > kMaxSensors) ? static_cast<uint8_t>(kMaxSensors) : sensorCount), cfg_(cfg)
{
}

void RedundancyVoter::reset()
{
    for (size_t i = 0; i < kMaxSensors; ++i) {
        track_[i] = Track{};
    }
    selectionChanges_ = 0u;
}

bool RedundancyVoter::agree(const Vec3f& a, const Vec3f& b) const
{
    const float na = vec::norm(a);
    const float nb = vec::norm(b);
    const float tol = cfg_.absTolerance + cfg_.relTolerance * ((na > nb) ? na : nb);
    return vec::distance(a, b) <= tol;
}

VoteResult RedundancyVoter::update(const VoteInput (&in)[kMaxSensors], bool newData)
{
    VoteResult r{};
    r.status = RedundancyStatus::Unavailable;

    uint8_t excludedBefore = 0u;
    for (size_t i = 0; i < n_; ++i) {
        if (track_[i].votedOut || track_[i].requalifying) excludedBefore |= bit(i);
    }

    // ---- 1. candidates and time skew ----
    bool haveNewest = false;
    uint32_t newest = 0u;
    for (size_t i = 0; i < n_; ++i) {
        if (in[i].available && vec::isFinite(in[i].value)) {
            if (!haveNewest || timebase::isAfter(in[i].timestampMs, newest)) {
                newest = in[i].timestampMs;
                haveNewest = true;
            }
        }
    }
    for (size_t i = 0; i < n_; ++i) {
        if (!in[i].available || !vec::isFinite(in[i].value)) {
            continue;
        }
        if (timebase::ageMs(in[i].timestampMs, newest) > cfg_.maxSkewMs) {
            r.skewedMask |= bit(i);
        } else {
            r.candidateMask |= bit(i);
        }
    }

    // ---- absence bookkeeping (re-qualification on return) ----
    if (newData) {
        for (size_t i = 0; i < n_; ++i) {
            Track& t = track_[i];
            if ((r.candidateMask & bit(i)) == 0u) {
                bump(t.absentStreak);
                if (t.everSeen && !t.votedOut && t.absentStreak >= cfg_.requalifyAfterAbsent) {
                    t.requalifying = true;
                    t.agreeStreak = 0u;
                }
            } else {
                t.absentStreak = 0u;
                t.everSeen = true;   // first appearance: included immediately (start-up)
            }
        }
    }

    // ---- 2. pairwise agreement ----
    bool ag[kMaxSensors][kMaxSensors] = {};
    for (size_t i = 0; i < n_; ++i) {
        for (size_t j = i + 1u; j < n_; ++j) {
            if ((r.candidateMask & bit(i)) && (r.candidateMask & bit(j))) {
                const float d = vec::distance(in[i].value, in[j].value);
                r.maxPairDifference = (d > r.maxPairDifference) ? d : r.maxPairDifference;
                ag[i][j] = ag[j][i] = agree(in[i].value, in[j].value);
            }
        }
    }

    // ---- 3. voters and this update's outlier ----
    uint8_t voters = 0u;
    size_t voterCount = 0u;
    for (size_t i = 0; i < n_; ++i) {
        if ((r.candidateMask & bit(i)) && !track_[i].votedOut && !track_[i].requalifying) {
            voters |= bit(i);
            ++voterCount;
        }
    }
    if (voterCount == 3u) {
        for (size_t i = 0; i < 3u; ++i) {
            const size_t j = (i + 1u) % 3u;
            const size_t k = (i + 2u) % 3u;
            if (!ag[i][j] && !ag[i][k] && ag[j][k]) {
                r.outlierMask |= bit(i);
            }
        }
    }

    // ---- 4. contributors and output ----
    size_t idx[kMaxSensors] = {};
    size_t m = 0u;
    for (size_t i = 0; i < n_; ++i) {
        if ((voters & bit(i)) && !(r.outlierMask & bit(i))) {
            idx[m++] = i;
        }
    }

    if (r.candidateMask == 0u) {
        r.status = RedundancyStatus::Unavailable;
    } else if (m == 0u) {
        // Only excluded / re-qualifying sensors are available: nothing trustworthy.
        r.status = RedundancyStatus::Ambiguous;
    } else if (m == 1u) {
        r.valid = true;
        r.value = in[idx[0]].value;
        r.status = RedundancyStatus::Degraded;
        // A re-qualifying (not voted-out) sensor that disagrees with the single
        // contributor is an unresolved 1-vs-1 conflict: do not pick a side.
        for (size_t i = 0; i < n_; ++i) {
            if ((r.candidateMask & bit(i)) && track_[i].requalifying && !track_[i].votedOut &&
                !agree(in[i].value, in[idx[0]].value)) {
                r.valid = false;
                r.status = RedundancyStatus::Ambiguous;
            }
        }
    } else if (m == 2u) {
        if (ag[idx[0]][idx[1]]) {
            r.valid = true;
            r.value = vec::scale(vec::add(in[idx[0]].value, in[idx[1]].value), 0.5f);
            r.status = (n_ == 2u && voterCount == 2u) ? RedundancyStatus::Healthy : RedundancyStatus::Degraded;
        } else {
            r.status = RedundancyStatus::Ambiguous;
        }
    } else {  // m == 3
        const int edges = (ag[0][1] ? 1 : 0) + (ag[0][2] ? 1 : 0) + (ag[1][2] ? 1 : 0);
        if (edges == 3) {
            r.valid = true;
            r.value = vec::scale(vec::add(vec::add(in[0].value, in[1].value), in[2].value), 1.0f / 3.0f);
            r.status = RedundancyStatus::Healthy;
        } else if (edges == 2) {
            r.valid = true;
            r.value = vec::median3(in[0].value, in[1].value, in[2].value);
            r.status = RedundancyStatus::Degraded;
            r.partialAgreement = true;
        } else {
            r.status = RedundancyStatus::Ambiguous;  // (edges == 1 would have produced an outlier)
        }
    }
    if (r.valid) {
        for (size_t k = 0; k < m; ++k) {
            r.contributingMask |= bit(idx[k]);
        }
        bool first = true;
        for (size_t k = 0; k < m; ++k) {
            const uint32_t t = in[idx[k]].timestampMs;
            if (first) {
                r.timestampMs = r.oldestTimestampMs = t;
                first = false;
            } else {
                if (timebase::isAfter(t, r.timestampMs)) r.timestampMs = t;
                if (timebase::isAfter(r.oldestTimestampMs, t)) r.oldestTimestampMs = t;
            }
        }
    }

    // ---- 5. persistence / hysteresis ----
    if (newData) {
        for (size_t i = 0; i < n_; ++i) {
            if ((r.candidateMask & bit(i)) == 0u) {
                continue;
            }
            Track& t = track_[i];
            if (voters & bit(i)) {
                if (r.outlierMask & bit(i)) {
                    t.agreeStreak = 0u;
                    bump(t.disagreeStreak);
                    if (t.disagreeStreak >= cfg_.excludeAfter) {
                        t.votedOut = true;
                        t.disagreeStreak = 0u;
                        if (t.exclusions != UINT32_MAX) ++t.exclusions;
                    }
                } else if (r.valid) {
                    t.disagreeStreak = 0u;
                    bump(t.agreeStreak);
                }
                // Ambiguous: no evidence either way, streaks unchanged.
            } else if (r.valid) {
                // Excluded or re-qualifying: compare with the trusted output.
                if (agree(in[i].value, r.value)) {
                    bump(t.agreeStreak);
                    if (t.agreeStreak >= cfg_.includeAfter) {
                        t.votedOut = false;
                        t.requalifying = false;
                        t.agreeStreak = 0u;
                        t.disagreeStreak = 0u;
                        if (t.readmissions != UINT32_MAX) ++t.readmissions;
                    }
                } else {
                    t.agreeStreak = 0u;
                }
            }
        }
    }

    for (size_t i = 0; i < n_; ++i) {
        if (track_[i].votedOut || track_[i].requalifying) r.excludedMask |= bit(i);
    }
    if (r.excludedMask != excludedBefore && selectionChanges_ != UINT32_MAX) {
        ++selectionChanges_;
    }
    return r;
}

}  // namespace pg
