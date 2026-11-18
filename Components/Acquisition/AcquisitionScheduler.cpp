/**
 ******************************************************************************
 * @file    AcquisitionScheduler.cpp
 * @brief   See AcquisitionScheduler.hpp.
 ******************************************************************************
 */
#include "Acquisition/AcquisitionScheduler.hpp"

namespace pg {

bool AcquisitionScheduler::add(IAcquisitionSource* source)
{
    if (source == nullptr || count_ >= kMaxSources) {
        return false;
    }
    sources_[count_++] = source;
    return true;
}

uint32_t AcquisitionScheduler::runCycle(ISampleSink& sink)
{
    const uint32_t start = clock_.millis();
    CycleBudget budget{};
    budget.initAttemptsRemaining = initAttemptsPerCycle_;

    // Rotate the starting source each cycle so that, when several sources want
    // the single init slot in the same cycle, the list order does not always
    // decide who goes first (fair tie-breaking; backoff already prevents any
    // one failing source from taking every slot).
    for (size_t k = 0; k < count_; ++k) {
        sources_[(rotation_ + k) % count_]->service(start, sink, budget);
    }
    if (count_ > 0u) {
        rotation_ = (rotation_ + 1u) % count_;
    }

    const uint32_t elapsed = clock_.millis() - start;
    ++stats_.cycles;
    stats_.lastCycleTimeMs = elapsed;
    if (elapsed > stats_.maxCycleTimeMs) {
        stats_.maxCycleTimeMs = elapsed;
    }
    if (elapsed >= periodMs_) {
        ++stats_.cycleOverruns;
    }
    stats_.initAttemptsDeferred += budget.initAttemptsDeferred;
    return elapsed;
}

}  // namespace pg
