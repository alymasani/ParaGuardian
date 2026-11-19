/**
 ******************************************************************************
 * @file    SosRequestMailbox.hpp
 * @brief   The RELIABLE record of the current SOS request, separate from the
 *          (lossy-under-overflow) SafetyEvent queue.
 *
 * The EmergencyManager owns the request; the emergency task publishes it here
 * every wake so other tasks (a future BLE task, logging, diagnostics) can
 * always read the CURRENT request and its stage, no matter how many advisory
 * events were dropped. Latest-value semantics; never overflows.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_EMERGENCY_SOS_REQUEST_MAILBOX_HPP
#define PARAGUARDIAN_ALGORITHMS_EMERGENCY_SOS_REQUEST_MAILBOX_HPP

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection
#include "Algorithms/Emergency/EmergencyTypes.hpp"
#include "Algorithms/Emergency/SosRequest.hpp"

namespace pg {

struct PendingSosRecord {
    bool active;              ///< a request (alert or cancel notice) is being handled
    SosRequest request;       ///< current revision
    DeliveryStage stage;
    uint16_t reportedStages;
    EmergencyState state;
};

class SosRequestMailbox {
public:
    explicit SosRequestMailbox(ICriticalSection& lock) : lock_(lock) {}
    void publish(const PendingSosRecord& r)
    {
        lock_.enter();
        value_ = r;
        present_ = true;
        lock_.exit();
    }
    bool read(PendingSosRecord& out) const
    {
        lock_.enter();
        const bool p = present_;
        if (p) out = value_;
        lock_.exit();
        return p;
    }

private:
    ICriticalSection& lock_;
    PendingSosRecord value_{};
    bool present_ = false;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_EMERGENCY_SOS_REQUEST_MAILBOX_HPP
