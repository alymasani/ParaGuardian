/**
 ******************************************************************************
 * @file    LatestSampleMailbox.hpp
 * @brief   "Latest value per sensor" store for consumers that must act on the
 *          FRESHEST data (safety monitoring), never on a backlog.
 *
 * One slot per SensorId. store() overwrites that sensor's slot; read() copies
 * it out. Nothing queues up, so a slow reader can never see old data piling
 * up - but it can see an OLD value if the sensor stopped producing, which is
 * why every read should check the sample age (SampleFreshness.hpp).
 *
 * Thread safety: the producer (acquisition task) and any number of readers
 * may use it concurrently. Each copy in/out happens inside an injected
 * critical section (ICriticalSection): on the STM32 that is
 * taskENTER_CRITICAL / taskEXIT_CRITICAL (Tasks/RtosCriticalSection.hpp), on
 * the laptop a std::mutex (tests). A copy is ~100 bytes, so the lock is held
 * for well under a microsecond at 64 MHz.
 *
 * Memory: kSlots x sizeof(SensorMessage), fixed at compile time.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ACQUISITION_LATEST_SAMPLE_MAILBOX_HPP
#define PARAGUARDIAN_ACQUISITION_LATEST_SAMPLE_MAILBOX_HPP

#include <cstddef>
#include <cstdint>

#include "Acquisition/SensorMessage.hpp"

namespace pg {

/// Minimal lock interface so the same mailbox works on FreeRTOS and in host tests.
class ICriticalSection {
public:
    virtual void enter() = 0;
    virtual void exit() = 0;

protected:
    ~ICriticalSection() = default;
};

/// For single-threaded use (unit tests that do not need locking).
class NoCriticalSection final : public ICriticalSection {
public:
    void enter() override {}
    void exit() override {}
};

class LatestSampleMailbox {
public:
    static constexpr size_t kSlots = static_cast<size_t>(SensorId::Count);

    explicit LatestSampleMailbox(ICriticalSection& lock) : lock_(lock) {}

    /// Stores @p msg in its sensor's slot (ignored if it carries no sample or an invalid id).
    void store(const SensorMessage& msg)
    {
        const SampleHeader* h = headerOf(msg);
        if (h == nullptr || static_cast<size_t>(h->id) >= kSlots) {
            return;
        }
        Slot& slot = slots_[static_cast<size_t>(h->id)];
        lock_.enter();
        slot.msg = msg;
        slot.present = true;
        ++slot.writes;
        lock_.exit();
    }

    /// Copies the latest message of sensor @p id. Returns false if none was ever stored.
    bool read(SensorId id, SensorMessage& out) const
    {
        if (static_cast<size_t>(id) >= kSlots) {
            return false;
        }
        const Slot& slot = slots_[static_cast<size_t>(id)];
        lock_.enter();
        const bool present = slot.present;
        if (present) {
            out = slot.msg;
        }
        lock_.exit();
        return present;
    }

    /// How many times sensor @p id's slot was written (diagnostics).
    uint32_t writes(SensorId id) const
    {
        if (static_cast<size_t>(id) >= kSlots) {
            return 0u;
        }
        lock_.enter();
        const uint32_t w = slots_[static_cast<size_t>(id)].writes;
        lock_.exit();
        return w;
    }

private:
    struct Slot {
        SensorMessage msg;
        bool present;
        uint32_t writes;
    };
    ICriticalSection& lock_;
    Slot slots_[kSlots] = {};
};

}  // namespace pg

#endif  // PARAGUARDIAN_ACQUISITION_LATEST_SAMPLE_MAILBOX_HPP
