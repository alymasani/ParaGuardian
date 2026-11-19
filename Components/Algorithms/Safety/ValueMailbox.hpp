/**
 ******************************************************************************
 * @file    ValueMailbox.hpp
 * @brief   Generic latest-value hand-over (same pattern as AttitudeMailbox /
 *          NavigationMailbox / SafetySnapshotMailbox) for the Milestone 6
 *          status structs (EmergencyStatus, DispatchStatus). One writer task,
 *          any number of readers; never overflows, never blocks for long.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ALGORITHMS_SAFETY_VALUE_MAILBOX_HPP
#define PARAGUARDIAN_ALGORITHMS_SAFETY_VALUE_MAILBOX_HPP

#include <type_traits>

#include "Acquisition/LatestSampleMailbox.hpp"   // ICriticalSection

namespace pg {

template <typename T>
class ValueMailbox {
    static_assert(std::is_trivially_copyable<T>::value, "mailbox values must be memcpy-safe");

public:
    explicit ValueMailbox(ICriticalSection& lock) : lock_(lock) {}
    void publish(const T& v)
    {
        lock_.enter();
        value_ = v;
        present_ = true;
        lock_.exit();
    }
    bool read(T& out) const
    {
        lock_.enter();
        const bool p = present_;
        if (p) out = value_;
        lock_.exit();
        return p;
    }

private:
    ICriticalSection& lock_;
    T value_{};
    bool present_ = false;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ALGORITHMS_SAFETY_VALUE_MAILBOX_HPP
