/**
 ******************************************************************************
 * @file    SampleQueue.hpp
 * @brief   Abstract bounded queue of SensorMessage + a portable implementation.
 *
 * The SensorDataHub publishes into ISampleQueue objects. On the STM32 each
 * consumer task owns an RtosSampleQueue (Components/Tasks), backed by a
 * statically allocated FreeRTOS queue so it can block waiting for data.
 * On the laptop the tests use BoundedSampleQueue below.
 *
 * Contract for tryPush(): NEVER blocks. If the queue is full it returns false
 * and the caller counts a drop. A sampling loop must not stall because a
 * slow consumer stopped reading.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_ACQUISITION_SAMPLE_QUEUE_HPP
#define PARAGUARDIAN_ACQUISITION_SAMPLE_QUEUE_HPP

#include <cstddef>

#include "Acquisition/SensorMessage.hpp"

namespace pg {

class ISampleQueue {
public:
    virtual bool tryPush(const SensorMessage& msg) = 0;
    virtual bool tryPop(SensorMessage& msg) = 0;
    /**
     * Removes the OLDEST element without returning it (used by the
     * drop-oldest overflow policy). Called only by the single producer; the
     * consumer may pop concurrently. Returns false if the queue was empty.
     */
    virtual bool discardOldest() = 0;
    virtual size_t size() const = 0;
    virtual size_t capacity() const = 0;

protected:
    ~ISampleQueue() = default;
};

/// Simple fixed-capacity FIFO, NOT thread-safe. For host tests and single-task use.
template <size_t N>
class BoundedSampleQueue final : public ISampleQueue {
    static_assert(N > 0u, "capacity must be > 0");

public:
    bool tryPush(const SensorMessage& msg) override
    {
        if (count_ == N) {
            return false;
        }
        items_[(head_ + count_) % N] = msg;
        ++count_;
        return true;
    }
    bool tryPop(SensorMessage& msg) override
    {
        if (count_ == 0u) {
            return false;
        }
        msg = items_[head_];
        head_ = (head_ + 1u) % N;
        --count_;
        return true;
    }
    bool discardOldest() override
    {
        SensorMessage dropped{};
        return tryPop(dropped);
    }
    size_t size() const override { return count_; }
    size_t capacity() const override { return N; }

private:
    SensorMessage items_[N] = {};
    size_t head_ = 0u;
    size_t count_ = 0u;
};

}  // namespace pg

#endif  // PARAGUARDIAN_ACQUISITION_SAMPLE_QUEUE_HPP
