/**
 ******************************************************************************
 * @file    RtosSampleQueue.hpp
 * @brief   ISampleQueue backed by a STATICALLY allocated FreeRTOS queue.
 *
 * xQueueCreateStatic() uses memory we provide (storage_ + qcb_) instead of
 * pvPortMalloc(), so the 15 KB FreeRTOS heap is not touched and creation
 * cannot fail for lack of memory.
 *
 * Call create() once, after osKernelInitialize(), before use.
 * tryPush() uses a 0-tick timeout: it never blocks (see ISampleQueue).
 * It is for TASK context only; an ISR would need xQueueSendFromISR().
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TASKS_RTOS_SAMPLE_QUEUE_HPP
#define PARAGUARDIAN_TASKS_RTOS_SAMPLE_QUEUE_HPP

#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "queue.h"

#include "Acquisition/SampleQueue.hpp"

namespace pg {

template <size_t N>
class RtosSampleQueue final : public ISampleQueue {
public:
    bool create()
    {
        if (handle_ == nullptr) {
            handle_ = xQueueCreateStatic(static_cast<UBaseType_t>(N), sizeof(SensorMessage), storage_, &control_);
        }
        return handle_ != nullptr;
    }

    bool tryPush(const SensorMessage& msg) override
    {
        return handle_ != nullptr && xQueueSend(handle_, &msg, 0) == pdPASS;
    }

    bool tryPop(SensorMessage& msg) override
    {
        return handle_ != nullptr && xQueueReceive(handle_, &msg, 0) == pdPASS;
    }

    /**
     * Drop-oldest support: removes the oldest item with a 0-tick receive.
     * Safe with one producer (the caller) and one consumer: each FreeRTOS
     * queue operation is atomic, so producer and consumer simply receive
     * different items; nothing else adds items, so the following tryPush()
     * finds the freed slot.
     */
    bool discardOldest() override
    {
        SensorMessage dropped;
        return handle_ != nullptr && xQueueReceive(handle_, &dropped, 0) == pdPASS;
    }

    /// Blocks the calling task until a message arrives or @p ticks elapse.
    bool popWait(SensorMessage& msg, TickType_t ticks)
    {
        return handle_ != nullptr && xQueueReceive(handle_, &msg, ticks) == pdPASS;
    }

    size_t size() const override
    {
        return (handle_ != nullptr) ? static_cast<size_t>(uxQueueMessagesWaiting(handle_)) : 0u;
    }
    size_t capacity() const override { return N; }

private:
    uint8_t storage_[N * sizeof(SensorMessage)] = {};
    StaticQueue_t control_{};
    QueueHandle_t handle_ = nullptr;
};

}  // namespace pg

#endif  // PARAGUARDIAN_TASKS_RTOS_SAMPLE_QUEUE_HPP
