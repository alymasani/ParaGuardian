/**
 ******************************************************************************
 * @file    ByteRingBuffer.hpp
 * @brief   Fixed-size single-producer / single-consumer byte FIFO.
 *
 * Used between a UART receive interrupt (producer: push one byte at a time)
 * and a task (consumer: pop many bytes at once). No locks are needed because:
 *   - only the producer ever writes head_,
 *   - only the consumer ever writes tail_,
 *   - both indices are std::atomic<uint32_t>, which is lock-free on the
 *     Cortex-M7 (a single aligned 32-bit load/store), and acquire/release
 *     ordering makes sure the byte is stored before the index that publishes it.
 *
 * Capacity N must be a power of two so "index % N" becomes "index & (N-1)".
 * The indices run freely and wrap at 2^32; (head - tail) is always the fill level.
 * One slot is NOT wasted: the buffer really holds N bytes.
 *
 * Overflow policy: when full, the NEW byte is dropped and overflowCount() is
 * incremented. The parser downstream then sees a gap and resynchronises.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_COMMON_BYTE_RING_BUFFER_HPP
#define PARAGUARDIAN_COMMON_BYTE_RING_BUFFER_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace pg {

template <uint32_t N>
class ByteRingBuffer {
    static_assert(N >= 2u && (N & (N - 1u)) == 0u, "ByteRingBuffer capacity must be a power of two");

public:
    /// Producer side (ISR-safe). Returns false (and counts) if the buffer was full.
    bool push(uint8_t byte)
    {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        const uint32_t tail = tail_.load(std::memory_order_acquire);
        if ((head - tail) >= N) {
            overflowCount_.fetch_add(1u, std::memory_order_relaxed);
            return false;
        }
        storage_[head & (N - 1u)] = byte;
        head_.store(head + 1u, std::memory_order_release);  // publish the byte
        return true;
    }

    /// Consumer side. Copies up to maxLen bytes; returns number copied.
    size_t pop(uint8_t* dst, size_t maxLen)
    {
        if (dst == nullptr) {
            return 0u;
        }
        uint32_t tail = tail_.load(std::memory_order_relaxed);
        const uint32_t head = head_.load(std::memory_order_acquire);
        size_t count = 0u;
        while (tail != head && count < maxLen) {
            dst[count++] = storage_[tail & (N - 1u)];
            ++tail;
        }
        tail_.store(tail, std::memory_order_release);  // free the slots
        return count;
    }

    size_t size() const
    {
        return static_cast<size_t>(head_.load(std::memory_order_acquire) -
                                   tail_.load(std::memory_order_acquire));
    }
    static constexpr size_t capacity() { return N; }
    uint32_t overflowCount() const { return overflowCount_.load(std::memory_order_relaxed); }

    /// Only call when the producer is stopped (e.g. before enabling the UART interrupt).
    void reset()
    {
        head_.store(0u);
        tail_.store(0u);
        overflowCount_.store(0u);
    }

private:
    uint8_t storage_[N] = {};
    std::atomic<uint32_t> head_{0u};
    std::atomic<uint32_t> tail_{0u};
    std::atomic<uint32_t> overflowCount_{0u};
};

}  // namespace pg

#endif  // PARAGUARDIAN_COMMON_BYTE_RING_BUFFER_HPP
