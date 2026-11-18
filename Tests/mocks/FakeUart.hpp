/**
 ******************************************************************************
 * @file    FakeUart.hpp
 * @brief   Fake UART: a byte source tests can fill, and a writer that records.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_FAKE_UART_HPP
#define PARAGUARDIAN_TESTS_FAKE_UART_HPP

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "Common/BusInterfaces.hpp"

class FakeByteSource final : public pg::IByteSource {
public:
    std::deque<uint8_t> pending;

    void add(const std::vector<uint8_t>& bytes) { pending.insert(pending.end(), bytes.begin(), bytes.end()); }

    size_t read(uint8_t* dst, size_t maxLen) override
    {
        size_t n = 0;
        while (n < maxLen && !pending.empty()) {
            dst[n++] = pending.front();
            pending.pop_front();
        }
        return n;
    }
};

class FakeByteWriter final : public pg::IByteWriter {
public:
    std::string written;
    pg::Status result = pg::Status::Ok;

    pg::Status write(const uint8_t* data, size_t len) override
    {
        if (result == pg::Status::Ok) written.append(reinterpret_cast<const char*>(data), len);
        return result;
    }
};

#endif  // PARAGUARDIAN_TESTS_FAKE_UART_HPP
