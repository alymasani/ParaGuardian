/**
 ******************************************************************************
 * @file    MockClock.hpp
 * @brief   Fake IClock: time only moves when the code under test waits.
 *
 * delayMs(n) advances time by n instantly, so a "50 ms reset timeout" test
 * takes microseconds. totalDelayMs records how long the driver chose to wait.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_MOCK_CLOCK_HPP
#define PARAGUARDIAN_TESTS_MOCK_CLOCK_HPP

#include <cstdint>

#include "Common/BusInterfaces.hpp"

class MockClock final : public pg::IClock {
public:
    uint32_t millis() override { return now; }
    void delayMs(uint32_t ms) override
    {
        now += ms;
        totalDelayMs += ms;
        ++delayCalls;
    }

    uint32_t now = 0;
    uint32_t totalDelayMs = 0;
    uint32_t delayCalls = 0;
};

#endif  // PARAGUARDIAN_TESTS_MOCK_CLOCK_HPP
