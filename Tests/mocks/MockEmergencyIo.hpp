/**
 ******************************************************************************
 * @file    MockEmergencyIo.hpp
 * @brief   Host-test doubles for the Milestone 6 output / transport /
 *          persistence interfaces. They record what the firmware asked for;
 *          they do NOT model any real BLE module, phone, speaker or LED.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_TESTS_MOCKS_MOCK_EMERGENCY_IO_HPP
#define PARAGUARDIAN_TESTS_MOCKS_MOCK_EMERGENCY_IO_HPP

#include <cstdint>
#include <cstring>
#include <vector>

#include "Acquisition/LatestSampleMailbox.hpp"
#include "Algorithms/Emergency/IEmergencyStore.hpp"
#include "Algorithms/Emergency/ISosTransport.hpp"
#include "Output/Audio/IAudioAlert.hpp"
#include "Output/LED/IStatusLed.hpp"

namespace mock {

struct NoLock final : pg::ICriticalSection {
    void enter() override {}
    void exit() override {}
};

/// Transport: records submitted requests; the test decides link state and results.
struct SosTransport final : pg::ISosTransport {
    bool link = true;
    pg::TransportSubmitResult result = pg::TransportSubmitResult::Accepted;
    struct Sent {
        pg::SosRequest req;
        uint32_t ms;
    };
    std::vector<Sent> sent;
    bool linkUp() const override { return link; }
    pg::TransportSubmitResult submit(const pg::SosRequest& r, uint32_t nowMs) override
    {
        if (!link) return pg::TransportSubmitResult::LinkDown;
        if (result == pg::TransportSubmitResult::Accepted) sent.push_back(Sent{r, nowMs});
        return result;
    }
};

/// RAM "persistence" (survives a simulated reset because the test keeps the object).
struct EmergencyStore final : pg::IEmergencyStore {
    bool avail = true;
    bool failWrites = false;
    bool has = false;
    uint8_t bytes[pg::kEmergencyRecordBytes] = {};
    uint32_t writes = 0;
    bool available() const override { return avail; }
    bool load(uint8_t* dst, size_t len) override
    {
        if (!has || len != sizeof(bytes)) return false;
        std::memcpy(dst, bytes, len);
        return true;
    }
    bool save(const uint8_t* src, size_t len) override
    {
        if (failWrites || len != sizeof(bytes)) return false;
        std::memcpy(bytes, src, len);
        has = true;
        ++writes;
        return true;
    }
};

struct AudioAlert final : pg::IAudioAlert {
    bool fail = false;
    struct Req {
        pg::AlertRequest r;
        uint32_t index;
    };
    std::vector<pg::AlertRequest> requests;
    std::vector<pg::AlertTone> cancels;
    pg::Status request(const pg::AlertRequest& r) override
    {
        if (fail) return pg::Status::NotInitialized;
        requests.push_back(r);
        return pg::Status::Ok;
    }
    void cancel(pg::AlertTone t) override { cancels.push_back(t); }
    bool isPlaying() const override { return !requests.empty(); }
    size_t count(pg::AlertTone t) const
    {
        size_t n = 0;
        for (const auto& r : requests) n += (r.tone == t) ? 1u : 0u;
        return n;
    }
};

struct StatusLed final : pg::IStatusLed {
    bool avail = true;
    pg::LedPattern pattern = pg::LedPattern::Off;
    pg::LedColor color = pg::LedColor::Default;
    uint32_t sets = 0, ticks = 0;
    void setPattern(pg::LedPattern p, pg::LedColor c) override
    {
        pattern = p;
        color = c;
        ++sets;
    }
    void tick(uint32_t) override { ++ticks; }
    bool available() const override { return avail; }
};

}  // namespace mock

#endif  // PARAGUARDIAN_TESTS_MOCKS_MOCK_EMERGENCY_IO_HPP
