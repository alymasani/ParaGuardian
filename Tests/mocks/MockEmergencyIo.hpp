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
/// Synchronous by default (save() returns Committed). async = true models the real
/// queued store: save() returns Queued and nothing is durable until commitPending().
struct EmergencyStore final : pg::IEmergencyStore {
    bool avail = true;
    bool failWrites = false;
    bool async = false;
    bool has = false;
    bool corrupt = false;      ///< mountState() reports Corrupt (data present, nothing valid)
    uint8_t bytes[pg::kEmergencyRecordBytes] = {};
    bool hasPending = false;
    uint8_t pending[pg::kEmergencyRecordBytes] = {};
    bool hasUncommitted = false;
    uint8_t uncommitted[pg::kEmergencyRecordBytes] = {};
    uint32_t writes = 0;       ///< committed records
    uint32_t saves = 0;        ///< save() calls accepted
    uint32_t ticket = 0, committed = 0, failed = 0;
    bool available() const override { return avail; }
    pg::StoreMount mountState() const override
    {
        if (!avail) return pg::StoreMount::Unavailable;
        if (has) return pg::StoreMount::Recovered;
        return corrupt ? pg::StoreMount::Corrupt : pg::StoreMount::Blank;
    }
    bool load(uint8_t* dst, size_t len) override
    {
        if (!has || len != sizeof(bytes)) return false;
        std::memcpy(dst, bytes, len);
        return true;
    }
    bool loadUncommitted(uint8_t* dst, size_t len) override
    {
        if (!hasUncommitted || len != sizeof(uncommitted)) return false;
        std::memcpy(dst, uncommitted, len);
        return true;
    }
    pg::PersistAck save(const uint8_t* src, size_t len, uint32_t& t) override
    {
        if (!avail || len != sizeof(bytes)) return pg::PersistAck::Rejected;
        t = ++ticket;
        ++saves;
        if (async) {
            std::memcpy(pending, src, len);
            hasPending = true;
            return pg::PersistAck::Queued;
        }
        if (failWrites) {
            failed = t;
            return pg::PersistAck::Rejected;
        }
        std::memcpy(bytes, src, len);
        has = true;
        ++writes;
        committed = t;
        return pg::PersistAck::Committed;
    }
    /// The (simulated) storage task commits the newest pending record.
    bool commitPending()
    {
        if (!hasPending) return false;
        if (failWrites) {
            failed = ticket;
            return false;
        }
        std::memcpy(bytes, pending, sizeof(bytes));
        has = true;
        hasPending = false;
        ++writes;
        committed = ticket;
        return true;
    }
    /// A reset: RAM-only state (the queue) is lost.
    void reset() { hasPending = false; }
    uint32_t committedTicket() const override { return committed; }
    uint32_t failedTicket() const override { return failed; }
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
