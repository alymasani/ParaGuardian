/**
 * W25Q128JV tests against the FakeW25q behavioural model.
 * Geometry: 16 MiB, 256-byte pages, 4 KiB sectors. JEDEC EF 40 18 (or EF 70 18).
 */
#include <vector>

#include "TestFramework.hpp"
#include "mocks/FakeW25q.hpp"
#include "mocks/MockClock.hpp"

#include "Storage/W25Q128/W25q128.hpp"

using namespace pg;

namespace {
struct FlashFixture {
    FakeW25q chip;
    MockClock clock;
    W25q128 flash{chip, clock};
};
}  // namespace

TEST(w25q_init_reads_jedec_and_never_erases)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    CHECK(f.flash.isInitialized());
    CHECK(f.chip.sawCommand(0xAB));
    CHECK(f.chip.sawCommand(0x9F));
    CHECK(!f.chip.sawCommand(0x20));
    CHECK(!f.chip.sawCommand(0xD8));
    CHECK(!f.chip.sawCommand(0xC7));
    CHECK(!f.chip.sawCommand(0x60));
}

TEST(w25q_dtr_variant_accepted_wrong_part_rejected)
{
    FlashFixture a;
    a.chip.jedec[1] = 0x70;
    CHECK(isOk(a.flash.init()));
    FlashFixture b;
    b.chip.jedec[2] = 0x17;  // W25Q64 capacity code
    CHECK(b.flash.init() == Status::WrongDeviceId);
    FlashFixture c;
    c.chip.jedec[0] = 0xC2;  // Macronix
    CHECK(c.flash.init() == Status::WrongDeviceId);
    uint8_t buf[4];
    CHECK(c.flash.read(0, buf, 4) == Status::NotInitialized);
}

TEST(w25q_bounds_checking)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    uint8_t b[2] = {};
    CHECK(isOk(f.flash.read(0xFFFFFF, b, 1)));                   // last byte
    CHECK(f.flash.read(0xFFFFFF, b, 2) == Status::OutOfRange);    // one past the end
    CHECK(f.flash.read(0x1000000, b, 1) == Status::OutOfRange);
    CHECK(f.flash.read(0, nullptr, 1) == Status::InvalidArgument);
    CHECK(f.flash.read(0, b, 0) == Status::InvalidArgument);
    CHECK(!W25q128::rangeIsValid(0xFFFFFFFFu, 2));               // no integer overflow
    CHECK(W25q128::rangeIsValid(0, 16u * 1024u * 1024u));
}

TEST(w25q_page_program_rejects_page_crossing)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    uint8_t data[32] = {};
    CHECK(f.flash.pageProgram(0xF0, data, 32) == Status::InvalidArgument);  // 0xF0 + 32 > 0x100
    CHECK(f.chip.programs.empty());
    CHECK(isOk(f.flash.pageProgram(0xE0, data, 32)));                      // exactly to the end
    CHECK(f.flash.pageProgram(0, data, 257) == Status::InvalidArgument);
}

TEST(w25q_write_splits_at_page_boundary_and_data_correct)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    std::vector<uint8_t> data(32);
    for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i + 1);
    CHECK(isOk(f.flash.write(0xF0, data.data(), data.size(), true)));
    CHECK_EQ(f.chip.programs.size(), 2u);
    CHECK_EQ(f.chip.programs[0].addr, 0xF0u);
    CHECK_EQ(f.chip.programs[0].len, 16u);
    CHECK_EQ(f.chip.programs[1].addr, 0x100u);
    CHECK_EQ(f.chip.programs[1].len, 16u);
    for (size_t i = 0; i < data.size(); ++i) CHECK_EQ(f.chip.mem[0xF0 + i], data[i]);
    CHECK_EQ(f.chip.mem[0x00], 0xFF);  // nothing wrapped to the page start
    uint8_t back[32] = {};
    CHECK(isOk(f.flash.read(0xF0, back, 32)));
    for (size_t i = 0; i < 32; ++i) CHECK_EQ(back[i], data[i]);
}

TEST(w25q_large_write_spanning_many_pages)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    std::vector<uint8_t> data(1000);
    for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<uint8_t>(i * 7);
    CHECK(isOk(f.flash.write(0x1234, data.data(), data.size())));
    // 0x1234 -> page offset 0x34: first chunk 0xCC (204) bytes, then 3x256, then 28
    CHECK_EQ(f.chip.programs.size(), 5u);
    CHECK_EQ(f.chip.programs[0].len, 204u);
    CHECK_EQ(f.chip.programs[4].len, 28u);
    for (size_t i = 0; i < data.size(); ++i) CHECK_EQ(f.chip.mem[0x1234 + i], data[i]);
    CHECK_EQ(f.flash.stats().pagesProgrammed, 5u);
}

TEST(w25q_write_protected_chip_reports_verify_failed)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    f.chip.writeProtected = true;  // WEL never sets
    uint8_t d[4] = {1, 2, 3, 4};
    CHECK(f.flash.pageProgram(0, d, 4) == Status::VerifyFailed);
    CHECK(f.flash.eraseSector(0) == Status::VerifyFailed);
}

TEST(w25q_verify_detects_unerased_area)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    f.chip.mem[0x10] = 0x00;  // previously programmed byte, not erased
    uint8_t d[4] = {0xAA, 0xAA, 0xAA, 0xAA};
    CHECK(f.flash.write(0x10, d, 4, true) == Status::VerifyFailed);  // NOR can't set 0 -> 1
    CHECK_EQ(f.flash.stats().verifyFailures, 1u);
}

TEST(w25q_sector_erase_alignment_and_effect)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    CHECK(f.flash.eraseSector(0x1001) == Status::InvalidArgument);
    CHECK(f.chip.erases.empty());
    for (uint32_t i = 0; i < 3u * 4096u; ++i) f.chip.mem[i] = 0x00;  // sectors 0, 1 and 2 programmed
    CHECK(isOk(f.flash.eraseSector(0x1000)));
    CHECK_EQ(f.chip.mem[0x0FFF], 0x00);  // previous sector untouched
    CHECK_EQ(f.chip.mem[0x1000], 0xFF);
    CHECK_EQ(f.chip.mem[0x1FFF], 0xFF);
    CHECK_EQ(f.chip.mem[0x2000], 0x00);  // next sector untouched
    CHECK(f.flash.eraseSector(0x1000000) == Status::OutOfRange);
}

TEST(w25q_stuck_busy_times_out)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    f.chip.stuckBusy = true;
    uint8_t d[1] = {0};
    const uint32_t start = f.clock.now;
    CHECK(f.flash.pageProgram(0, d, 1) == Status::Timeout);
    CHECK(f.clock.now - start <= w25q::PAGE_PROGRAM_TIMEOUT_MS + 1u);
    CHECK_EQ(f.flash.stats().busyTimeouts, 1u);
}

TEST(w25q_bus_error_propagates)
{
    FlashFixture f;
    CHECK(isOk(f.flash.init()));
    f.chip.failNext = true;
    uint8_t b[4];
    CHECK(f.flash.read(0, b, 4) == Status::BusError);
    CHECK_EQ(f.flash.stats().busErrors, 1u);
}

TEST(w25q_read_command_bytes_big_endian_address)
{
    // Check the exact bytes sent for READ DATA at 0x123456: 03 12 34 56.
    struct Recorder final : public ISpiDevice {
        uint8_t last[4] = {};
        uint8_t jedec[3] = {0xEF, 0x40, 0x18};
        Status writeThenRead(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override {
            if (tx[0] == 0x9F) { for (size_t i = 0; i < rxLen; ++i) rx[i] = jedec[i]; return Status::Ok; }
            if (tx[0] == 0x05) { rx[0] = 0; return Status::Ok; }
            for (size_t i = 0; i < txLen && i < 4; ++i) last[i] = tx[i];
            for (size_t i = 0; i < rxLen; ++i) rx[i] = 0;
            return Status::Ok;
        }
        Status writeThenWrite(const uint8_t*, size_t, const uint8_t*, size_t) override { return Status::Ok; }
    } rec;
    MockClock clock;
    W25q128 flash(rec, clock);
    CHECK(isOk(flash.init()));
    uint8_t b[1];
    CHECK(isOk(flash.read(0x123456, b, 1)));
    CHECK_EQ(rec.last[0], 0x03);
    CHECK_EQ(rec.last[1], 0x12);
    CHECK_EQ(rec.last[2], 0x34);
    CHECK_EQ(rec.last[3], 0x56);
}
