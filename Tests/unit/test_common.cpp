/**
 * Tests: byte-order helpers, register-bus command bytes, ring buffer, health counters.
 * Expected values are worked out by hand from two's-complement rules / datasheets,
 * not by calling the implementation.
 */
#include "TestFramework.hpp"
#include "mocks/FakeRegisterDevice.hpp"

#include "Common/ByteOrder.hpp"
#include "Common/ByteRingBuffer.hpp"
#include "Common/RegisterBus.hpp"
#include "Sensors/Common/SensorHealth.hpp"

using namespace pg;

TEST(byteorder_signed16_extremes)
{
    const uint8_t minus32768[2] = {0x00, 0x80};  // 0x8000
    const uint8_t plus32767[2] = {0xFF, 0x7F};   // 0x7FFF
    const uint8_t minus1[2] = {0xFF, 0xFF};
    const uint8_t plus258[2] = {0x02, 0x01};     // 0x0102 = 258
    CHECK_EQ(bytes::le_i16(minus32768), -32768);
    CHECK_EQ(bytes::le_i16(plus32767), 32767);
    CHECK_EQ(bytes::le_i16(minus1), -1);
    CHECK_EQ(bytes::le_i16(plus258), 258);
}

TEST(byteorder_signed24_sign_extension)
{
    const uint8_t minMinus[3] = {0x00, 0x00, 0x80};  // 0x800000 = -8388608
    const uint8_t minus1[3] = {0xFF, 0xFF, 0xFF};
    const uint8_t maxPlus[3] = {0xFF, 0xFF, 0x7F};   // 0x7FFFFF = 8388607
    CHECK_EQ(bytes::le_i24(minMinus), -8388608);
    CHECK_EQ(bytes::le_i24(minus1), -1);
    CHECK_EQ(bytes::le_i24(maxPlus), 8388607);
    CHECK_EQ(bytes::le_u24(minus1), 16777215u);
}

TEST(byteorder_32_and_writers)
{
    const uint8_t v[4] = {0x78, 0x56, 0x34, 0x12};
    CHECK_EQ(bytes::le_u32(v), 0x12345678u);
    const uint8_t neg[4] = {0xFE, 0xFF, 0xFF, 0xFF};
    CHECK_EQ(bytes::le_i32(neg), -2);
    uint8_t out[3] = {};
    bytes::put_be_u24(out, 0x123456u);
    CHECK_EQ(out[0], 0x12);
    CHECK_EQ(out[1], 0x34);
    CHECK_EQ(out[2], 0x56);
}

TEST(spi_register_bus_sets_read_bit_and_write_frame)
{
    FakeRegisterDevice dev;
    dev.regs[0x0F] = 0x6B;
    FakeSpiRegisterDevice spi(dev);
    SpiRegisterBus bus(spi);
    uint8_t v = 0;
    CHECK(isOk(bus.readRegister(0x0F, v)));  // fake rejects reads without bit 7
    CHECK_EQ(v, 0x6B);
    CHECK(isOk(bus.writeRegister(0x10, 0xAB)));
    CHECK_EQ(dev.protocolErrors, 0);
    CHECK_EQ(dev.lastWrite(0x10), 0xAB);
}

TEST(register_bus_rejects_bad_arguments)
{
    FakeRegisterDevice dev;
    FakeSpiRegisterDevice spi(dev);
    SpiRegisterBus bus(spi);
    CHECK(bus.readRegisters(0x00, nullptr, 1) == Status::InvalidArgument);
    uint8_t b = 0;
    CHECK(bus.readRegisters(0x00, &b, 0) == Status::InvalidArgument);
    CHECK_EQ(dev.transactions, 0);  // nothing reached the bus
}

TEST(i2c_register_bus_wrong_address_is_bus_error)
{
    FakeRegisterDevice dev;
    FakeI2cBus i2c(dev, 0x6A);
    I2cRegisterBus right(i2c, 0x6A);
    I2cRegisterBus wrong(i2c, 0x6B);
    uint8_t v = 0;
    CHECK(isOk(right.readRegister(0x0F, v)));
    CHECK(wrong.readRegister(0x0F, v) == Status::BusError);  // NACK
}

TEST(ring_buffer_fifo_order_and_wrap)
{
    ByteRingBuffer<8> rb;
    uint8_t out[8] = {};
    for (int round = 0; round < 5; ++round) {  // indices wrap several times
        for (uint8_t i = 0; i < 6; ++i) CHECK(rb.push(static_cast<uint8_t>(round * 10 + i)));
        CHECK_EQ(rb.size(), 6u);
        CHECK_EQ(rb.pop(out, sizeof(out)), 6u);
        for (uint8_t i = 0; i < 6; ++i) CHECK_EQ(out[i], round * 10 + i);
    }
    CHECK_EQ(rb.overflowCount(), 0u);
}

TEST(ring_buffer_overflow_drops_newest_and_counts)
{
    ByteRingBuffer<4> rb;
    for (uint8_t i = 1; i <= 6; ++i) rb.push(i);
    CHECK_EQ(rb.size(), 4u);
    CHECK_EQ(rb.overflowCount(), 2u);
    uint8_t out[8] = {};
    CHECK_EQ(rb.pop(out, 8), 4u);
    CHECK_EQ(out[0], 1);  // oldest kept
    CHECK_EQ(out[3], 4);
    CHECK_EQ(rb.pop(out, 8), 0u);  // empty
}

TEST(ring_buffer_partial_pop)
{
    ByteRingBuffer<16> rb;
    for (uint8_t i = 0; i < 10; ++i) rb.push(i);
    uint8_t out[4] = {};
    CHECK_EQ(rb.pop(out, 4), 4u);
    CHECK_EQ(out[3], 3);
    CHECK_EQ(rb.size(), 6u);
}

TEST(sensor_health_counters)
{
    SensorHealth h;
    h.initialized = true;
    h.recordFailure(Status::BusError);
    h.recordFailure(Status::Timeout);
    CHECK_EQ(h.consecutiveFailures, 2);
    CHECK(h.isHealthy(3));
    h.recordFailure(Status::InvalidData);
    CHECK(!h.isHealthy(3));
    CHECK_EQ(h.busErrorCount, 1u);
    CHECK_EQ(h.timeoutCount, 1u);
    CHECK_EQ(h.invalidDataCount, 1u);
    h.recordSuccess(1234);
    CHECK_EQ(h.consecutiveFailures, 0);
    CHECK_EQ(h.lastSuccessMs, 1234u);
    CHECK(h.lastError == Status::InvalidData);  // last error remembered
}
