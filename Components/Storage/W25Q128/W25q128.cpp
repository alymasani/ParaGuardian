/**
 ******************************************************************************
 * @file    W25q128.cpp
 * @brief   W25Q128JV driver implementation. See W25q128.hpp.
 ******************************************************************************
 */
#include "Storage/W25Q128/W25q128.hpp"

#include "Common/ByteOrder.hpp"

namespace pg {

using namespace w25q;

bool W25q128::rangeIsValid(uint32_t address, size_t len)
{
    // Written to avoid overflow: check address first, then remaining space.
    return address < CAPACITY_BYTES && len <= (CAPACITY_BYTES - address);
}

Status W25q128::countBus(Status s)
{
    if (s == Status::BusError || s == Status::Timeout) {
        ++stats_.busErrors;
    }
    return s;
}

Status W25q128::readJedecId(JedecId& id)
{
    const uint8_t cmd = CMD_JEDEC_ID;
    uint8_t rx[3] = {};
    const Status s = countBus(spi_.writeThenRead(&cmd, 1u, rx, sizeof(rx)));
    if (isOk(s)) {
        id = {rx[0], rx[1], rx[2]};
    }
    return s;
}

Status W25q128::readStatus1(uint8_t& sr1)
{
    const uint8_t cmd = CMD_READ_STATUS1;
    return countBus(spi_.writeThenRead(&cmd, 1u, &sr1, 1u));
}

Status W25q128::init()
{
    initialized_ = false;

    // Release from power-down (harmless if not powered down). tRES1 = 3 us.
    const uint8_t wake = CMD_RELEASE_POWER_DOWN;
    Status s = countBus(spi_.writeThenWrite(&wake, 1u, nullptr, 0u));
    if (!isOk(s)) {
        return s;
    }
    clock_.delayMs(1u);

    JedecId id{};
    if (!isOk(s = readJedecId(id))) {
        return s;
    }
    if (id.manufacturer != MANUFACTURER_WINBOND ||
        (id.memoryType != MEMORY_TYPE_SPI && id.memoryType != MEMORY_TYPE_DTR) || id.capacity != CAPACITY_128MBIT) {
        return Status::WrongDeviceId;
    }

    // A previous program/erase may still be running after an MCU reset.
    if (!isOk(s = waitUntilReady(BLOCK_ERASE_TIMEOUT_MS))) {
        return s;
    }
    initialized_ = true;
    return Status::Ok;
}

Status W25q128::waitUntilReady(uint32_t timeoutMs)
{
    const uint32_t start = clock_.millis();
    for (;;) {
        uint8_t sr1 = 0u;
        const Status s = readStatus1(sr1);
        if (!isOk(s)) {
            return s;
        }
        if ((sr1 & SR1_BUSY) == 0u) {
            return Status::Ok;
        }
        if ((clock_.millis() - start) >= timeoutMs) {
            ++stats_.busyTimeouts;
            return Status::Timeout;
        }
        clock_.delayMs(1u);
    }
}

Status W25q128::writeEnable()
{
    const uint8_t cmd = CMD_WRITE_ENABLE;
    Status s = countBus(spi_.writeThenWrite(&cmd, 1u, nullptr, 0u));
    if (!isOk(s)) {
        return s;
    }
    // Confirm the Write Enable Latch really got set (it will not if the chip
    // is busy, or /WP + status-register protection blocks it).
    uint8_t sr1 = 0u;
    if (!isOk(s = readStatus1(sr1))) {
        return s;
    }
    return ((sr1 & SR1_WEL) != 0u) ? Status::Ok : Status::VerifyFailed;
}

Status W25q128::read(uint32_t address, uint8_t* dst, size_t len)
{
    if (!initialized_) {
        return Status::NotInitialized;
    }
    if (dst == nullptr || len == 0u) {
        return Status::InvalidArgument;
    }
    if (!rangeIsValid(address, len)) {
        return Status::OutOfRange;
    }
    uint8_t header[4] = {CMD_READ_DATA, 0, 0, 0};
    bytes::put_be_u24(&header[1], address);
    return countBus(spi_.writeThenRead(header, sizeof(header), dst, len));
}

Status W25q128::pageProgram(uint32_t address, const uint8_t* src, size_t len)
{
    if (!initialized_) {
        return Status::NotInitialized;
    }
    if (src == nullptr || len == 0u || len > PAGE_SIZE) {
        return Status::InvalidArgument;
    }
    if (!rangeIsValid(address, len)) {
        return Status::OutOfRange;
    }
    const uint32_t offsetInPage = address % PAGE_SIZE;
    if (offsetInPage + len > PAGE_SIZE) {
        return Status::InvalidArgument;  // would wrap inside the page on the real chip
    }

    Status s = writeEnable();
    if (!isOk(s)) {
        return s;
    }
    uint8_t header[4] = {CMD_PAGE_PROGRAM, 0, 0, 0};
    bytes::put_be_u24(&header[1], address);
    if (!isOk(s = countBus(spi_.writeThenWrite(header, sizeof(header), src, len)))) {
        return s;
    }
    if (!isOk(s = waitUntilReady(PAGE_PROGRAM_TIMEOUT_MS))) {
        return s;
    }
    ++stats_.pagesProgrammed;
    return Status::Ok;
}

Status W25q128::write(uint32_t address, const uint8_t* src, size_t len, bool verify)
{
    if (!initialized_) {
        return Status::NotInitialized;
    }
    if (src == nullptr || len == 0u) {
        return Status::InvalidArgument;
    }
    if (!rangeIsValid(address, len)) {
        return Status::OutOfRange;
    }
    size_t done = 0u;
    while (done < len) {
        const uint32_t addr = address + static_cast<uint32_t>(done);
        const size_t roomInPage = PAGE_SIZE - (addr % PAGE_SIZE);
        const size_t chunk = (len - done < roomInPage) ? (len - done) : roomInPage;
        Status s = pageProgram(addr, src + done, chunk);
        if (!isOk(s)) {
            return s;
        }
        if (verify) {
            uint8_t check[PAGE_SIZE];
            if (!isOk(s = read(addr, check, chunk))) {
                return s;
            }
            for (size_t i = 0; i < chunk; ++i) {
                if (check[i] != src[done + i]) {
                    ++stats_.verifyFailures;
                    return Status::VerifyFailed;
                }
            }
        }
        done += chunk;
    }
    return Status::Ok;
}

Status W25q128::eraseCommand(uint8_t command, uint32_t address, uint32_t alignment, uint32_t timeoutMs)
{
    if (!initialized_) {
        return Status::NotInitialized;
    }
    if (address >= CAPACITY_BYTES) {
        return Status::OutOfRange;
    }
    if ((address % alignment) != 0u) {
        return Status::InvalidArgument;
    }
    Status s = writeEnable();
    if (!isOk(s)) {
        return s;
    }
    uint8_t frame[4] = {command, 0, 0, 0};
    bytes::put_be_u24(&frame[1], address);
    if (!isOk(s = countBus(spi_.writeThenWrite(frame, sizeof(frame), nullptr, 0u)))) {
        return s;
    }
    return waitUntilReady(timeoutMs);
}

Status W25q128::eraseSector(uint32_t address)
{
    const Status s = eraseCommand(CMD_SECTOR_ERASE_4K, address, SECTOR_SIZE, SECTOR_ERASE_TIMEOUT_MS);
    if (isOk(s)) {
        ++stats_.sectorsErased;
    }
    return s;
}

Status W25q128::eraseBlock64k(uint32_t address)
{
    return eraseCommand(CMD_BLOCK_ERASE_64K, address, BLOCK_SIZE_64K, BLOCK_ERASE_TIMEOUT_MS);
}

}  // namespace pg
