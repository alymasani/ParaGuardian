/**
 ******************************************************************************
 * @file    BusInterfaces.hpp
 * @brief   Abstract SPI / I2C / UART / clock interfaces used by every driver.
 *
 * WHY INTERFACES?
 *   A driver such as Lsm6dsrx only needs "send these bytes, give me those
 *   bytes". It must NOT know that the board uses hspi2 with chip-select on
 *   PB12 (the schematic does not exist yet), and it must be testable on a
 *   laptop where there is no SPI peripheral at all.
 *
 *   So drivers talk to these small abstract classes. On the STM32 the BSP
 *   layer (Components/BSP) provides implementations that call HAL_SPI_* etc.
 *   In the host unit tests (Tests/mocks) fake implementations record every
 *   transaction and return scripted bytes.
 *
 * LIFETIME RULE
 *   Drivers keep a *reference* to the bus object they are given. The bus
 *   object must therefore live at least as long as the driver. In firmware
 *   both are static objects that live forever, so this is automatic.
 *
 * DESTRUCTORS
 *   The destructors are protected and non-virtual. Nobody ever deletes a bus
 *   through an interface pointer (there is no heap use), and leaving the
 *   destructor non-virtual stops the compiler from emitting a "deleting
 *   destructor" that would pull operator delete into the firmware.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_COMMON_BUS_INTERFACES_HPP
#define PARAGUARDIAN_COMMON_BUS_INTERFACES_HPP

#include <cstddef>
#include <cstdint>

#include "Common/Status.hpp"

namespace pg {

/**
 * One SPI *device* (= one chip-select line on some SPI bus).
 * The implementation asserts chip-select (drives it low) at the start of each
 * call and releases it at the end, so each call is exactly one SPI transaction.
 */
class ISpiDevice {
public:
    /**
     * Transmit @p txLen bytes, then clock in @p rxLen bytes (the device sees
     * dummy 0x00 bytes on MOSI while we read), all under ONE chip-select assertion.
     * @p rx may be nullptr only when @p rxLen is 0.
     */
    virtual Status writeThenRead(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) = 0;

    /**
     * Transmit @p headerLen bytes followed by @p payloadLen bytes under ONE
     * chip-select assertion (e.g. flash command + address, then page data).
     * @p payload may be nullptr only when @p payloadLen is 0.
     */
    virtual Status writeThenWrite(const uint8_t* header, size_t headerLen, const uint8_t* payload,
                                  size_t payloadLen) = 0;

protected:
    ~ISpiDevice() = default;
};

/**
 * One I2C bus (several devices with different addresses can share it).
 * Addresses are 7-bit (e.g. 0x6A), NOT pre-shifted.
 */
class II2cBus {
public:
    /// START, address+W, @p len data bytes, STOP.
    virtual Status write(uint8_t address7, const uint8_t* data, size_t len) = 0;

    /// START, address+W, tx bytes, REPEATED START, address+R, rx bytes, STOP.
    virtual Status writeThenRead(uint8_t address7, const uint8_t* tx, size_t txLen, uint8_t* rx,
                                 size_t rxLen) = 0;

protected:
    ~II2cBus() = default;
};

/// Byte-oriented transmit side of a UART (GNSS configuration, Bluetooth commands).
class IByteWriter {
public:
    virtual Status write(const uint8_t* data, size_t len) = 0;

protected:
    ~IByteWriter() = default;
};

/// Receive side of a UART after the interrupt has buffered the bytes.
class IByteSource {
public:
    /// Copies up to @p maxLen buffered bytes into @p dst; returns how many were copied (0 = none).
    virtual size_t read(uint8_t* dst, size_t maxLen) = 0;

protected:
    ~IByteSource() = default;
};

/**
 * Time source + delay. Drivers need to wait (reset times, busy polling) and
 * to measure timeouts. On the STM32 this is backed by the FreeRTOS tick
 * (1 ms). In tests a fake clock advances instantly so tests run fast.
 *
 * millis() wraps after ~49.7 days; always compare times with unsigned
 * subtraction:  (now - start) >= timeout   which stays correct across a wrap.
 */
class IClock {
public:
    virtual uint32_t millis() = 0;
    virtual void delayMs(uint32_t ms) = 0;

protected:
    ~IClock() = default;
};

}  // namespace pg

#endif  // PARAGUARDIAN_COMMON_BUS_INTERFACES_HPP
