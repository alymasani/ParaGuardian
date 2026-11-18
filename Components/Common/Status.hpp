/**
 ******************************************************************************
 * @file    Status.hpp
 * @brief   ParaGuardian-wide result code returned by every driver function.
 *
 * Portable: no HAL / FreeRTOS includes. Compiles on the STM32 and on a laptop.
 *
 * Every function that can fail returns a Status instead of throwing an
 * exception (exceptions are disabled in the firmware build: -fno-exceptions)
 * or asserting (an assert resets the MCU, which is wrong for a recoverable
 * fault such as a sensor that briefly stops answering).
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_COMMON_STATUS_HPP
#define PARAGUARDIAN_COMMON_STATUS_HPP

#include <cstdint>

namespace pg {

enum class Status : uint8_t {
    Ok = 0,           ///< Operation succeeded.
    BusError,         ///< The SPI/I2C/UART peripheral reported an error (NACK, overrun, HAL_ERROR...).
    Timeout,          ///< Waited longer than the allowed time (bus or device busy flag).
    InvalidArgument,  ///< Caller passed a null pointer, zero length, out-of-range enum, etc.
    WrongDeviceId,    ///< WHO_AM_I / CHIP_ID / JEDEC ID did not match the expected part.
    NotInitialized,   ///< Driver used before a successful init().
    NotReady,         ///< Device answered but has no new data / is not ready yet.
    ChecksumError,    ///< A received frame failed its checksum.
    BufferOverflow,   ///< A bounded buffer would have overflowed; data was dropped.
    VerifyFailed,     ///< Read-back of a configuration register did not match what was written.
    OutOfRange,       ///< Address/length outside the device or a measurement outside sensor range.
    Busy,             ///< Device is busy with a previous operation (e.g. flash erase).
    Unsupported,      ///< Requested feature is not implemented / not supported by this part.
    InvalidData,      ///< Data was received but is implausible (e.g. all 0x00 or all 0xFF).
};

/// True when @p s is Status::Ok. Reads better than comparing in every if().
constexpr bool isOk(Status s) { return s == Status::Ok; }

/// Short human-readable name, for debug prints. Never returns nullptr.
const char* toString(Status s);

}  // namespace pg

#endif  // PARAGUARDIAN_COMMON_STATUS_HPP
