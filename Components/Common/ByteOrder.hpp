/**
 ******************************************************************************
 * @file    ByteOrder.hpp
 * @brief   Helpers that assemble multi-byte values from raw sensor bytes.
 *
 * Sensors send numbers one byte at a time. These helpers glue the bytes back
 * together explicitly with shifts, so the result does not depend on the CPU's
 * own byte order and never performs an unaligned memory access.
 *
 *   little-endian ("LE"): least significant byte first  (LSM6DSRX, BMP585, UBX, TF02-Pro)
 *   big-endian    ("BE"): most significant byte first   (W25Q128 addresses)
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_COMMON_BYTE_ORDER_HPP
#define PARAGUARDIAN_COMMON_BYTE_ORDER_HPP

#include <cstdint>

namespace pg {
namespace bytes {

/// Two bytes, low first, as an unsigned 16-bit value: p[0] | p[1] << 8.
constexpr uint16_t le_u16(const uint8_t* p)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

/// Two bytes, low first, interpreted as two's-complement signed 16-bit.
/// Example: {0x00, 0x80} -> 0x8000 -> -32768;  {0xFF, 0xFF} -> -1.
constexpr int16_t le_i16(const uint8_t* p)
{
    return static_cast<int16_t>(le_u16(p));
}

/// Three bytes, low first, as an unsigned 24-bit value (held in 32 bits).
constexpr uint32_t le_u24(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16);
}

/// Three bytes, low first, as a signed 24-bit two's-complement value.
/// Bit 23 is the sign bit; if it is set we "sign-extend" by filling bits 24..31 with 1s.
constexpr int32_t le_i24(const uint8_t* p)
{
    uint32_t v = le_u24(p);
    if ((v & 0x00800000u) != 0u) {
        v |= 0xFF000000u;
    }
    return static_cast<int32_t>(v);
}

/// Four bytes, low first, unsigned.
constexpr uint32_t le_u32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

/// Four bytes, low first, signed two's complement.
constexpr int32_t le_i32(const uint8_t* p)
{
    return static_cast<int32_t>(le_u32(p));
}

/// Writes a 16-bit value low byte first.
inline void put_le_u16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

/// Writes a 32-bit value low byte first.
inline void put_le_u32(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

/// Writes the low 24 bits of @p v most-significant byte first (flash addresses).
inline void put_be_u24(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>((v >> 16) & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
    p[2] = static_cast<uint8_t>(v & 0xFFu);
}

}  // namespace bytes
}  // namespace pg

#endif  // PARAGUARDIAN_COMMON_BYTE_ORDER_HPP
