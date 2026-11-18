/**
 ******************************************************************************
 * @file    InterruptMask.hpp
 * @brief   Pure helper: would a Cortex-M interrupt of a given priority be
 *          blocked by the current PRIMASK / BASEPRI values?
 *
 * Portable (no CMSIS includes) so the rule can be unit-tested on a laptop.
 * The firmware passes in the real register values (__get_PRIMASK(),
 * __get_BASEPRI(), NVIC_GetPriority()).
 *
 * Cortex-M rules (ARMv7-M Architecture Reference Manual, B1.5.4):
 *   - PRIMASK bit 0 = 1 blocks every configurable-priority exception.
 *   - BASEPRI = 0 disables BASEPRI masking.
 *   - BASEPRI != 0 blocks every exception whose priority VALUE is greater than
 *     or equal to BASEPRI. (Lower value = more urgent.)
 *   - Priorities live in the top __NVIC_PRIO_BITS bits of an 8-bit field, so
 *     the NVIC priority number must be shifted left by (8 - prioBits) before
 *     it is compared with BASEPRI.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_COMMON_INTERRUPT_MASK_HPP
#define PARAGUARDIAN_COMMON_INTERRUPT_MASK_HPP

#include <cstdint>

namespace pg {

/**
 * @param primask      value of the PRIMASK register (bit 0 = interrupts disabled)
 * @param basepri      value of the BASEPRI register (already in the 8-bit shifted form)
 * @param irqPriority  NVIC priority number of the interrupt (0..2^prioBits-1, as NVIC_GetPriority returns)
 * @param prioBits     implemented priority bits (__NVIC_PRIO_BITS, 4 on STM32H7)
 * @return true if the interrupt cannot currently be taken because of PRIMASK/BASEPRI.
 */
constexpr bool interruptBlockedByMask(uint32_t primask, uint32_t basepri, uint32_t irqPriority, uint32_t prioBits)
{
    if ((primask & 1u) != 0u) {
        return true;
    }
    if ((basepri & 0xFFu) == 0u) {
        return false;
    }
    const uint32_t shiftedPriority = (irqPriority << (8u - prioBits)) & 0xFFu;
    return shiftedPriority >= (basepri & 0xFFu);
}

}  // namespace pg

#endif  // PARAGUARDIAN_COMMON_INTERRUPT_MASK_HPP
