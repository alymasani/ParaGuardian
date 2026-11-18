/**
 ******************************************************************************
 * @file    HalUartPort.hpp
 * @brief   UART adapter: interrupt-driven receive into a ring buffer
 *          (IByteSource) + blocking transmit (IByteWriter).
 *
 * Compiled only when CubeMX enables a UART/USART with the HAL driver
 * (HAL_UART_MODULE_ENABLED). Not compiled today.
 *
 * RECEIVE PATH
 *   start() arms HAL_UART_Receive_IT for ONE byte. When it arrives, HAL calls
 *   HAL_UART_RxCpltCallback(huart) in interrupt context. That callback is a
 *   global weak C function that CubeMX-generated code may also want to own, so
 *   ParaGuardian does NOT define it. Instead, add to main.c USER CODE 4:
 *
 *       void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 *       { paraguardian_uart_rx_complete(huart); }
 *       void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 *       { paraguardian_uart_error(huart); }
 *
 *   The ISR pushes the byte into a 1024-byte ring buffer and re-arms. The
 *   acquisition task later drains the buffer with read().
 *
 *   One interrupt per byte is fine at GNSS/LiDAR rates (<= 115200 baud =
 *   ~11.5 k interrupts/s). Upgrade path: HAL_UARTEx_ReceiveToIdle_DMA with a
 *   circular buffer in a DMA-accessible RAM region (not DTCM).
 *
 * The UART interrupt priority must be numerically >= 5
 * (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY) if the ISR ever calls a
 * FreeRTOS "FromISR" function. This adapter does not, but keep it >= 5 anyway.
 ******************************************************************************
 */
#ifndef PARAGUARDIAN_BSP_HAL_UART_PORT_HPP
#define PARAGUARDIAN_BSP_HAL_UART_PORT_HPP

#include "main.h"

#if defined(HAL_UART_MODULE_ENABLED)

#include <cstdint>

#include "Common/BusInterfaces.hpp"
#include "Common/ByteRingBuffer.hpp"

namespace pg {
namespace bsp {

class HalUartPort final : public IByteSource, public IByteWriter {
public:
    // 1024 B = ~89 ms of continuous data at 115200 baud (11.5 B/ms), so the
    // buffer survives a stalled acquisition cycle (e.g. a 50 ms sensor reset
    // wait) without overflowing. Must be a power of two.
    static constexpr uint32_t kRxBufferSize = 1024u;
    static constexpr size_t kMaxPorts = 4u;

    HalUartPort(UART_HandleTypeDef* huart, uint32_t txTimeoutMs) : huart_(huart), txTimeoutMs_(txTimeoutMs) {}

    /// Registers the port for callback dispatch and arms reception. Call from a task.
    Status start();

    size_t read(uint8_t* dst, size_t maxLen) override { return rx_.pop(dst, maxLen); }
    Status write(const uint8_t* data, size_t len) override;

    uint32_t rxOverflows() const { return rx_.overflowCount(); }
    uint32_t rxErrors() const { return rxErrors_; }

    // Called from the C callbacks (interrupt context).
    static void dispatchRxComplete(UART_HandleTypeDef* huart);
    static void dispatchError(UART_HandleTypeDef* huart);

private:
    void onRxComplete();
    void onError();
    void arm();

    UART_HandleTypeDef* huart_;
    uint32_t txTimeoutMs_;
    uint8_t rxByte_ = 0u;
    volatile uint32_t rxErrors_ = 0u;
    ByteRingBuffer<kRxBufferSize> rx_{};
};

}  // namespace bsp
}  // namespace pg

extern "C" void paraguardian_uart_rx_complete(UART_HandleTypeDef* huart);
extern "C" void paraguardian_uart_error(UART_HandleTypeDef* huart);

#endif  // HAL_UART_MODULE_ENABLED
#endif  // PARAGUARDIAN_BSP_HAL_UART_PORT_HPP
