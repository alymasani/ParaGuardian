/**
 ******************************************************************************
 * @file    HalUartPort.cpp
 * @brief   See HalUartPort.hpp.
 ******************************************************************************
 */
#include "BSP/HalUartPort.hpp"

#if defined(HAL_UART_MODULE_ENABLED)

namespace pg {
namespace bsp {

namespace {
// Fixed registry of started ports (no heap). Written before the matching
// UART interrupt is armed, read from the ISR afterwards.
HalUartPort* g_ports[HalUartPort::kMaxPorts] = {};
UART_HandleTypeDef* g_handles[HalUartPort::kMaxPorts] = {};
}  // namespace

Status HalUartPort::start()
{
    if (huart_ == nullptr) {
        return Status::InvalidArgument;
    }
    for (size_t i = 0; i < kMaxPorts; ++i) {
        if (g_ports[i] == nullptr || g_ports[i] == this) {
            g_handles[i] = huart_;
            g_ports[i] = this;
            arm();
            return Status::Ok;
        }
    }
    return Status::BufferOverflow;  // registry full
}

void HalUartPort::arm()
{
    if (HAL_UART_Receive_IT(huart_, &rxByte_, 1u) != HAL_OK) {
        ++rxErrors_;
    }
}

void HalUartPort::onRxComplete()
{
    rx_.push(rxByte_);  // drops + counts if the task has not drained the buffer
    arm();
}

void HalUartPort::onError()
{
    // Overrun / framing / noise: HAL has already aborted the transfer.
    ++rxErrors_;
    arm();
}

Status HalUartPort::write(const uint8_t* data, size_t len)
{
    if (data == nullptr || len == 0u || len > UINT16_MAX) {
        return Status::InvalidArgument;
    }
    const HAL_StatusTypeDef hs =
        HAL_UART_Transmit(huart_, const_cast<uint8_t*>(data), static_cast<uint16_t>(len), txTimeoutMs_);
    return (hs == HAL_OK) ? Status::Ok : (hs == HAL_TIMEOUT ? Status::Timeout : Status::BusError);
}

void HalUartPort::dispatchRxComplete(UART_HandleTypeDef* huart)
{
    for (size_t i = 0; i < kMaxPorts; ++i) {
        if (g_handles[i] == huart && g_ports[i] != nullptr) {
            g_ports[i]->onRxComplete();
            return;
        }
    }
}

void HalUartPort::dispatchError(UART_HandleTypeDef* huart)
{
    for (size_t i = 0; i < kMaxPorts; ++i) {
        if (g_handles[i] == huart && g_ports[i] != nullptr) {
            g_ports[i]->onError();
            return;
        }
    }
}

}  // namespace bsp
}  // namespace pg

extern "C" void paraguardian_uart_rx_complete(UART_HandleTypeDef* huart)
{
    pg::bsp::HalUartPort::dispatchRxComplete(huart);
}

extern "C" void paraguardian_uart_error(UART_HandleTypeDef* huart)
{
    pg::bsp::HalUartPort::dispatchError(huart);
}

#endif  // HAL_UART_MODULE_ENABLED
