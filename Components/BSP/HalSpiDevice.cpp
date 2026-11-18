/**
 ******************************************************************************
 * @file    HalSpiDevice.cpp
 * @brief   See HalSpiDevice.hpp. Compiled out unless HAL_SPI_MODULE_ENABLED.
 ******************************************************************************
 */
#include "BSP/HalSpiDevice.hpp"

#if defined(HAL_SPI_MODULE_ENABLED)

namespace pg {
namespace bsp {

Status HalSpiDevice::fromHal(HAL_StatusTypeDef s)
{
    switch (s) {
        case HAL_OK:      return Status::Ok;
        case HAL_TIMEOUT: return Status::Timeout;
        case HAL_BUSY:    return Status::Busy;
        case HAL_ERROR:
        default:          return Status::BusError;
    }
}

bool HalSpiDevice::lock()
{
    return (busLock_ == nullptr) || busLock_->Lock(timeoutMs_);
}

void HalSpiDevice::unlock()
{
    if (busLock_ != nullptr) {
        busLock_->Unlock();
    }
}

Status HalSpiDevice::writeThenRead(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen)
{
    if (hspi_ == nullptr || (txLen > 0u && tx == nullptr) || (rxLen > 0u && rx == nullptr) ||
        txLen > UINT16_MAX || rxLen > UINT16_MAX) {
        return Status::InvalidArgument;
    }
    if (!lock()) {
        return Status::Timeout;
    }
    select();
    HAL_StatusTypeDef hs = HAL_OK;
    if (txLen > 0u) {
        // HAL takes a non-const pointer but only reads from it when transmitting.
        hs = HAL_SPI_Transmit(hspi_, const_cast<uint8_t*>(tx), static_cast<uint16_t>(txLen), timeoutMs_);
    }
    if (hs == HAL_OK && rxLen > 0u) {
        hs = HAL_SPI_Receive(hspi_, rx, static_cast<uint16_t>(rxLen), timeoutMs_);
    }
    deselect();  // always release CS, even after an error
    unlock();
    return fromHal(hs);
}

Status HalSpiDevice::writeThenWrite(const uint8_t* header, size_t headerLen, const uint8_t* payload,
                                    size_t payloadLen)
{
    if (hspi_ == nullptr || (headerLen > 0u && header == nullptr) || (payloadLen > 0u && payload == nullptr) ||
        headerLen > UINT16_MAX || payloadLen > UINT16_MAX) {
        return Status::InvalidArgument;
    }
    if (!lock()) {
        return Status::Timeout;
    }
    select();
    HAL_StatusTypeDef hs = HAL_OK;
    if (headerLen > 0u) {
        hs = HAL_SPI_Transmit(hspi_, const_cast<uint8_t*>(header), static_cast<uint16_t>(headerLen), timeoutMs_);
    }
    if (hs == HAL_OK && payloadLen > 0u) {
        hs = HAL_SPI_Transmit(hspi_, const_cast<uint8_t*>(payload), static_cast<uint16_t>(payloadLen), timeoutMs_);
    }
    deselect();
    unlock();
    return fromHal(hs);
}

}  // namespace bsp
}  // namespace pg

#endif  // HAL_SPI_MODULE_ENABLED
