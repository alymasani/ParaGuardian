/**
 ******************************************************************************
 * @file    Status.cpp
 * @brief   String names for pg::Status (debug output only).
 ******************************************************************************
 */
#include "Common/Status.hpp"

namespace pg {

const char* toString(Status s)
{
    switch (s) {
        case Status::Ok:              return "Ok";
        case Status::BusError:        return "BusError";
        case Status::Timeout:         return "Timeout";
        case Status::InvalidArgument: return "InvalidArgument";
        case Status::WrongDeviceId:   return "WrongDeviceId";
        case Status::NotInitialized:  return "NotInitialized";
        case Status::NotReady:        return "NotReady";
        case Status::ChecksumError:   return "ChecksumError";
        case Status::BufferOverflow:  return "BufferOverflow";
        case Status::VerifyFailed:    return "VerifyFailed";
        case Status::OutOfRange:      return "OutOfRange";
        case Status::Busy:            return "Busy";
        case Status::Unsupported:     return "Unsupported";
        case Status::InvalidData:     return "InvalidData";
    }
    return "Unknown";
}

}  // namespace pg
