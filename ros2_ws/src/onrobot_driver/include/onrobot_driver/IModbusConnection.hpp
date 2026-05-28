#pragma once
#include <vector>
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"

class IModbusConnection {
public:
    virtual ~IModbusConnection() = default;
    // Sends a request and returns a parsed ModbusResponse.
    virtual MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) = 0;
    // Closes the connection.
    virtual void close() = 0;
};
