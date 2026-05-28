#pragma once
#include "IModbusConnection.hpp"
#include "MB/Serial/connection.hpp"
#include <string>
#include <tuple>

class SerialConnectionWrapper : public IModbusConnection {
public:
    // Create a Serial connection using the provided device path.
    SerialConnectionWrapper(const std::string &device)
    : connection(device) {
        connection.connect();
        connection.setTwoStopBits(false); // Use one stop bit
        connection.setEvenParity();
        connection.setBaudRate(115200);
        connection.setTimeout(1000);
    }

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override {
        try {
            // Send the raw request bytes.
            connection.send(req.toRaw());
            // Wait for the complete response.
            auto [response, rawData] = connection.awaitResponse();
            return response;
        } catch (const MB::ModbusException &ex) {
            throw;
        }
    }

    void close() override {
        // Do nothing at the moment. Can explicitly close the connection if needed.
    }

private:
    MB::Serial::Connection connection;
};
