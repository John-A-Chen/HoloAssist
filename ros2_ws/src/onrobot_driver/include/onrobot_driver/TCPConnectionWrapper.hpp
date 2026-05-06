#pragma once
#include "IModbusConnection.hpp"
#include "MB/TCP/connection.hpp"
#include <string>

class TCPConnectionWrapper : public IModbusConnection {
public:
    // Create a TCP connection using the library’s static with() method.
    TCPConnectionWrapper(const std::string &ip, int port) 
      : connection(MB::TCP::Connection::with(ip, port)) {}

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override {
        try {
            // Send the request.
            connection.sendRequest(req);
            // Wait for the complete response and return it.
            return connection.awaitResponse();
        } catch (const MB::ModbusException &ex) {
            throw;
        }
    }

    void close() override {
        // Do nothing at the moment. Can explicitly close the connection if needed.
    }

private:
    MB::TCP::Connection connection;
};
