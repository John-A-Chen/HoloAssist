#pragma once
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>

#include "onrobot_driver/IModbusConnection.hpp"
#include "onrobot_driver/TCPConnectionWrapper.hpp"
#include "onrobot_driver/SerialConnectionWrapper.hpp"

// Modbus request/response definitions and utilities
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "MB/modbusUtils.hpp"

class RG {
public:
    // Constructors for TCP and Serial connections
    RG(const std::string &type, const std::string &ip, int port);
    RG(const std::string &type, const std::string &device);
    ~RG();

    // Read commands
    float getFingertipOffset();
    float getWidth();
    std::vector<int> getStatus();
    std::vector<int> getStatusAndPrint();
    float getWidthWithOffset();

    // Write commands
    void setFingertipOffset(float offset_val);
    void setTargetForce(float force_val);
    void setTargetWidth(float width_val);
    void setControlMode(uint16_t command);
    
    // Gripper control commands
    void closeGripper();
    void openGripper();
    void moveGripper(float width_val);

private:
    std::unique_ptr<IModbusConnection> connection;
    std::string type;
    float max_width;
    float max_force;

    // Default parameters
    float default_force;
    float default_fingertip_offset;

    // Constants for registers and commands
    static constexpr uint16_t DEVICE_ID = 65;
    static constexpr uint16_t REG_TARGET_FORCE = 0;
    static constexpr uint16_t REG_TARGET_WIDTH = 1;
    static constexpr uint16_t REG_CONTROL = 2;
    static constexpr uint16_t REG_FINGERTIP_OFFSET = 258;
    static constexpr uint16_t REG_ACTUAL_DEPTH = 263;
    static constexpr uint16_t REG_ACTUAL_RELATIVE_DEPTH = 264;
    static constexpr uint16_t REG_ACTUAL_WIDTH = 267;
    static constexpr uint16_t REG_STATUS = 268;
    static constexpr uint16_t REG_ACTUAL_WIDTH_WITH_OFFSET = 275;
    static constexpr uint16_t REG_SET_FINGERTIP_OFFSET = 1031;

    // Control commands
    static constexpr uint16_t CMD_GRIP = 1;
    static constexpr uint16_t CMD_STOP = 8;
    static constexpr uint16_t CMD_GRIP_WITH_OFFSET = 16;

    // Helper function to send a MODBUS request
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
};
