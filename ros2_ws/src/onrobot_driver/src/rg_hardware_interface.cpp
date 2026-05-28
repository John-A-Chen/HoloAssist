#include "onrobot_driver/rg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rg_hardware_interface
{

    RGHardwareInterface::RGHardwareInterface()
        : finger_width_state_(0.0),
          finger_width_command_(0.0)
    {
    }

    RGHardwareInterface::~RGHardwareInterface()
    {
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        // Retrieve the gripper type from the hardware parameters.
        if (info.hardware_parameters.find("onrobot_type") != info.hardware_parameters.end())
        {
            onrobot_type_ = info.hardware_parameters.at("onrobot_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing onrobot_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Check for valid gripper type.
        if (onrobot_type_ != "rg2" && onrobot_type_ != "rg6")
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Invalid onrobot_type: %s", onrobot_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve the connection type from the hardware parameters.
        if (info.hardware_parameters.find("connection_type") != info.hardware_parameters.end())
        {
            connection_type_ = info.hardware_parameters.at("connection_type");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing connection_type parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // Check for TCP connection parameters.
        if (connection_type_ == "tcp")
        {
            if (info.hardware_parameters.find("ip_address") != info.hardware_parameters.end() &&
                info.hardware_parameters.find("port") != info.hardware_parameters.end())
            {
                ip_address_ = info.hardware_parameters.at("ip_address");
                port_ = std::stoi(info.hardware_parameters.at("port"));
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing ip_address or port for TCP connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        // Check for Serial connection parameters.
        else if (connection_type_ == "serial")
        {
            if (info.hardware_parameters.find("device") != info.hardware_parameters.end())
            {
                device_ = info.hardware_parameters.at("device");
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing device parameter for Serial connection");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Unsupported connection_type: %s", connection_type_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Retrieve the prefix from the hardware parameters.
        if (info.hardware_parameters.find("prefix") != info.hardware_parameters.end())
        {
            prefix_ = info.hardware_parameters.at("prefix");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Missing prefix parameter");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialise joint variables
        finger_width_state_ = 0.0;
        finger_width_command_ = 0.0;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
    {
        // Create the RG instance.
        try
        {
            if (connection_type_ == "tcp")
            {
                gripper_ = std::make_unique<RG>(onrobot_type_, ip_address_, port_);
            }
            else if (connection_type_ == "serial")
            {
                gripper_ = std::make_unique<RG>(onrobot_type_, device_);
            }

            // Get the starting width of the gripper.
            finger_width_state_ = gripper_->getWidthWithOffset();
            // Set the command to the current state to avoid moving to 0.0 at start.
            finger_width_command_ = finger_width_state_;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Failed to create RG instance: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &)
    {
        if (gripper_)
        {
            gripper_.reset();
            gripper_.release();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_shutdown(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn RGHardwareInterface::on_error(const rclcpp_lifecycle::State &)
    {
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> RGHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(prefix_ + "finger_width", "position", &finger_width_state_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> RGHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(prefix_ + "finger_width", "position", &finger_width_command_));
        return command_interfaces;
    }

    hardware_interface::return_type RGHardwareInterface::read(const rclcpp::Time &,
                                                              const rclcpp::Duration &)
    {
        std::lock_guard<std::mutex> lock(hw_interface_mutex_);
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Gripper not initialised");
            return hardware_interface::return_type::ERROR;
        }
        try
        {
            finger_width_state_ = gripper_->getWidthWithOffset();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Failed to read gripper state: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RGHardwareInterface::write(const rclcpp::Time &,
                                                               const rclcpp::Duration &)
    {
        std::lock_guard<std::mutex> lock(hw_interface_mutex_);
        if (!gripper_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Gripper not initialised");
            return hardware_interface::return_type::ERROR;
        }
        try
        {
            gripper_->moveGripper(finger_width_command_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("RGHardwareInterface"), "Failed to write command to gripper: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

} // namespace rg_hardware_interface

// Export the hardware interface as a plugin for ros2_control.
PLUGINLIB_EXPORT_CLASS(rg_hardware_interface::RGHardwareInterface, hardware_interface::ActuatorInterface)
