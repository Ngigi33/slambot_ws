#include "slambot_firmware/slambot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

// #include <string>
// #include <vector>
// #include <iostream>
// #include <sstream>
// #include <std_msgs/msg/string.hpp>
// #include "example_interfaces/msg/string.hpp"
// #include <std_msgs/msg/int16.hpp>
// #include <std_msgs/msg/float64.hpp>

namespace slambot_firmware
{
    SlambotInterface::SlambotInterface()
    {
    }
    SlambotInterface::~SlambotInterface()
    {
        if (stm_.IsOpen())
        {
            try
            {
                stm_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("SlambotInterface"), "Error while Closing Port" << port_);
            }
        }
    }

    CallbackReturn SlambotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS)
        {
            return result;
        }
        try
        {
            port_ = info_.hardware_parameters.at("port");
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("SlambotInterface"), "NO serial PORT !!! ABORTING !!!!!");
            return CallbackReturn::FAILURE;
        }

        // setting memory space
        velocity_commands_.reserve(info_.joints.size());
        position_states_.reserve(info_.joints.size());
        velocity_states_.reserve(info_.joints.size());
        last_run_ = rclcpp::Clock().now();

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SlambotInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
                                                                             hardware_interface::HW_IF_POSITION, &position_states_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name,
                                                                             hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SlambotInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
                                                                                 hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn SlambotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("SlambotInterface"), " Starting Robot HARDWARE : Hurrah !!1");
        velocity_commands_ = {0.0, 0.0};
        position_states_ = {0.0, 0.0};
        velocity_states_ = {0.0, 0.0};

        try
        {
            stm_.Open(port_);
            stm_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch (...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("SlambotInterface"), " Error connecting to port" << port_);
            return CallbackReturn::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("SlambotInterface"), "Hardware Started , ready for Commands ..............");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn SlambotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("SlambotInterface"), " Stoppong the RoBot !!!!!!!!!");
        if (stm_.IsOpen())
        {
            try
            {
                stm_.Close();
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("SlambotInterface"), " Error connecting to port" << port_);
                return CallbackReturn::FAILURE;
            }
        }
    }

    hardware_interface::return_type SlambotInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        if (stm_.IsDataAvailable())
        {
            auto dt = (rclcpp::Clock().now() - last_run_).seconds();
            std::string message;
            stm_.ReadLine(message);

            // nb: if error in reading , know the splitting part is the issue

            std::vector<std::string> text = split(message, ',');
            if (text.size() < 2)
            {
                RCLCPP_ERROR(rclcpp::get_logger("SlambotInterface"), "Invalid message received: %s. Expected at least 4 parts.", message.c_str());
                // Exit the function early if the message is invalid
            }

            try
            {
                // Convert the strings to float
                double right_rpm = std::stod(text[0]);
                double left_rpm = std::stod(text[1]);

                RCLCPP_INFO_STREAM(rclcpp::get_logger("SlambotInterface"), "Right_RPM,leftRPM : " << right_rpm<<","<< left_rpm);

                double right_angular_vel = (right_rpm * 2.0 * M_PI) / 60.0;
                double left_angular_vel = (left_rpm * 2.0 * M_PI) / 60.0;

                velocity_states_.at(0) = right_angular_vel;
                position_states_.at(0) += velocity_states_.at(0) * dt;
                velocity_states_.at(1) = left_angular_vel;
                position_states_.at(1) += velocity_states_.at(1) * dt;
            }
            catch (const std::invalid_argument &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("SlambotInterface"), "Invalid number format in message: %s", message.c_str());
            }
            catch (const std::out_of_range &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("SlambotInterface"), "Number out of range in message: %s", message.c_str());
            }
            last_run_ = rclcpp::Clock().now();
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SlambotInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {

        // Implement communication protocol with the STM32
        std::stringstream message_stream;
        char right_wheel_sign = velocity_commands_.at(0) >= 0 ? '1' : '2';
        char left_wheel_sign = velocity_commands_.at(1) >= 0 ? '1' : '2';
        std::string compensate_zeros_right = "";
        std::string compensate_zeros_left = "";

        if (std::abs(velocity_commands_.at(0)) < 10.5)
        {
            compensate_zeros_right = "0";
        }
        else
        {
            compensate_zeros_right = "";
        }
        if (std::abs(velocity_commands_.at(1)) < 10.5)
        {
            compensate_zeros_left = "0";
        }
        else
        {
            compensate_zeros_left = "";
        }

        int right_wheel_cmd = int((std::abs(velocity_commands_.at(0)) * 60.0) / (2 * M_PI));
        int left_wheel_cmd = int((std::abs(velocity_commands_.at(1)) * 60.0) / (2 * M_PI));

        // message_stream << right_wheel_sign << compensate_zeros_right << right_wheel_cmd
        //                << left_wheel_sign << compensate_zeros_left << left_wheel_cmd << "\n";
        message_stream << right_wheel_sign << "," << right_wheel_cmd << ","
                       << left_wheel_sign << "," << left_wheel_cmd << "\n";

        try
        {
            stm_.Write(message_stream.str());
        }
        catch (...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("SlambotInterface"), "Error while sending message "
                                                                            << message_stream.str() << " on Port" << port_);
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(slambot_firmware::SlambotInterface, hardware_interface::SystemInterface);