#ifndef SLAMBOT_INTERFACE_HPP
#define SLAMBOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>

#include <vector>
#include <string>

namespace slambot_firmware
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class SlambotInterface : public hardware_interface::SystemInterface
    {
    public:
        SlambotInterface();

        virtual ~SlambotInterface(); // destructor

        virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

        virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        LibSerial::SerialPort stm_;

        std::string port_;
        std::vector<double> velocity_commands_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;

        rclcpp::Time last_run_;

        std::vector<std::string> split(const std::string &s, char delimiter)
        {
            std::vector<std::string> result;
            std::stringstream ss(s);
            std::string item;

            while (getline(ss, item, delimiter))
            {
                result.push_back(item);
            }
            return result;
        }
    };

}

#endif