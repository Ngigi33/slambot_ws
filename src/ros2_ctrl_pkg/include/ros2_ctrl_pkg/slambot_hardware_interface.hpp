// #ifndef SLAMBOT_INTERFACE_HPP
// #define SLAMBOT_INTERFACE_HPP

// #include "hardware_interface/system_interface.hpp"
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_lifecycle/state.hpp>
// #include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
// #include <libserial/SerialPort.h>
// #include <rclcpp_lifecycle/lifecycle_publisher.hpp>
// #include <hardware_interface/hardware_info.hpp>

// // #include "hardware_interface/hardware_component_info.hpp"

// namespace slambot_firmware

// {
//     using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

//     class SlambotInterface : public hardware_interface::SystemInterface
//     {
//     public:
//         SlambotInterface();

//         // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
//         CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
//         CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
//         CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
//         CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
//         CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

//         // Implementing hardware_interface::SystemInterface
//         std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
//         std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
//         hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
//         hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

//     private:
//         LibSerial::SerialPort stm_;
//         std::string port_;
//     };

// }

// #endif