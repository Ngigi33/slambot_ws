// #include "slambot_hardware_interface.hpp"
// #include <rclcpp/rclcpp.hpp>

// namespace slambot_firmware
// {

//     SlambotInterface::SlambotInterface()
//     {
//     }

//     CallbackReturn SlambotInterface::on_init(const hardware_interface::HardwareInfo &info)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("slambot_interface"), "....Initializing Hardware Interface....");
//         CallbackReturn result = hardware_interface::SystemInterface::on_init(info);

//         if (result != CallbackReturn::SUCCESS)
//         {
//             return result;
//         }
//         try
//         {
//             port_ = info_.hardware_parameters.at("port");
//         }
//         catch (const std::exception &e)
//         {
//             RCLCPP_FATAL(rclcpp::get_logger("slambot_interface"), "No Serial POrt provided! Aborting!");
//         }
//     }

// }
