#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eigen3/Eigen/Core>

class SimpleController : public rclcpp::Node
{
private:
    void vel_callback(const geometry_msgs::msg::TwistStamped &msg);

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub;

    double wheel_radius;
    double wheel_separation;
    Eigen::Matrix2d speed_conversion;

public:
    SimpleController(const std::string &name);
};

#endif
