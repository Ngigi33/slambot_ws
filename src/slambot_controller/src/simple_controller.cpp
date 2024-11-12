#include "slambot_controller/simple_controller.hpp"
#include <eigen3/Eigen/Geometry>

SimpleController::SimpleController(const std::string &name)
    : Node(name)
{
    declare_parameter("wheel_radius", 0.0425);
    declare_parameter("wheel_separation", 0.245);

    wheel_radius = get_parameter("wheel_radius").as_double();
    wheel_separation = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Wheel_radius==" << wheel_radius);
    RCLCPP_INFO_STREAM(get_logger(), "wheel_separation==" << wheel_separation);

    wheel_cmd_pub = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
        // wheel_cmd_pub = create_publisher<std_msgs::msg::Float64MultiArray>("/slambot_diffdrive_controller/cmd_vel", 10);
    vel_sub = create_subscription<geometry_msgs::msg::TwistStamped>("/slambot/cmd_vel_out", 10, std::bind(&SimpleController::vel_callback, this, std::placeholders::_1));

    speed_conversion << wheel_radius / 2, wheel_radius / 2, wheel_radius / wheel_separation, -wheel_radius / wheel_separation;

    RCLCPP_INFO_STREAM(get_logger(), "Conversion Matrix == \n"
                                         << speed_conversion);
}


void SimpleController::vel_callback(const geometry_msgs::msg::TwistStamped &msg)
{
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);

    Eigen::Vector2d wheel_speed= speed_conversion.inverse()*robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1)); // left wheel of robot;sending rpmto motor
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));  //right wheel of robot;sending rpmto motor

    wheel_cmd_pub->publish(wheel_speed_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                       // initialize ROS 2 communication
    auto node = std::make_shared<SimpleController>("simple_controller"); // creating a shared ponter to the node
    rclcpp::spin(node);                             // loop as node is executed
    rclcpp::shutdown();                             // shutdown the communication, stop spinning
    return 0;
}