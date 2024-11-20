#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomRepublisher : public rclcpp::Node {
public:
    OdomRepublisher() : Node("odom_republisher") {
        // Create the subscription to the original odometry topic
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/slambot_diffdrive_controller/odom", 
            10, 
            std::bind(&OdomRepublisher::odom_callback, this, std::placeholders::_1)
        );

        // Create the publisher for the new odometry topic
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {
        // Publish the message on the new topic
        publisher_->publish(*msg);
    }

    // Subscriber to the original topic
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    // Publisher for the remapped topic
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
