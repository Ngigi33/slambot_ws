#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

class simple_serial_transmitter : public rclcpp::Node
{
public:
    simple_serial_transmitter() : Node("sub") // Create a node with name stated
    {
        declare_parameter<std::string>("port","/dev/ttyUSB0");
        //  declare_parameter<std::string>("port","usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0");
        port=get_parameter("port").as_string();
        stm_.Open(port);
        stm_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "serial_transmitter", 10,
            std::bind(&simple_serial_transmitter::sub_callBack, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "WE have subscribed");
    }
    ~simple_serial_transmitter()
    {
        stm_.Close();
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; // calling subscriber
    LibSerial::SerialPort stm_;
    std::string port;

    void sub_callBack(const std_msgs::msg::String &msg)
    {
        stm_.Write(msg.data);
        RCLCPP_INFO_STREAM(get_logger(), "%s" << msg.data);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                                  // initialize ROS 2 communication
    auto node = std::make_shared<simple_serial_transmitter>(); // creating a shared ponter to the node
    rclcpp::spin(node);                                        // loop as node is executed
    rclcpp::shutdown();                                        // shutdown the communication, stop spinning
    return 0;
}
