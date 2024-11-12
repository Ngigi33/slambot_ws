#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>
#include <chrono>

using namespace std::chrono_literals;

class SimpleSerialReceiver : public rclcpp::Node
{
public:
    SimpleSerialReceiver() : Node("SimpleSerialReceiver") // Create a node with name stated
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        //  declare_parameter<std::string>("port","usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0");
        port = get_parameter("port").as_string();
        stm_.Open(port);
        stm_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "serial_receiver", 10);
        timer_ = create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "WE have Published");
    }
    ~SimpleSerialReceiver()
    {
        stm_.Close();
    }

private:
    void timer_callback()
    {
        auto msg = std_msgs::msg::String();
        if (rclcpp::ok() && stm_.IsDataAvailable())
        {
            stm_.ReadLine(msg.data);
            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // calling publisher
    rclcpp::TimerBase::SharedPtr timer_;
    LibSerial::SerialPort stm_;
    std::string port;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                             // initialize ROS 2 communication
    auto node = std::make_shared<SimpleSerialReceiver>(); // creating a shared ponter to the node
    rclcpp::spin(node);                                   // loop as node is executed
    rclcpp::shutdown();                                   // shutdown the communication, stop spinning
    return 0;
}
