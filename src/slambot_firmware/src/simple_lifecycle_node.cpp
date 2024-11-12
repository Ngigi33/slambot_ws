#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <memory>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    void msg_callback(const std_msgs::msg::String &msg)
    {
        auto state = get_current_state();
        if (state.label() == "active")
        {
            RCLCPP_INFO_STREAM(get_logger(), "LIfecycle node heard:" << msg.data.c_str());
        }
    }

public:
    explicit SimpleLifecycleNode(const std::string &node_name, bool intra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
    {
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleLifecycleNode::msg_callback, this, _1));
        RCLCPP_INFO(get_logger(), "LIfecyscle Node on_configure() Called ********");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        sub_.reset();
        RCLCPP_INFO(get_logger(), "LIfecyscle Node on_shutdown() Called ********");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        sub_.reset();
        RCLCPP_INFO(get_logger(), "LIfecyscle Node on_cleanup() Called ********");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_activate(state);
        std::this_thread::sleep_for(2s);
        RCLCPP_INFO(get_logger(), "LIfecyscle Node on_activate() Called ********");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state)
    {
        LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(get_logger(), "LIfecyscle Node on_configure() Called ********");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initialize ROS 2 communication
    rclcpp::executors::SingleThreadedExecutor ste;
    std::shared_ptr<SimpleLifecycleNode> simple_lifecycle_node = std::make_shared<SimpleLifecycleNode>("simple_lifecycle_node");
    // auto node = std::make_shared<simple_lifecycle_node>(); // creating a shared ponter to the node
    ste.add_node(simple_lifecycle_node->get_node_base_interface());
    ste.spin();         // loop as node is executed
    rclcpp::shutdown(); // shutdown the communication, stop spinning
    return 0;
}
