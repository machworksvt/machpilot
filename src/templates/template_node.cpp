#include <device_driver.hpp>
#include <memory>
#include <iostream>

#include <lifecycle_interface.hpp>

using std::placeholders::_1;

class DeviceNode : public Device
{
public:

DeviceNode(int args) : Device("node_name")
{

}

CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

~PCA9685Node()
{

}

private:

rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    auto node = std::make_shared<DeviceNode>(1);

    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
