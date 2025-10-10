#include <memory>
#include <iostream>
#include "lifecycle_interface.hpp"

using std::placeholders::_1;

#include "pca9685_driver.h"

class PCA9685Node : public Device
{
public:
PCA9685Node(int addr) : Device("pca9685_node")
{
    _pca = std::make_shared<PCA9685>(I2C_FILE_PATH, 0, addr);
    RCLCPP_INFO(get_logger(), "PCA9685: reset successful");

    _timer = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&PCA9685Node::timer_callback, this));
}

~PCA9685Node()
{
    
}

private:
CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    
    if (_pca->reset() != 0) {
        RCLCPP_ERROR(get_logger(), "PCA9685: reset failed");
        return CallbackReturn::FAILURE;
    }

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

CallbackReturn on_error(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

int timer_callback() {
    
}

private:
std::shared_ptr<PCA9685> _pca;
rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    auto node = std::make_shared<PCA9685Node>(0x40);

    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}
