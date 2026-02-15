#include <memory>
#include <iostream>
#include "pca9685_driver.h"

#include "lifecycle_interface.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>


using std::placeholders::_1;



class PCA9685Node : public Device
{
public:
PCA9685Node(int addr) : Device("pca9685_node")
{
    pca_ = std::make_unique<PCA9685>(I2C_FILE_PATH, 7, addr);

}

~PCA9685Node()
{
    
}

private:
CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    if (pca_ == nullptr) {
        RCLCPP_ERROR(get_logger(), "PCA9685: not initialized");
        return CallbackReturn::FAILURE;
    }
    
    if (pca_->reset() != 0) {
        RCLCPP_ERROR(get_logger(), "PCA9685: reset failed");
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    pca_.reset();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    // Verify that the PCA9685 is connected by reading the MODE1 register (should be 0xA0 after reset)
    uint8_t mode1[1];

    if (i2c_read(&(pca_->i2c_info_), 0x00, 1, mode1)) {
        RCLCPP_ERROR(get_logger(), "PCA9685: not found at address 0x%02X", pca_->i2c_info_.address);
        return CallbackReturn::FAILURE;
    }

    if (mode1[0] != 0x11) {
        RCLCPP_ERROR(get_logger(), "PCA9685: unexpected MODE1 register value 0x%02X", (uint)mode1[0]);
        return CallbackReturn::FAILURE;
    }

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&PCA9685Node::timer_callback, this));

    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "pwm_angles", 10, std::bind(&PCA9685Node::set_angles_callback, this, _1));

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    timer_->cancel();
    timer_.reset();

    sub_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    pca_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn on_error(const rclcpp_lifecycle::State &state) override
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    return CallbackReturn::SUCCESS;
}

int timer_callback() {

    if (timeout_flag_) {
        RCLCPP_WARN(get_logger(), "PCA9685: no message received in the last second");
        return -1;
    }

    timeout_flag_ = true;
    
    return 0;
    
}

int set_angles_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

    timeout_flag_= false;

    if (msg->data.size() > 16) {
        RCLCPP_ERROR(get_logger(), "PCA9685: too many angles, max is 16");
        return -1;
    }

    for (size_t i = 0; i < msg->data.size(); i++) {
        if (pca_->set_angle(msg->data[i], i) != 0) {
            RCLCPP_ERROR(get_logger(), "PCA9685: failed to set angle for channel %ld", i);
            return -1;
        }
    }

    return 0;
}

private:
std::unique_ptr<PCA9685> pca_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

bool timeout_flag_{true};

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
