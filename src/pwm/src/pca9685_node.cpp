#include <memory>
#include <iostream>
#include "controller.hpp"

using std::placeholders::_1;

extern "C" {
  #include "pca9685_driver.h"
}

class PCA9685Node : public rclcpp::Node
{
public:
PCA9685Node(int addr) : Node("pca9685_node")
{
    pca9685_ = std::make_shared<PCA9685>(addr);
    // Create publishers for temperature and fluid pressure
    pwm_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "servo_angles", 10, std::bind(&PCA9685Node::topic_callback, this, _1));
    std::cout << "pwm node initialized" << std::endl;
}

~PCA9685Node()
{

}

private:
void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const 
{
    for (uint8_t i = 0; i < msg->data.size(); i++) {
        pca9685_->angles_[i] = msg->data.at(i);
    }

    pca9685_->pwm_set_all();
}

std::shared_ptr<PCA9685> pca9685_;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCA9685Node>(0x40);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
