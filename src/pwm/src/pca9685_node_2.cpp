#include <memory>
#include <iostream>
#include <chrono>
#include <math.h>
#include "controller.hpp"

using std::placeholders::_1;

class PCA9685Node2 : public rclcpp::Node
{
public:
PCA9685Node2() : Node("pca9685_node_2")
{
    pwm_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("servo_angles", 10);

    // Create a timer to publish sensor data every second
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        [this]() -> void
        {
            auto pwm_msg = std_msgs::msg::Float32MultiArray();
            
            float angles[16] = {10, 20, 30, 40,
                                50, 60, 70, 80, 
                                90, 100, 110, 120, 
                                130, 140, 150, 160};
            for (int i = 0; i < 16; i++) {
                angles[i] = 90.0 * sin(rand() % 180 + angles[i]) + 90.0;
                pwm_msg.data.insert(pwm_msg.data.begin() + i, angles[i]);
            }

            if (pwm_msg.data.size() == 16) {
                pwm_publisher_->publish(pwm_msg);
                printf("sending servo angles: %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f, %1.1f \n"
                       , angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6], angles[7], 
                       angles[8], angles[9], angles[10], angles[11], angles[12], angles[13], angles[14], angles[15]);
            }
        }
    );
}

~PCA9685Node2()
{
        
}

private:
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pwm_publisher_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCA9685Node2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
