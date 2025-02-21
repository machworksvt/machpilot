#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>

using std::placeholders::_1;

class DisplayNode : public rclcpp::Node
{
    public:
        DisplayNode(): Node("Display_node")
            {
                subscription = this->create_subscription<sensor_msgs::msg::Temperature>(
                "bmp_temp", 10, std::bind(&DisplayNode::topic_callback, this, _1));
                std::cout << "node bit is running!! :o" << std::endl;
                system("src/ssd1306_linux/./ssd1306_bin -I 128x64");
                system("src/ssd1306_linux/./ssd1306_bin -c");
                system("src/ssd1306_linux/./ssd1306_bin -d 1");

                
            }
    private:
        void topic_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) const
        {
            system("src/ssd1306_linux/./ssd1306_bin -c");
            RCLCPP_INFO(this->get_logger(), "I heard: '%f", msg->temperature);
            std::cout << "private callback is running!! :o" << std::endl;

            float temp = std::round(msg->temperature * 100)/100;
            std::string string_ = " temp: " + std::to_string(temp);
            std::string command = "src/ssd1306_linux/./ssd1306_bin -l \"" + string_ + "\"";
            system(command.c_str());

        }
        rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr subscription;
        //std::cout << "private is running!! :o" << std::endl;
};
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisplayNode>());
    rclcpp::shutdown();
    std::cout << "main is running!! :o" << std::endl;
    return 0;
}