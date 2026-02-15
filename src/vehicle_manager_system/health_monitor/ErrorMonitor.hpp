#ifndef ERROR_MONITOR_HPP_
#define ERROR_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

class ErrorMonitorNode : public rclcpp::Node
{
public:
    ErrorMonitorNode();

private:
    void error_callback(const std_msgs::msg::String::SharedPtr msg);
    
    std::string read_csv(const std_msgs::msg::String & msg);

    static const std::string sensors[6];    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

#endif