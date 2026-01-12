

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "logger_message_interface/msg/log.hpp"    
#include "log_class/message.hpp"

class LoggerPublisher {
private:
    rclcpp::Publisher<logger_message_interface::msg::Log>::SharedPtr publisher;
public:
    LoggerPublisher(rclcpp::Node* node, int buffer_size);
    void publish(const LogInner& inner, Severity severity);
};
