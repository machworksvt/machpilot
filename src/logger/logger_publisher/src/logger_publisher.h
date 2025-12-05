#include "rclcpp/rclcpp.hpp"
#include "logger_message/msg/LoggerMessage.hpp"   
#include "message.hpp"


class LoggerPublisher{
    private:
        rclcpp::Publisher<logger_message::msg::LoggerMessage>::SharedPtr publisher;

    public:
        LoggerPublisher(rclcpp::Node node,int buffer_size,Source source);

}