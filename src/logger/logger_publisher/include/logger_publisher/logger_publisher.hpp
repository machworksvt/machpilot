

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "logger_message_interface/msg/log.hpp"    
#include "log_class/message.hpp"

class LoggerPublisher {
private:
    rclcpp::Publisher<logger_message_interface::msg::Log>::SharedPtr publisher;
    Source source;
public:
    /**
     * @brief Creates an empty LoggerPublisher you will not be able to publish.
     */
    LoggerPublisher();


    /**
     * @brief Creates a new LoggerPublisher.
     * @param node The node that this will publish as.
     * @param buffer_size The number of eliments to buffer locally in this node.
     * @param source The place the logs will report comming from.
     */
    LoggerPublisher(rclcpp::Node* node, int buffer_size, Source source);

    /**
     * @brief Pubishes a log.
     * @param inner The data that will be logged.
     * @param severity How severe the log is.
     */
    void publish(const LogInner& inner, Severity severity);
};
