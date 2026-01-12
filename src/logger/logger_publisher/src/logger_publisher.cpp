#ifndef LOGGER_PUBLISHER_HPP
#define LOGGER_PUBLISHER_HPP

#include "logger_publisher/logger_publisher.hpp"

#include <chrono>
#include <cstring>

#include "log_class/message.hpp"

LoggerPublisher::LoggerPublisher(rclcpp::Node* node, int buffer_size) {
  // Initialize publisher with topic "log_topic"
  publisher = node->create_publisher<logger_message_interface::msg::Log>(
      "log_topic", buffer_size);
}

void LoggerPublisher::publish(const LogInner& inner, Severity severity) {
  Log log_object(severity, inner);

  logger_message_interface::msg::Log msg;

  // Copy memory
  std::memcpy(msg.data.data(), &log_object, sizeof(Log));

  publisher->publish(msg);
}

#endif  // LOGGER_PUBLISHER_HPP