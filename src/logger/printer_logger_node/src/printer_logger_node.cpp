#include <cstring>
#include <iostream>
#include <memory>

#include "log_class/message.hpp"
#include "logger_message_interface/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class PrinterLoggerNode : public rclcpp::Node {
 public:
  PrinterLoggerNode() : Node("PrinterLoggerNode") {
    subscription_ =
        this->create_subscription<logger_message_interface::msg::Log>(
            "log_topic", 10,
            std::bind(&PrinterLoggerNode::topic_callback, this, _1));
  }

 private:
  void topic_callback(const logger_message_interface::msg::Log& msg) const {
    Log log;
    std::memcpy(&log, msg.data.data(), sizeof(Log));
    std::cout << log << std::endl;
  }

  rclcpp::Subscription<logger_message_interface::msg::Log>::SharedPtr
      subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::cout << Log(Severity::Log, LoggerStartup()) << std::endl;
  rclcpp::spin(std::make_shared<PrinterLoggerNode>());
  rclcpp::shutdown();
  return 0;
}