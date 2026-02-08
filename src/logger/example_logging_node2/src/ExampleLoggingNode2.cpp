#include "log_class/message.hpp"
#include "logger_publisher/logger_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ExampleLoggingNode : public rclcpp::Node {
 public:
  ExampleLoggingNode() : Node("example_loggging_node"), publisher_(this, 64) {
    timer_ = this->create_wall_timer(
        0ms, std::bind(&ExampleLoggingNode::timer_callback, this));
  }

 private:
  void timer_callback() {
    std::string input;
    std::cin >> input;
    Severity log_level;

    if (input == "LOG") {
      log_level = Severity::Log;
    } else if (input == "WARN") {
      log_level = Severity::Warning;
    } else if (input == "ERROR") {
      log_level = Severity::Error;
    } else {
      std::cerr << "invalid log level \"" << input << "\""<<std::endl;
      rclcpp::shutdown();
      exit(1);
    }

    publisher_.publish(Heartbeat{}, log_level);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  LoggerPublisher publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleLoggingNode>());
  rclcpp::shutdown();
  return 0;
}