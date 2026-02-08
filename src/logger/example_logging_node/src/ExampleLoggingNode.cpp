#include "log_class/message.hpp"
#include "logger_publisher/logger_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ExampleLoggingNode : public rclcpp::Node {
 public:
  ExampleLoggingNode()
      : Node("example_loggging_node"),
        /*publisher contructor takes in a pointer to the node and the
// size of its buffer*/
        publisher_(this, 16) {
    // call timer_callback every half a second
    timer_ = this->create_wall_timer(
        500ms, std::bind(&ExampleLoggingNode::timer_callback, this));
  }

 private:
  void timer_callback() {
    // publish function takes in the sublog and the severity of the log
    publisher_.publish(Heartbeat(), Severity::Log);
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