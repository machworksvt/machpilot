#include "log_class/message.hpp"
#include "logger_publisher/logger_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "log_class/severity.hpp"

using namespace std::chrono_literals;

const int NUM_SOURCES=3;
const std::array<Source,NUM_SOURCES> TESTING_SOURCES={{Source::LoggerTester0,Source::LoggerTester1,Source::LoggerTester2}};


class ExampleLoggingNode : public rclcpp::Node {
 public:
  ExampleLoggingNode() : Node("log_tester_node") 
  {
    for (int i=0; i<NUM_SOURCES; i++){
      this->publishers[i]=LoggerPublisher{this,64,TESTING_SOURCES[i]};
    }
    timer = this->create_wall_timer(
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

    std::cin >> input;
    bool found=false;

    for (int i=0; i<NUM_SOURCES; i++){
      if (source_to_string(TESTING_SOURCES[i])==input){
        publishers[i].publish(Heartbeat{}, log_level);
        found=true;
        break;
      }
    }
    
    if (!found){
      std::cerr << "invalid source \"" << input << "\""<<std::endl;
      rclcpp::shutdown();
      exit(1);    
    }
  }

  rclcpp::TimerBase::SharedPtr timer;
  std::array<LoggerPublisher,3> publishers;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleLoggingNode>());
  rclcpp::shutdown();
  return 0;
}