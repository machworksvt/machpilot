#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mavsdk/mavsdk.h>
#include "std_msgs/msg/float32.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class GroundStationNode : public rclcpp::Node
{
  public:
    GroundStationNode()
    : Node("ground_station"), count_(0)
    {
      fake_engine_data_publisher_ = this->create_publisher<std_msgs::msg::Float32>("downlink/engine_data", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&GroundStationNode::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = std_msgs::msg::Float32();
      message.data = rand() % 100;
      RCLCPP_INFO(this->get_logger(), "Publishing.");
      fake_engine_data_publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fake_engine_data_publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundStationNode>());
  rclcpp::shutdown();
  return 0;
}