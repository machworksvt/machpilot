#include "controller.hpp"
#include "state_estimator.h"
#include "rc/math/math.h"
#include "rc/time.h"

using std::placeholders::_1;
class STATE_ESTMATORNode : public Controller
{
public:
  STATE_ESTMATORNode() : Controller("state_estimator_node") {

    bmp_t_subscriber_ = this->create_subscription<sensor_msgs::msg::Temperature>(
    "bmp_temp", 10, std::bind(&STATE_ESTMATORNode::topic_callback<sensor_msgs::msg::Temperature::SharedPtr>, this, _1));
    bmp_p_subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
    "bmp_press", 10, std::bind(&STATE_ESTMATORNode::topic_callback<sensor_msgs::msg::FluidPressure::SharedPtr>, this, _1));
    std::cout << "bmp_sub initialized" << std::endl;

    pitot_t_subscriber_ = this->create_subscription<sensor_msgs::msg::Temperature>(
    "pitot_temp", 10, std::bind(&STATE_ESTMATORNode::topic_callback<sensor_msgs::msg::Temperature::SharedPtr>, this, _1));
    pitot_p_subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
    "pitot_press", 10, std::bind(&STATE_ESTMATORNode::topic_callback<sensor_msgs::msg::FluidPressure::SharedPtr>, this, _1));

  }

private:
template <typename mtype> void topic_callback(const mtype msg) const 
{
  switch (mtype)
  {
    case sensor_msgs::msg::Temperature::SharedPtr:
      if (msg->header.frame_id == "bmp_frame") {
        raw_state_.bmp_time_1r = msg->stamp;
        raw_state_.bmp_temp_1r = msg->temperature;
        raw_state_.bmp_var_t_1r = msg->variance;
      }
      else if (msg->header.frame_id == "ms4525do_frame") {
        raw_state_.pitot_time_1r = msg->stamp;
        raw_state_.pitot_temp_1r = msg->temperature;
        raw_state_.pitot_var_t_1r = msg->variance;
      }
      else {
        std::cout << "unrecognized temperature message" << std::endl;
      }
      break;
    case sensor_msgs::msg::FluidPressure::SharedPtr:
      if (msg->header.frame_id == "bmp_frame") {
        raw_state_.bmp_time_1r = msg->stamp;
        raw_state_.bmp_pressure_1r = msg->fluid_pressure;
        raw_state_.bmp_var_p_1r = msg->variance;
      }
      else if (msg->header.frame_id == "ms4525do_frame") {
        raw_state_.pitot_time_1r = msg->stamp;
        raw_state_.pitot_pressure_1r = msg->fluid_pressure;
        raw_state_.pitot_var_p_1r = msg->variance;
      }
      else {
        std::cout << "unrecognized pressure message" << std::endl;
      }
      break;
    default:
      break;
  }
}

rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr bmp_t_subscriber_;
rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr bmp_p_subscriber_;
rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr pitot_t_subscriber_;
rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pitot_p_subscriber_;

raw_state_t raw_state_;
state_estimate_t state_estimate_;
};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<STATE_ESTMATORNode>());
  rclcpp::shutdown();
  return 0;
}