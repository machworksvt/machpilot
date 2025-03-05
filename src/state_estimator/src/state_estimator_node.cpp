#include "state_estimator.h"
#include "rc/math/math.h"
#include "rc/time.h"

using std::placeholders::_1;

class STATE_ESTMATORNode : public Controller
{
public:
  STATE_ESTMATORNode() : Controller("state_estimator_node") {

    raw_state_;
    state_estimate_;

    raw_state_.imu_omega_1 = RC_VECTOR_INITIALIZER;
    raw_state_.imu_omega_2 = RC_VECTOR_INITIALIZER;

    rc_vector_alloc(&raw_state_.imu_omega_1, 3);
    rc_vector_alloc(&raw_state_.imu_omega_2, 3);

    bmp_t_subscriber_ = this->create_subscription<sensor_msgs::msg::Temperature>(
    "bmp_temp", 10, std::bind(&STATE_ESTMATORNode::topic_callback<sensor_msgs::msg::Temperature::SharedPtr>, this, _1));
    bmp_p_subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
    "bmp_press", 10, std::bind(&STATE_ESTMATORNode::topic_callback<sensor_msgs::msg::FluidPressure::SharedPtr>, this, _1));
    std::cout << "bmp_sub initialized" << std::endl;

    pitot_t_subscriber_ = this->create_subscription<sensor_msgs::msg::Temperature>(
    "pitot_temp", 10, std::bind(&STATE_ESTMATORNode::topic_callback<sensor_msgs::msg::Temperature::SharedPtr>, this, _1));
    pitot_p_subscriber_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
    "pitot_press", 10, std::bind(&STATE_ESTMATORNode::topic_callback<sensor_msgs::msg::FluidPressure::SharedPtr>, this, _1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&STATE_ESTMATORNode::topic_callback<sensor_msgs::msg::Imu::SharedPtr>, this, _1));

  }

  raw_state_t raw_state_;
  state_estimate_t state_estimate_;

private:
  template<class mtype> void topic_callback(const mtype msg) {}

  void topic_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    bool which = msg->header.frame_id.back(); // the original or redundant sensor
    uint8_t sensor = msg->header.frame_id.front(); // the sensor code itself (make a number for efficiency)

    if (which > 1) {
      printf("undefined redundant sensor code");
      return;
    }
    // codes include all sensors broadcasting a temperature message
    // #TODO include a typedef enum for all sensors, maybe from sensor.h?
    switch(sensor) {
      case 0: // BMP390
      if (which) {
        raw_state_.bmp_time_1 = msg->header.stamp.nanosec;
        raw_state_.bmp_temp_1 = msg->temperature;
      }
      else {
        raw_state_.bmp_time_2 = msg->header.stamp.nanosec;
        raw_state_.bmp_temp_2 = msg->temperature;
      }

      case 1: //MS4525DO
      if (which) {
        raw_state_.pitot_time_1 = msg->header.stamp.nanosec;
        raw_state_.pitot_temp_1 = msg->temperature;
      }
      else {
        raw_state_.pitot_time_2 = msg->header.stamp.nanosec;
        raw_state_.pitot_temp_2 = msg->temperature;
      }

      default:
      printf("Undefined sensor reference");
      return;
    }
    
  }

  void topic_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
    bool which = msg->header.frame_id.back(); // the original or redundant sensor
    uint8_t sensor = msg->header.frame_id.front(); // the sensor code itself (make a number for efficiency)

    if (which > 1) {
      printf("undefined redundant sensor code");
      return;
    }
    // codes include all sensors broadcasting a temperature message
    switch(sensor) {
      case 0: // BMP390
        if (which) {
          raw_state_.bmp_time_1 = msg->header.stamp.nanosec;
          raw_state_.bmp_temp_1 = msg->fluid_pressure;
        }
        else {
          raw_state_.bmp_time_2 = msg->header.stamp.nanosec;
          raw_state_.bmp_temp_2 = msg->fluid_pressure;
        }

      case 1: //MS4525DO
        if (which) {
          raw_state_.pitot_time_1 = msg->header.stamp.nanosec;
          raw_state_.pitot_temp_1 = msg->fluid_pressure;
        }
        else {
          raw_state_.pitot_time_2 = msg->header.stamp.nanosec;
          raw_state_.pitot_temp_1 = msg->fluid_pressure;
        }

      default:
        printf("Undefined sensor reference");
        return;
    }
  }

  void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    bool which = msg->header.frame_id.back(); // the original or redundant sensor
    // only which is needed here because no other sensors broadcast IMU

    if (which) {
      raw_state_.imu_time_1 = msg->header.stamp.nanosec;
      raw_state_.imu_omega_1.d[0] = msg->angular_velocity.x;
      raw_state_.imu_omega_1.d[1] = msg->angular_velocity.y;
      raw_state_.imu_omega_1.d[2] = msg->angular_velocity.z;
    }
    else {
      raw_state_.imu_time_2 = msg->header.stamp.nanosec;
      raw_state_.imu_omega_2.d[0] = msg->angular_velocity.x;
      raw_state_.imu_omega_2.d[1] = msg->angular_velocity.y;
      raw_state_.imu_omega_2.d[2] = msg->angular_velocity.z;
    }
  }
  

  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr bmp_t_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr bmp_p_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr pitot_t_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pitot_p_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;


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
