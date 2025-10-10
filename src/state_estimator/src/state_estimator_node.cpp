#include "state_estimator.h"

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

// local magnetic field vector for Blacksburg, attained from NOAA's calculator
#define INCLINATION 64.0972

using std::placeholders::_1;

class STATE_ESTMATORNode : public rclcpp::Node
{
public:
  STATE_ESTMATORNode() : rclcpp::Node("state_estimator_node") {

    // initializing state variables
    raw_state_;
    state_estimate_;

    // initializing magnetic field reference
    double mag_x = cos(INCLINATION * M_PI / 180.0),
           mag_y = 0.0,
           mag_z = sin(INCLINATION * M_PI / 180.0);

    mag_norm_ = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&mag_norm_, 3);

    mag_norm_.d[0] = mag_x;
    mag_norm_.d[1] = mag_y;
    mag_norm_.d[2] = mag_z;
    
    // initializing the Kalman filters
    if (imu_kf_init()) {
      std::cout << "IMU KF not initialized" << std::endl;
      return;
    }

    // initializing raw state
    raw_state_.imu_omega_1 = RC_VECTOR_INITIALIZER;
    raw_state_.imu_omega_2 = RC_VECTOR_INITIALIZER;
    raw_state_.imu_accel_1 = RC_VECTOR_INITIALIZER;
    raw_state_.imu_accel_2 = RC_VECTOR_INITIALIZER;
    raw_state_.imu_quat_1 = RC_VECTOR_INITIALIZER;
    raw_state_.imu_quat_2 = RC_VECTOR_INITIALIZER;

    rc_vector_alloc(&raw_state_.imu_omega_1, 3);
    rc_vector_alloc(&raw_state_.imu_omega_2, 3);
    rc_vector_alloc(&raw_state_.imu_accel_1, 3);
    rc_vector_alloc(&raw_state_.imu_accel_2, 3);
    rc_vector_alloc(&raw_state_.imu_quat_1, 4);
    rc_vector_alloc(&raw_state_.imu_quat_2, 4);

    raw_state_.imu_var_o_1 = RC_MATRIX_INITIALIZER;
    raw_state_.imu_var_o_2 = RC_MATRIX_INITIALIZER;
    raw_state_.imu_var_a_1 = RC_MATRIX_INITIALIZER;
    raw_state_.imu_var_a_2 = RC_MATRIX_INITIALIZER;
    raw_state_.imu_var_t_1 = RC_MATRIX_INITIALIZER;
    raw_state_.imu_var_t_2 = RC_MATRIX_INITIALIZER;

    rc_matrix_alloc(&raw_state_.imu_var_o_1, 3, 3);
    rc_matrix_alloc(&raw_state_.imu_var_o_2, 3, 3);
    rc_matrix_alloc(&raw_state_.imu_var_a_1, 3, 3);
    rc_matrix_alloc(&raw_state_.imu_var_a_2, 3, 3);
    rc_matrix_alloc(&raw_state_.imu_var_t_1, 3, 3);
    rc_matrix_alloc(&raw_state_.imu_var_t_2, 3, 3);

    // initializing estimated state
    state_estimate_.imu_omega_1 = RC_VECTOR_INITIALIZER;
    state_estimate_.imu_omega_2 = RC_VECTOR_INITIALIZER;
    state_estimate_.imu_quat_1 = RC_VECTOR_INITIALIZER;
    state_estimate_.imu_quat_2 = RC_VECTOR_INITIALIZER;

    rc_vector_alloc(&state_estimate_.imu_omega_1, 3);
    rc_vector_alloc(&state_estimate_.imu_omega_2, 3);
    rc_vector_alloc(&state_estimate_.imu_quat_1, 4);
    rc_vector_alloc(&state_estimate_.imu_quat_2, 4);

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

  uint8_t imu_kf_init() {
    rc_matrix_t F = RC_MATRIX_INITIALIZER;
    rc_matrix_t G = RC_MATRIX_INITIALIZER;
    rc_matrix_t H = RC_MATRIX_INITIALIZER;
    rc_matrix_t Q = RC_MATRIX_INITIALIZER;
    rc_matrix_t R = RC_MATRIX_INITIALIZER;
    rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

    imu_kf_ = RC_KALMAN_INITIALIZER;

    uint8_t Nx = 7, Ny = 9, Nu = 1;

    rc_matrix_zeros(&F, Nx, Nx);
    rc_matrix_zeros(&G, Nx, Nu);
    rc_matrix_zeros(&H, Ny, Nx);
    rc_matrix_zeros(&Q, Nx, Nx);
    rc_matrix_zeros(&R, Ny, Ny);
    rc_matrix_zeros(&Pi, Nx, Nx);

    // F, rest are zeros for now
    F.d[0][0] = 1.0;
    F.d[1][1] = 1.0;
    F.d[2][2] = 1.0;
    F.d[3][3] = 1.0;
    F.d[4][5] = 1.0;
    F.d[5][5] = 1.0;
    F.d[6][6] = 1.0;

    // H for gyrometer, rest are zeros for now
    H.d[0][4] = 1.0;
    H.d[1][5] = 1.0;
    H.d[2][6] = 1.0;

    if (rc_kalman_alloc_lin(&imu_kf_, F, G, H, Q, R, Pi) == -1) {
      return -1;
    }
    rc_matrix_free(&F);
    rc_matrix_free(&G);
    rc_matrix_free(&H);
    rc_matrix_free(&Q);
    rc_matrix_free(&R);
    rc_matrix_free(&Pi);

    return 0;
  }

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
    bool r = msg->header.frame_id.back(); // the original or redundant sensor
    // only r is needed here because no other sensors broadcast on IMU

    // initializing a pointer to the state data, using r
    rc_vector_t* vdp[3];
    rc_matrix_t* mdp[3];

    if (r) {
      // normal sensor
      raw_state_.imu_time_1 = msg->header.stamp.nanosec;

      vdp[0] = &(raw_state_.imu_omega_1);
      vdp[1] = &(raw_state_.imu_accel_1);
      vdp[2] = &(raw_state_.imu_quat_1);

      mdp[0] = &(raw_state_.imu_var_o_1);
      mdp[1] = &(raw_state_.imu_var_a_1);
      mdp[2] = &(raw_state_.imu_var_t_1);
    }
    else {
      // redundant sensor
      raw_state_.imu_time_2 = msg->header.stamp.nanosec;

      vdp[0] = &(raw_state_.imu_omega_2);
      vdp[1] = &(raw_state_.imu_accel_2);
      vdp[2] = &(raw_state_.imu_quat_2);

      mdp[0] = &(raw_state_.imu_var_o_2);
      mdp[1] = &(raw_state_.imu_var_a_2);
      mdp[2] = &(raw_state_.imu_var_t_2);
    }

    if (!vdp[0]->initialized) {
      printf("Uninitialized data vector");
      return;
    }
    if (!mdp[0]->initialized) {
      printf("Uninitialized data matrix");
      return;
    }

    // adding data to each value pointed to by vdp
    vdp[0]->d[0] = msg->angular_velocity.x;
    vdp[0]->d[1] = msg->angular_velocity.y;
    vdp[0]->d[2] = msg->angular_velocity.z;

    vdp[1]->d[0] = msg->linear_acceleration.x;
    vdp[1]->d[1] = msg->linear_acceleration.y;
    vdp[1]->d[2] = msg->linear_acceleration.z;

    // orientation may not be included, depending on the sensor used
    vdp[2]->d[0] = msg->orientation.w;
    vdp[2]->d[1] = msg->orientation.x;
    vdp[2]->d[2] = msg->orientation.y;
    vdp[2]->d[3] = msg->orientation.z;

    // adding data to each value pointed to by mdp, covariance matrices
    for (int8_t i = 0; i < 8; i++) {
      uint8_t col = i % 3;
      uint8_t row = i / 3;
      mdp[0]->d[row][col] = msg->angular_velocity_covariance.at(i);
      mdp[1]->d[row][col] = msg->linear_acceleration_covariance.at(i);
      // #TODO: figure out why this covariance is 3x3 and not 4x4
      mdp[2]->d[row][col] = msg->orientation_covariance.at(i);
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr bmp_t_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr bmp_p_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr pitot_t_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pitot_p_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  raw_state_t raw_state_;
  state_estimate_t state_estimate_;
  rc_kalman_t imu_kf_;
  rc_vector_t mag_norm_;
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
