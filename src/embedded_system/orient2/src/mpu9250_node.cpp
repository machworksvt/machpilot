#include <sensor.hpp>

extern "C" {
    #include "driver_mpu9250_basic.h"
}

class MPU9250Node : public rclcpp::Node
{
public:
  MPU9250Node() : Node("mpu9250_node") 
  {

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("mpu_imu", 10);
    mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mpu_mag", 10);
  
    mpu9250_interface_t interface = MPU9250_INTERFACE_IIC;
    mpu9250_address_t addr = MPU9250_ADDRESS_AD0_LOW;
    uint8_t status = mpu9250_basic_init(interface, addr);
    if (status != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU9250, error code: %d", status);
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "MPU9250 located");
    }
  
      // Create a timer to read and publish sensor data every second
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() -> void
      {
        float accel[3];
        float gyro[3];
        float mag[3];

        uint8_t status = mpu9250_basic_read(accel, gyro, mag);
        if (status != 0)
        {
          RCLCPP_WARN(this->get_logger(), "Failed to read MPU9250 data, error code: %d", status);
          return;
        }
  
        // Log the results
        RCLCPP_INFO(this->get_logger(),
                    "MPU: accel = [%.2f, %.2f, %.2f] m/s^2, gyro = [%.2f, %.2f, %.2f] degrees/s, mag = [%.2f, %.2f, %.2f] T",
                    accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
  
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "mpu9250_frame";

        imu_msg.linear_acceleration.x = accel[0];
        imu_msg.linear_acceleration.y = accel[1];
        imu_msg.linear_acceleration.z = accel[2];
        //imu_msg.linear_acceleration_covariance; fill with 3x3 covariance matrix

        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];
        //imu_msg.angular_velocity_covariance; fill with 3x3 covariance matrix

        imu_publisher_->publish(imu_msg);

        auto mag_msg = sensor_msgs::msg::MagneticField();
        mag_msg.header.stamp = this->now();
        mag_msg.header.frame_id = "bmp390_frame";

        mag_msg.magnetic_field.x = mag[0];
        mag_msg.magnetic_field.y = mag[1];
        mag_msg.magnetic_field.z = mag[2];
        //mag_msg.magnetic_field_covariance; fill with 3x3 covariance matrix

        mag_publisher_->publish(mag_msg);
      }
    );
  
    // Mark sensor as successfully initialized
    initialized_ = true;
  }
  
  ~MPU9250Node()
  {
    if (initialized_)
    {
      // Deinitialize the sensor on shutdown
      mpu9250_basic_deinit();
      RCLCPP_INFO(this->get_logger(), "MPU9250 deinitialized.");
    }
  }
  
private:
  bool initialized_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPU9250Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

