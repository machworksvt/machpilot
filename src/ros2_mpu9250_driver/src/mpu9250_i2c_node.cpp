#include <sensor.h>
#include "mpu9250/mpu9250sensor.h"

#include <chrono>
#include <memory>

#include "LinuxI2cCommunicator.h"

using namespace std::chrono_literals;

class MPU9250 : public Sensor {
public:
    MPU9250();
    bool initSrvs(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    bool resetSrvs(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void run();
    void stop();
    bool publish();

private:
    bool param_calibrate;
    int param_gyro_range;
    int param_accel_range;
    int param_dlpf_bandwidth;
    double param_gyro_x_offset;
    double param_gyro_y_offset;
    double param_gyro_z_offset;
    double param_accel_x_offset;
    double param_accel_y_offset;
    double param_accel_z_offset;
    int param_frequency;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
    std::unique_ptr<MPU9250Sensor> mpu9250;
    size_t count;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_init;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset;
};

MPU9250::MPU9250() : Sensor("MPU9250") {
    // Create concrete I2C communicator and pass to sensor
    std::unique_ptr<I2cCommunicator> i2cBus = std::make_unique<LinuxI2cCommunicator>();
    mpu9250 = std::make_unique<MPU9250Sensor>(std::move(i2cBus));
    
    // Declare and set parameters, sets to members
    this->declare_parameter<bool>("calibrate", true);
    this->declare_parameter<int>("gyro_range", MPU9250Sensor::GyroRange::GYR_250_DEG_S);
    this->declare_parameter<int>("accel_range", MPU9250Sensor::AccelRange::ACC_2_G);
    this->declare_parameter<int>("dlpf_bandwidth", MPU9250Sensor::DlpfBandwidth::DLPF_260_HZ);
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);
    this->declare_parameter<int>("frequency", 0.0);

    this->get_parameter("calibrate", param_calibrate);
    this->get_parameter("gyro_range", param_gyro_range);
    this->get_parameter("accel_range", param_accel_range);
    this->get_parameter("dlpf_bandwidth", param_dlpf_bandwidth);
    this->get_parameter("gyro_x_offset", param_gyro_x_offset);
    this->get_parameter("gyro_y_offset", param_gyro_y_offset);
    this->get_parameter("gyro_z_offset", param_gyro_z_offset);
    this->get_parameter("accel_x_offset", param_accel_x_offset);
    this->get_parameter("accel_y_offset", param_accel_y_offset);
    this->get_parameter("accel_z_offset", param_accel_z_offset);
    this->get_parameter("frequency", param_frequency);

    // Set parameters in the sensor object
    mpu9250->setGyroscopeRange(
        static_cast<MPU9250Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
    mpu9250->setAccelerometerRange(
        static_cast<MPU9250Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
    mpu9250->setDlpfBandwidth(
        static_cast<MPU9250Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
    mpu9250->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
                                this->get_parameter("gyro_y_offset").as_double(),
                                this->get_parameter("gyro_z_offset").as_double());
    mpu9250->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
                                    this->get_parameter("accel_y_offset").as_double(),
                                    this->get_parameter("accel_z_offset").as_double());

    // Create publishers over applicable topics
    publisher = this->create_publisher<sensor_msgs::msg::Imu>("mpu9250_orient", 10);
    std::chrono::duration<int64_t, std::milli> frequency =
        1000ms / this->get_parameter("gyro_range").as_int();
    timer = this->create_wall_timer(frequency, std::bind(&MPU9250::publish, this));
    
    // Create services
    srv_init = this->create_service<std_srvs::srv::Trigger>("init", std::bind(&MPU9250::initSrvs, this, std::placeholders::_1, std::placeholders::_2));
    srv_reset = this->create_service<std_srvs::srv::Trigger>("reset", std::bind(&MPU9250::initSrvs, this, std::placeholders::_1, std::placeholders::_2));
}


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPU9250>());
  rclcpp::shutdown();
  return 0;
}