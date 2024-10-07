#include <sensor.h>
#include "imu_bno055/bno055_i2c_driver.h"
#include "watchdog/watchdog.h"
#include <csignal>
#include <memory>

class BNO055 : public Sensor {
public:
    BNO055();
private:
    std::unique_ptr<imu_bno055::BNO055I2CDriver> imu;

    std::string param_device;
    int param_address;
    double param_rate;
    std::string param_frame_id;

    diagnostic_msgs::msg::DiagnosticStatus current_status;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_data;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_raw;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_status;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset;

    std::unique_ptr<rclcpp::Rate> rate;

    watchdog::Watchdog watchdog;

    int seq;
};

BNO055::BNO055() : Sensor("BNO055_Node") {
    this->declare_parameter<std::string>("device", "/dev/i2c-1");
    this->declare_parameter<int>("address", BNO055_ADDRESS_A);
    this->declare_parameter<std::string>("frame_id", "imu");
    this->declare_parameter<double>("rate", 100.0);

    this->get_parameter("device", param_device);
    this->get_parameter("address", param_address);
    this->get_parameter("frame_id", param_frame_id);
    this->get_parameter("rate", param_rate);

    imu = std::make_unique<imu_bno055::BNO055I2CDriver>(param_device, param_address);

    pub_data = this->create_publisher<sensor_msgs::msg::Imu>("data", 10);
    pub_raw = this->create_publisher<sensor_msgs::msg::Imu>("raw", 10);
    pub_mag = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
    pub_temp = this->create_publisher<sensor_msgs::msg::Temperature>("temp", 10);
    pub_status = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("status", 10);

    srv_reset = this->create_service<std_srvs::srv::Trigger>("init", std::bind(&BNO055::initSrvs, this, std::placeholders::_1, std::placeholders::_2));

    seq = 0;

    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::msg::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::msg::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::msg::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::msg::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::msg::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::msg::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);

    rate = std::make_unique<rclcpp::Rate>(param_rate);
}
