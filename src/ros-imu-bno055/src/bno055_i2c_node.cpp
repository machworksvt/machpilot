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
    this->declare_parameter<int>("address", BNO055_ADDRESS_A);
}
