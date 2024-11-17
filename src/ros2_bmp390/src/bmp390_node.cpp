#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <bmp390/src/driver_bmp390.h>  // Include the BMP390 driver

class BMP390Node : public rclcpp::Node {
public:
    BMP390Node() : Node("bmp390_node") {
        // Initialize BMP390
        if (bmp390_init(&bmp390) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize BMP390 sensor.");
            return;
        }

        // Create publishers for temperature and pressure data
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("bmp390_temp", 10);
        pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("bmp390_press", 10);

        // Timer to periodically read data from the sensor
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&BMP390Node::read_sensor_data, this));
    }

private:
    bmp390_handle_t bmp390;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void read_sensor_data() {
         bmp390_bool_t temperature, pressure;

        // Read temperature and pressure from BMP390
        if (bmp390_get_temperature(&bmp390, &temperature) == 0 &&
            bmp390_get_pressure(&bmp390, &pressure) == 0) {

            // Publish temperature
            auto temp_msg = sensor_msgs::msg::Temperature();
            temp_msg.temperature = temperature;
            temp_msg.header.stamp = this->now();
            temp_pub_->publish(temp_msg);

            // Publish pressure
            auto pressure_msg = sensor_msgs::msg::FluidPressure();
            pressure_msg.fluid_pressure = pressure;
            pressure_msg.header.stamp = this->now();
            pressure_pub_->publish(pressure_msg);

            RCLCPP_INFO(this->get_logger(), "Temperature: %.2f °C, Pressure: %.2f Pa", temperature, pressure);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to read data from BMP390 sensor.");
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BMP390Node>());
    rclcpp::shutdown();
    return 0;
}