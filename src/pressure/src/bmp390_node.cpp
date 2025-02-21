#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>

// Adjust these includes to your actual file names in the BMP390 driver
extern "C" {
  #include "driver_bmp390_basic.h"
}

class BMP390Node : public rclcpp::Node
{
public:
  BMP390Node() : Node("bmp390_node")
  {
    // Create publishers for temperature and fluid pressure
    temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("bmp_temp", 10);
    pressure_publisher_    = this->create_publisher<sensor_msgs::msg::FluidPressure>("bmp_press", 10);

    // Initialize the sensor
    bmp390_interface_t interface = BMP390_INTERFACE_IIC;
    bmp390_address_t addr = BMP390_ADDRESS_ADO_LOW;
    uint8_t status = bmp390_basic_init(interface, addr);
    if (status != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize BMP390, error code: %d", status);
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "BMP390 located");
    }

    // Create a timer to read and publish sensor data every second
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() -> void
      {
        float temperature_c = 0.0f;
        float pressure_pa   = 0.0f;
        int status = bmp390_basic_read(&temperature_c, &pressure_pa);
        if (status != 0)
        {
          RCLCPP_WARN(this->get_logger(), "Failed to read BMP390 data, error code: %d", status);
          return;
        }

        // Log the results
        RCLCPP_INFO(this->get_logger(),
                    "BMP390: Temperature = %.2f °C, Pressure = %.2f Pa",
                    temperature_c, pressure_pa);

        // Publish Temperature message
        auto temp_msg = sensor_msgs::msg::Temperature();
        temp_msg.header.stamp = this->now();
        temp_msg.header.frame_id = "bmp390_frame";
        temp_msg.temperature = temperature_c;
        temp_msg.variance = 0.0;  // Unknown variance; adjust if available
        temperature_publisher_->publish(temp_msg);

        // Publish Fluid Pressure message
        auto press_msg = sensor_msgs::msg::FluidPressure();
        press_msg.header.stamp = this->now();
        press_msg.header.frame_id = "bmp390_frame";
        press_msg.fluid_pressure = pressure_pa;
        press_msg.variance = 0.0;  // Unknown variance; adjust if available
        pressure_publisher_->publish(press_msg);
      }
    );

    // Mark sensor as successfully initialized
    initialized_ = true;
  }

  ~BMP390Node()
  {
    if (initialized_)
    {
      // Deinitialize the sensor on shutdown
      bmp390_basic_deinit();
      RCLCPP_INFO(this->get_logger(), "BMP390 deinitialized.");
    }
  }

private:
  bool initialized_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BMP390Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
