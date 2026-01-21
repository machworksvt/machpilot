#include "ms4525do_driver.h"
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_interface.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>


#include <cstdio>

class MS4525DONode : public Device {
public:
  MS4525DONode(int addr) : Device("ms4525do_node") {
    _ms4525do = std::make_shared<MS4525DO>(I2C_FILE_PATH, 0, addr);

    _temperature_publisher = this->create_publisher<sensor_msgs::msg::Temperature>("pitot_temp", 10);
    _pressure_publisher = this->create_publisher<sensor_msgs::msg::FluidPressure>("pitot_press", 10);
  
    _timer = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() -> void
      {
        float temperature_c = 0.0f;
        float pressure_pa   = 0.0f;
        uint8_t status;

        _ms4525do->readPressureAndTempHD();

        temperature_c = _ms4525do->_data.temp;
        pressure_pa = _ms4525do->_data.pressure * PSI2PA; // Convert psi to pascal
        status = _ms4525do->_data.status;

        if (status != 0)
        {
          RCLCPP_WARN(this->get_logger(), "Failed to read MS4525 data, error code: %d", status);
          return;
        }

        // Log the results
        RCLCPP_INFO(this->get_logger(),
                    "MS4525DO: Temperature = %.2f °C, Pressure = %.5f pa",
                    temperature_c, pressure_pa);

        // Publish Temperature message
        auto temp_msg = sensor_msgs::msg::Temperature();
        temp_msg.header.stamp = this->now();
        temp_msg.header.frame_id = "1";
        temp_msg.temperature = temperature_c;
        temp_msg.variance = 0.0;  // Unknown variance; adjust if available
        _temperature_publisher->publish(temp_msg);

        // Publish Fluid Pressure message
        auto press_msg = sensor_msgs::msg::FluidPressure();
        press_msg.header.stamp = this->now();
        press_msg.header.frame_id = "ms4525do_frame";
        press_msg.fluid_pressure = pressure_pa;
        press_msg.variance = 0.0;  // Unknown variance; adjust if available
        _pressure_publisher->publish(press_msg);
      }
    );
  }

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      return CallbackReturn::SUCCESS;
  }

  std::shared_ptr<MS4525DO> _ms4525do;
  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr _temperature_publisher;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr _pressure_publisher;
};

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exe;

  auto node = std::make_shared<MS4525DONode>(0x28);

  exe.add_node(node->get_node_base_interface());
  exe.spin();

  rclcpp::shutdown();
  return 0;
}
