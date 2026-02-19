#include "ms4525do_driver.h"

#include <rclcpp/rclcpp.hpp>
#include <lifecycle_interface.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

#include <memory>
#include <iostream>
#include <cstdio>

using std::placeholders::_1;
class MS4525DONode : public Device {
public:
  MS4525DONode(int addr) : Device("ms4525do_node") {
    ms4525do_ = std::make_unique<MS4525DO>(I2C_FILE_PATH, 7, addr);

  }

private:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      
      if (ms4525do_->readMeasureRequest()) {
        RCLCPP_ERROR(get_logger(), "MS4525DO: read failure");
        return CallbackReturn::FAILURE;
      }
      
      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      
      ms4525do_.reset();
      
      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(DT * 1000)),
        std::bind(&MS4525DONode::timer_callback, this));

      pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(
        "pitot_press", 10);

      temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>(
        "pitot_temp", 10);

      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      
      timer_->cancel();
      timer_.reset();

      pressure_publisher_.reset();

      temperature_publisher_.reset();
      
      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      
      ms4525do_.reset();
      
      return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &state) override
  {
      RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
      return CallbackReturn::SUCCESS;
  }

  int timer_callback() {

    if (timeout_counter_ > (uint)(1.0f / DT)) {
        RCLCPP_WARN(get_logger(), "MS4525DO: no data sent in the last second");
    }

    double temperature_c = 0.0;
    double pressure_pa   = 0.0;
    uint8_t status;

    ms4525do_->readPressureAndTempHD();

    temperature_c = ms4525do_->data_.temp;
    pressure_pa = ms4525do_->data_.pressure * PSI2PA; // Convert psi to pascal
    status = ms4525do_->data_.status;

    if (status != 0) {
      RCLCPP_ERROR(this->get_logger(), "MS4525DO: Failed to read data, error code: %d", status);
      
      timeout_counter_++;

      return -1;
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
    temperature_publisher_->publish(temp_msg);

    // Publish Fluid Pressure message
    auto press_msg = sensor_msgs::msg::FluidPressure();
    press_msg.header.stamp = this->now();
    press_msg.header.frame_id = "ms4525do_frame";
    press_msg.fluid_pressure = pressure_pa;
    press_msg.variance = 0.0;  // Unknown variance; adjust if available
    pressure_publisher_->publish(press_msg);

    timeout_counter_ = 0;
    
    return 0;
  }

  std::unique_ptr<MS4525DO> ms4525do_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_publisher_;

  uint16_t timeout_counter_{0};
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
