#include <rclcpp/rclcpp.hpp>


// Adjust these includes to your actual file names in the BMP390 driver
// For example, if you have "driver_bmp390_basic.h" in "driver/" or "include/".
extern "C" {
  #include "driver_bmp390_basic.h"
}

class BMP390Node : public rclcpp::Node
{
public:
  BMP390Node() : Node("bmp390_node")
  {
    bmp390_interface_t interface = BMP390_INTERFACE_IIC;
    bmp390_address_t addr = BMP390_ADDRESS_ADO_HIGH;
    // attempt init
    uint8_t status = bmp390_basic_init(interface, addr);
    if (status != 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get BMP390, error code: %d", status);
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "BMP390 located");
    }

    // 2) Create a timer to periodically read data
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), // read every 1 second
      [this]() -> void
      {
        float temperature_c = 0.0f;
        float pressure_pa   = 0.0f;
        int status;

        // read data
        status = bmp390_basic_read(&temperature_c, &pressure_pa);
        if (status != 0)
        {
          RCLCPP_WARN(this->get_logger(), "Failed to read BMP390 data, error code: %d", status);
          return;
        }

        // log the results
        RCLCPP_INFO(this->get_logger(),
                    "BMP390: Temperature = %.2f °C, Pressure = %.2f Pa",
                    temperature_c, pressure_pa);
      }
    );

    // 3) Optionally store that the sensor was init’d successfully
    initialized_ = true;
  }

  ~BMP390Node()
  {
    if (initialized_)
    {
      // 4) De-initialize on shutdown
      bmp390_basic_deinit();
      RCLCPP_INFO(this->get_logger(), "BMP390 deinitialized.");
    }
  }

private:
  bool initialized_{false};
  rclcpp::TimerBase::SharedPtr timer_;
  bmp390_handle_t gs_handle;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BMP390Node>());
  rclcpp::shutdown();
  return 0;
}
