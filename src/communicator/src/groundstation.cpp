#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/connection_result.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param_server/param_server.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry_server/telemetry_server.h>
#include <mavsdk/plugins/action_server/action_server.h>
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.h>
#include <mavsdk/plugins/mission/mission.h>
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using namespace mavsdk;
using namespace rclcpp;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class GroundStationNode : public rclcpp::Node
{
  public:
    GroundStationNode()
    : Node("ground_station")
    {
      this->declare_parameter("connection_path", "serial:///dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00HIYX-if00-port0:57600");

      // Setup mavlink connection
      ConnectionResult connection_result = mavsdk_.add_any_connection(this->get_parameter("connection_path").as_string());
      if (connection_result != ConnectionResult::Success) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Connection succeeded. Waiting for ICARUS detection.");
      // subscribe to system found event
      mavsdk_.subscribe_on_new_system(std::bind(&GroundStationNode::system_found_callback, this));
    }

    public:
      ~GroundStationNode() noexcept override = default;

    private:
      void system_found_callback() {
        RCLCPP_INFO(this->get_logger(), "System detected. Verifying it is an autopilot.");
        link_setup();
      }

    private:
      void link_setup() {
        //setup link to vehicle
      }

    Mavsdk mavsdk_{Mavsdk::Configuration{ComponentType::GroundStation}};
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundStationNode>());
  rclcpp::shutdown();
  return 0;
}