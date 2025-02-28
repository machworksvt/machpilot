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

using namespace std::chrono_literals;
using namespace mavsdk;
using namespace rclcpp;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DataLinkNode : public rclcpp::Node
{
  public:
    DataLinkNode()
    : Node("datalink")
    {
      //declare parameter for path to the serial device
      this->declare_parameter("connection_path", "serial:///dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10O8MJG-if00-port0:57600"); //defaults to what I'm pretty sure it is

      // Setup mavlink connection
      ConnectionResult connection_result = mavsdk_.add_any_connection(this->get_parameter("connection_path").as_string());
      if (connection_result != ConnectionResult::Success) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Connection succeeded. Waiting for ground station detection.");
      // subscribe to system found event
      mavsdk_.subscribe_on_new_system(std::bind(&DataLinkNode::system_found_callback, this));
    }

  public:
    ~DataLinkNode() noexcept override = default;

  private:
    void system_found_callback() {
      RCLCPP_INFO(this->get_logger(), "System detected. Verifying it is a ground station.");
      link_setup();
    }

  private:
    void link_setup() {
      // setup link to ground station
    }
    
    Mavsdk mavsdk_{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::Autopilot}};
    std::shared_ptr<ServerComponent> server_component_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLinkNode>());
  rclcpp::shutdown();
  return 0;
}