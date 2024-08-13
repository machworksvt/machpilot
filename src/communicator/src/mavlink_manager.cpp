#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <connection_result.h>
#include <rclcpp/rclcpp.hpp>

using namespace mavsdk;

class MavlinkNode : public rclcpp::Node {
public:
    MavlinkNode() : Node("mavlink_node") {
        // Set up MAVSDK

        ConnectionResult result = mavsdk.add_any_connection("serial:///dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10O8MJG-if00-port0:57600");
        bool wasSuccess = result == ConnectionResult::Success;
        RCLCPP_INFO(this->get_logger(), "hit this line %d", wasSuccess);
        
        // Add additional code to connect, subscribe to telemetry, etc.
    }

    ~MavlinkNode() noexcept override = default;
private:
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::GroundStation}};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavlinkNode>());
  rclcpp::shutdown();
  return 0;
}