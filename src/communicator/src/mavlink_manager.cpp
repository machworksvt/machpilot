#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/connection_result.h>
#include "rclcpp/rclcpp.hpp"

using namespace mavsdk;

/* How this node should function
    this node is meant to sit on the icarus vehicle and do a few things:

    1 - Telemetry down
        monitor various telmetry/data topics
        monitor /down for specific messages going down
        take all relevant data/messages, translate to mavlink, and broadcast down

    2 - Telemetry up
        monitor for mavlink messages received from the ground
        form of commands to do things ex. start engine, deploy parahcute, abort, flip to manual
        hear heartbeats and acknowledge
        handle the messages from the ground and do the corresponding thing (service/action, etc.)

    3 - Monitor the link
        how is the bandwidth? latency? -> communicate this to the ground
        change from high bandwidth to low bandwidth modes (this could be based on distance to GS, or measured signal strength)
        


*/

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