#include <rclcpp/rclcpp.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <connection_result.h>
#include <memory>

using namespace mavsdk;

class rfd900x_node : public rclcpp::Node {
public:
    rfd900x_node();

private:
    bool connect_rfd900x();

    std::shared_ptr<Mavsdk> mavsdk_;
    std::shared_ptr<System> system_;
    std::shared_ptr<MavlinkPassthrough> mavlink_passthrough_;
    std::shared_ptr<Telemetry> telemetry_;
    rclcpp::TimerBase::SharedPtr timer_;
};


rfd900x_node::rfd900x_node() : Node("rfd900x_node") {
    if(!connect_rfd900x()) {
        // TODO: ERROR! Do Something!
    }
    else {
        //telemetry_ = std::make_shared<Telemetry>(system_);    // not implemented yet

        mavlink_passthrough_ = std::make_shared<MavlinkPassthrough>(system_);
        //timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(rfd900x_node::send_battery_info, this));
    }
}


/**
 * @brief Connects to USB port and instantiates mavlink_ and system_. 
 *
 * @return True if successful, false if unsuccessful.
 *
 * @note Function does not ensure the device is RFD 900x
 */
bool rfd900x_node::connect_rfd900x() {
    // TODO: ensure correct USB port address is used, baud rate should be 57600
    const std::string connection_url = "serial:///dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10O8MJG-if00-port0:57600";
    mavsdk_ = std::make_shared<Mavsdk>();
    ConnectionResult result = this->mavsdk_->add_any_connection(connection_url);
    if (result != ConnectionResult::Success) {
        RCLCPP_INFO(this->get_logger(), "Port Connection FAILED to Add");
        return false;
    }
    else {RCLCPP_INFO(this->get_logger(), "Port Connection Added Successfully");}
    
    // Wait for the system to be discovered (~3 second arbitrary limit)
    for (double t = 0; t < 3;) {
        if (this->mavsdk_->systems().size() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            t += 0.2;
        }
        else {break;}
    }

    if (!(this->mavsdk_->systems().at(0))) {
        RCLCPP_INFO(this->get_logger(), "No System Found");
        return false;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "System Found");
        system_ = this->mavsdk_->systems().at(0);
        return true;
    }
}





int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rfd900x_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}