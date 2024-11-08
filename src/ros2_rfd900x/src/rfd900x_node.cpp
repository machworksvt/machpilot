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
    void send_battery_info();

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
        timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(rfd900x_node::send_battery_info, this));
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


/**
 * @brief Send BATTERY_STATUS (147) message over RFD900x. 
 */
void rfd900x_node::send_battery_info() {
    uint8_t system_id = 1;                  // ID of the sending system
    uint8_t component_id = 0;               // ID of the sending component
    mavlink_message_t *msg;                 // MAVLink message to compress the data into
    uint8_t id = 0;                         // Battery ID
    uint8_t battery_function = 0;           // Function of the battery (https://mavlink.io/en/messages/common.html#MAV_BATTERY_FUNCTION)
    uint8_t type = 3;                       // Type (chemistry) of the battery (https://mavlink.io/en/messages/common.html#MAV_BATTERY_TYPE)
    int16_t temperature = INT16_MAX;        // Temperature of the battery (cdegC). INT16_MAX for unknown temperature.
    const uint16_t voltages[10] = {};       // Battery voltage of cells 1 to 10 (mV)
    int16_t current_battery = -1;           // Battery current (cA)
    int32_t current_consumed = -1;          // Consumed charge (mAh)
    int32_t energy_consumed = -1;           // Consumed energy (hJ)
    int8_t battery_remaining = 100;         // Remainign battery energy (%)
    int32_t time_remaining = 0;             // Remaining battery time (s)
    uint8_t charge_state = 0;               // State for extent of discharge (https://mavlink.io/en/messages/common.html#MAV_BATTERY_CHARGE_STATE)
    const uint16_t voltages_ext[4] = {};    // Battery voltages for cells 11 to 14
    uint8_t mode = 0;                       // Battery mode (https://mavlink.io/en/messages/common.html#MAV_BATTERY_MODE)
    uint32_t fault_bitmask = 0;             // Fault/health indicators (https://mavlink.io/en/messages/common.html#MAV_BATTERY_FAULT)
    mavlink_msg_battery_status_pack(system_id, component_id, msg,
    id, battery_function, type, temperature, voltages, current_battery, 
    current_consumed, energy_consumed, battery_remaining, time_remaining, 
    charge_state, voltages_ext, mode, fault_bitmask);

    mavlink_passthrough_->send_message(*msg);   // TODO: send_message is DEPRECATED, replace with queue_message
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rfd900x_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}