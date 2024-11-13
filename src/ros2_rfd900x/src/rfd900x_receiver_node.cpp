#include <rclcpp/rclcpp.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/mavlink_address.h>
#include <connection_result.h>
#include <memory>

using namespace mavsdk;

class rfd900x_receiver_node : public rclcpp::Node {
public:
    rfd900x_receiver_node();

private:
    bool connect_rfd900x();
    void on_battery_status(const mavlink_message_t& msg);

    std::shared_ptr<Mavsdk> mavsdk_;
    std::shared_ptr<System> system_;
    std::shared_ptr<MavlinkPassthrough> mavlink_passthrough_;
    std::shared_ptr<Telemetry> telemetry_;
    rclcpp::TimerBase::SharedPtr timer_;
};


rfd900x_receiver_node::rfd900x_receiver_node() : Node("rfd900x_receiver_node") {
    if(!connect_rfd900x()) {
        // TODO: ERROR! Do Something!
    }
    else {
        //telemetry_ = std::make_shared<Telemetry>(system_);    // not implemented yet

        mavlink_passthrough_ = std::make_shared<MavlinkPassthrough>(system_);
        // Subscribe to MAVLink messages
        mavlink_passthrough_->subscribe_message(
            MAVLINK_MSG_ID_BATTERY_STATUS, 
            [this](const mavlink_message_t& msg) {
                on_battery_status(msg);
            }
        );
    }
}


/**
 * @brief Connects to USB port and instantiates mavlink_ and system_. 
 *
 * @return True if successful, false if unsuccessful.
 *
 * @note Function does not ensure the device is RFD 900x
 */
bool rfd900x_receiver_node::connect_rfd900x() {
    // TODO: ensure correct USB port address is used, baud rate should be 57600
    const std::string connection_url = "/dev/ttyS1:57600";
   
    mavsdk_ = std::make_shared<Mavsdk>(Mavsdk::Configuration{2, 2, false});
    ConnectionResult connection_result = mavsdk_->add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success) {
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


void rfd900x_receiver_node::on_battery_status(const mavlink_message_t& msg) {
    mavlink_battery_status_t battery_status;
    mavlink_msg_battery_status_decode(&msg, &battery_status);

    // Process the battery data (ALL of it)
    RCLCPP_INFO(this->get_logger(),
            "Received Battery Status:\n"
            "Battery ID: %d\n"
            "Battery Function: %d\n"
            "Battery Type: %d\n"
            "Temperature: %d.%02d C\n"
            "Voltage (Cell 1): %d mV\n"
            "Voltage (Cell 2): %d mV\n"
            "Voltage (Cell 3): %d mV\n"
            "Voltage (Cell 4): %d mV\n"
            "Voltage (Cell 5): %d mV\n"
            "Voltage (Cell 6): %d mV\n"
            "Voltage (Cell 7): %d mV\n"
            "Voltage (Cell 8): %d mV\n"
            "Voltage (Cell 9): %d mV\n"
            "Voltage (Cell 10): %d mV\n"
            "Current: %d mA\n"
            "Current Consumed: %d mAh\n"
            "Energy Consumed: %d hJ\n"
            "Battery Remaining: %d%%\n"
            "Time Remaining: %d seconds\n"
            "Charge State: %d\n"
            "Voltage (Cell 11): %d mV\n"
            "Voltage (Cell 12): %d mV\n"
            "Voltage (Cell 13): %d mV\n"
            "Voltage (Cell 14): %d mV\n"
            "Mode: %d\n"
            "Fault Bitmask: %d",
            battery_status.id,
            battery_status.battery_function,
            battery_status.type,
            battery_status.temperature / 100,  // Convert from 100x Celsius to actual Celsius
            battery_status.temperature % 100, // Get the fractional part of the temperature
            battery_status.voltages[0], battery_status.voltages[1], battery_status.voltages[2], battery_status.voltages[3],
            battery_status.voltages[4], battery_status.voltages[5], battery_status.voltages[6], battery_status.voltages[7],
            battery_status.voltages[8], battery_status.voltages[9],
            battery_status.current_battery / 10,  // Convert from cA to mA
            battery_status.current_consumed,
            battery_status.energy_consumed,
            battery_status.battery_remaining,
            battery_status.time_remaining,
            battery_status.charge_state,
            battery_status.voltages_ext[0], battery_status.voltages_ext[1], battery_status.voltages_ext[2], battery_status.voltages_ext[3],
            battery_status.mode,
            battery_status.fault_bitmask);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rfd900x_receiver_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



// ~~~~~~~~~~~~~~~~~~~~~~~~~ NOTES  ~~~~~~~~~~~~~~~~~~~~~~~~~ //
/*

    need to "source install/setup.bash" before building

    colcon build --packages-select ros2_rfd900x

    need to "source ~/.bashrc" before running

    ros2 run ros2_rfd900x rfd900x_node

 */