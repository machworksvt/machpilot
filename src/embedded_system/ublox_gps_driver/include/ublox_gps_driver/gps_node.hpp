#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include "ublox_gps_driver/uart_interface.hpp"
#include "ublox_gps_driver/ublox_parser.hpp"

#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include <optional>
#include <vector>

namespace ublox {

    class GpsNode : public rclcpp_lifecycle::LifecycleNode {
    public: 
        explicit GpsNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
        ~GpsNode() override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& previous_state) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
        on_activate(const rclcpp_lifecycle::State& previous_state) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    
    private: 
        void readThread();
        void publishNavSatFix(const NavPVT& pvt);
        void publishDiagnostics();

        // GPS Configuration
        bool configureGPS();
        bool sendConfigMessage(const std::vector<uint8_t>& msg);
        std::vector<uint8_t> buildConfigurePort();
        std::vector<uint8_t> buildEnableNavPVT();

        // Parameters
        std::string device_path_;
        int baud_rate_;
        std::string frame_id_;

        // GPS components - STACK ALLOCATED with std::optional
        std::optional<UARTInterface> uart_;
        std::optional<UbloxParser> parser_;

        // Publishers
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_pub_;
        rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_pub_;

        // Thread management
        std::unique_ptr<std::thread> read_thread_;
        std::atomic<bool> running_;

        // Statistics
        std::atomic<uint32_t> message_count_;
        std::atomic<uint32_t> error_count_;
        rclcpp::Time last_message_time_;

        // NOTE ABOVE CODE WAS MADE USING LLM (BOILERPLATE CODE)
    };
}