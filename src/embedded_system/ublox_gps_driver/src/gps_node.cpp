#include "ublox_gps_driver/gps_node.hpp"
#include "ublox_gps_driver/ublox_messages.hpp"
#include <chrono>
#include <thread>

namespace ublox {

    GpsNode::GpsNode(const rclcpp::NodeOptions& options)
        :   rclcpp_lifecycle::LifecycleNode("gps_node", options),
            running_(false),
            message_count_(0),
            error_count_(0) {

        this->declare_parameter("device_path", "/dev/cu.usbserial-BG009G79");
        this->declare_parameter("baud_rate", 38400);
        this->declare_parameter("frame_id", "gps");

        RCLCPP_INFO(this->get_logger(), "GPS Node Constructed");
      }

    GpsNode::~GpsNode() {
        if (running_) {
            running_ = false;
            if (read_thread_ && read_thread_->joinable()) {
                read_thread_->join();
            }
        }
    }


    std::vector<uint8_t> GpsNode::buildConfigurePort() {
        std::vector<uint8_t> msg;
        
        msg.push_back(0xB5);  // Sync 1
        msg.push_back(0x62);  // Sync 2
        msg.push_back(0x06);  // Class: CFG
        msg.push_back(0x00);  // ID: PRT
        msg.push_back(20);    // Length low
        msg.push_back(0);     // Length high
        
        msg.push_back(1);     // Port ID (UART)
        msg.push_back(0);
        msg.push_back(0);
        msg.push_back(0);
        
        msg.push_back(0xC0);  // 8N1
        msg.push_back(0x08);
        msg.push_back(0x00);
        msg.push_back(0x00);
        
        // Baud rate
        uint32_t baud = static_cast<uint32_t>(baud_rate_);
        msg.push_back(baud & 0xFF);
        msg.push_back((baud >> 8) & 0xFF);
        msg.push_back((baud >> 16) & 0xFF);
        msg.push_back((baud >> 24) & 0xFF);
        
        // Input: UBX only
        msg.push_back(0x01);
        msg.push_back(0x00);
        
        // Output: UBX only
        msg.push_back(0x01);
        msg.push_back(0x00);
        
        msg.push_back(0x00);
        msg.push_back(0x00);
        msg.push_back(0x00);
        msg.push_back(0x00);
        
        // Calculate checksum
        auto [ck_a, ck_b] = calculateChecksum(msg.data() + 2, msg.size() - 2);
        msg.push_back(ck_a);
        msg.push_back(ck_b);
        
        return msg;
    }

    std::vector<uint8_t> GpsNode::buildEnableNavPVT() {
        std::vector<uint8_t> msg;
        
        msg.push_back(0xB5);  // Sync 1
        msg.push_back(0x62);  // Sync 2
        msg.push_back(0x06);  // Class: CFG
        msg.push_back(0x01);  // ID: MSG
        msg.push_back(3);     // Length
        msg.push_back(0);
        
        msg.push_back(0x01);  // Class: NAV
        msg.push_back(0x07);  // ID: PVT
        msg.push_back(1);     // Rate: every solution
        
        auto [ck_a, ck_b] = calculateChecksum(msg.data() + 2, msg.size() - 2);
        msg.push_back(ck_a);
        msg.push_back(ck_b);
        
        return msg;
    }

    bool GpsNode::sendConfigMessage(const std::vector<uint8_t>& msg) {
        if (!uart_) {
            RCLCPP_ERROR(this->get_logger(), "UART not initalized");
            return false;
        }

        ssize_t written = uart_->write(msg.data(), msg.size());
        if (written !=static_cast<ssize_t>(msg.size())) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write config message");
            return false; 
        }

        // wait for gps to process
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Flush any ACK responses (NOTE THIS MIGHT HAVE TO CHANGE!!!)
        uint8_t flush_buffer[256];
        // change 5 to a more descriptive value for ack
        for (int i = 0; i < 5; i++) {
            uart_->read(flush_buffer, sizeof(flush_buffer));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        return true;
    }

    bool GpsNode::configureGPS() {
        RCLCPP_INFO(this->get_logger(), "Configuring GPS to UBX protocol...");

        // 1. Configure port to UBX protocol
        auto cfg_prt = buildConfigurePort(); 
        if (!sendConfigMessage(cfg_prt)) {
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Port configured for UBX");

        // 2. Enable NAV-PVT messages
        auto cfg_msg = buildEnableNavPVT();
        if (!sendConfigMessage(cfg_msg)) {
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "NAV-PVT messages enabled");

        return true;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    GpsNode::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(this->get_logger(), "Configuring GPS Node");

        // Get params
        device_path_ = this->get_parameter("device_path").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        RCLCPP_INFO(this->get_logger(), "Device %s, Baud %d", 
                    device_path_.c_str(), baud_rate_);

        // Create publishers
        nav_sat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
            "fix", 10
        );

        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "diagnostics", 10
        );

        // Create uart interface (stack allocated)
        uart_.emplace(device_path_);

        if (!uart_->open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GPS device");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        
        if (!uart_->configure(baud_rate_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure UART");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        
        // Configure GPS for UBX protocol and NAV-PVT messages
        if (!configureGPS()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure GPS");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // create parser
        parser_.emplace();

        // set callback
        parser_->setNavPVTCallback([this](const NavPVT& pvt) {
            publishNavSatFix(pvt);
        });

        RCLCPP_INFO(this->get_logger(), "GPS Node configured succsefully!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    GpsNode::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(this->get_logger(), "Activating GPS Node");

        // Activate publishers
        nav_sat_pub_->on_activate();
        diag_pub_->on_activate();

        // Start reading thread
        running_ = true;
        read_thread_ = std::make_unique<std::thread>(&GpsNode::readThread, this);
        
        RCLCPP_INFO(this->get_logger(), "GPS Node activated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    GpsNode::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(this->get_logger(), "Deactivating GPS Node");
        
        // Stop reading thread
        running_ = false;
        if (read_thread_ && read_thread_->joinable()) {
            read_thread_->join();
        }
        
        // Deactivate publishers
        nav_sat_pub_->on_deactivate();
        diag_pub_->on_deactivate();
        
        RCLCPP_INFO(this->get_logger(), "GPS Node deactivated");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    GpsNode::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(this->get_logger(), "Cleaning up GPS Node");
        
        if (uart_) {
            uart_->close();
        }
        
        uart_.reset();
        parser_.reset();
        
        nav_sat_pub_.reset();
        diag_pub_.reset();
        
        RCLCPP_INFO(this->get_logger(), "GPS Node cleaned up");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    GpsNode::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(this->get_logger(), "Shutting down GPS Node");
        
        running_ = false;
        if (read_thread_ && read_thread_->joinable()) {
            read_thread_->join();
        }
        
        if (uart_) {
            uart_->close();
        }
        
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void GpsNode::readThread() {
        RCLCPP_INFO(this->get_logger(), "Read thread started");
        
        uint8_t buffer[256];
        
        while (running_ && rclcpp::ok()) {
            if (!uart_) {
                break;
            }
            
            ssize_t bytes_read = uart_->read(buffer, sizeof(buffer));
            
            if (bytes_read > 0) {
                for (ssize_t i = 0; i < bytes_read; i++) {
                    parser_->processByte(buffer[i]);
                }
            } else if (bytes_read < 0) {
                error_count_++;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "UART read error");
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(this->get_logger(), "Read thread stopped");
    }

    void GpsNode::publishNavSatFix(const NavPVT& pvt) {
        auto msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
        
        msg->header.stamp = this->now();
        msg->header.frame_id = frame_id_;
        
        // Convert from 1e-7 degrees to degrees
        msg->latitude = pvt.lat * 1e-7;
        msg->longitude = pvt.lon * 1e-7;
        msg->altitude = pvt.hMSL * 1e-3;  // mm to m
        
        // Status
        if (pvt.fixType == 3) {
            msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        } else if (pvt.fixType == 2) {
            msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        } else {
            msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }
        
        msg->status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        
        // Covariance (convert mm to m, then square for variance)
        double h_variance = std::pow(pvt.hAcc * 1e-3, 2);
        double v_variance = std::pow(pvt.vAcc * 1e-3, 2);
        
        msg->position_covariance[0] = h_variance;
        msg->position_covariance[4] = h_variance;
        msg->position_covariance[8] = v_variance;
        msg->position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        
        nav_sat_pub_->publish(std::move(msg));
        
        message_count_++;
        last_message_time_ = this->now();
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "GPS: %.6f°, %.6f°, %.1fm, %d sats",
                            pvt.lat * 1e-7, pvt.lon * 1e-7, pvt.hMSL * 1e-3, pvt.numSV);
    }
}