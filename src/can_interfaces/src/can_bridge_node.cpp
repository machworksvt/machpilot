/*
    This node is designed to abtract the can interface of the jetson nano. It is designed to be used in the MachPilot project.

    It simply forms a thread-safe interface to the can bus, and publishes the data to ROS2 topics.
    It publishes the following topics:
    - /can_rx (interfaces::msg::CanMsg) - The raw can message received from the bus

    And has the following serivces:
    - /can_tx (interfaces::srv::SendCanMessage) - The raw can message to send to the bus

*/
#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/can_msg.hpp"
#include "interfaces/srv/send_can_message.hpp"

// Linux SocketCAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

class CANHandler : public rclcpp::Node {
    public:
      CANHandler() : Node("can_bridge_node")
      {
        this->declare_parameter("can_interface", "can0");
        interface_name_ = this->get_parameter("can_interface").as_string();

        RCLCPP_INFO(this->get_logger(), "Opening CAN Socket on interface: %s", interface_name_.c_str());
        //Open the CAN socket
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open CAN socket");
            exit(1);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Opened CAN Socket on interface: %s", interface_name_.c_str());
        //set non-blocking mode
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
    
        //get interface index
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ);
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
          RCLCPP_FATAL(this->get_logger(), "Error in ioctl when getting interface index for %s", interface_name_.c_str());
          close(socket_fd_);
          exit(1);
          return;
        }
    
        //bind the socket to the interface
        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
          RCLCPP_FATAL(this->get_logger(), "Error binding socket to %s", interface_name_.c_str());
          close(socket_fd_);
          exit(1);
          return;
        }

        can_rx_pub_ = this->create_publisher<interfaces::msg::CanMsg>("/can_rx", 10); //publish the received can messages
        can_tx_srv_ = this->create_service<interfaces::srv::SendCanMessage>("/can_tx", std::bind(&CANHandler::handle_can_tx, this, _1, _2)); //service to send can messages


        start([this](const struct can_frame & frame) {
          handle_can_rx(frame);
        });
      }
    
      ~CANHandler() {
        if (socket_fd_ >= 0) {
          close(socket_fd_);
        }
      }
    
          //This function starts a dedicated thread to read from the CAN socket
    void start(std::function<void(struct can_frame)> frame_callback) {
        running_ = true;
        read_thread_ = std::thread([this, frame_callback]() {
            struct can_frame frame;
            while(running_) {
            //lock for thread-safe socket access
            {
                std::lock_guard<std::mutex> lock(mutex_);
                ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
                if (nbytes > 0) {
                frame_callback(frame);
                }
            }
            std::this_thread::sleep_for(10ms);
            }
        });
    }

    void stop() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }

    ssize_t send_frame(const struct can_frame & frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        return write(socket_fd_, &frame, sizeof(struct can_frame));
    }

    void handle_can_rx(const struct can_frame & frame) {
        interfaces::msg::CanMsg msg = interfaces::msg::CanMsg();
        msg.id = frame.can_id;
        msg.dlc = frame.can_dlc;
        msg.data = {frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]};
        can_rx_pub_->publish(msg);
    } 

    void handle_can_tx(const std::shared_ptr<interfaces::srv::SendCanMessage::Request> request,
        std::shared_ptr<interfaces::srv::SendCanMessage::Response> response) {
        struct can_frame frame;
        frame.can_id = request->msg.id;
        frame.can_dlc = request->msg.dlc;
        std::memcpy(frame.data, request->msg.data.data(), request->msg.data.size());
        ssize_t nbytes = send_frame(frame);
        response->success = nbytes > 0;
    }
    
    private:
        std::string interface_name_;
        int socket_fd_{-1};
        std::thread read_thread_;
        std::mutex mutex_;
        std::atomic<bool> running_;
        rclcpp::Publisher<interfaces::msg::CanMsg>::SharedPtr can_rx_pub_;
        rclcpp::Service<interfaces::srv::SendCanMessage>::SharedPtr can_tx_srv_;
    };

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CANHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    