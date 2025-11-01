#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/can_msg.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// Linux SocketCAN headers
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>


class Here3Node : public rclcpp::Node
{
public:
    Here3Node() : Node("here3_node")
    {

        can_rx_sub_ = this->create_subscription<interfaces::msg::CanMsg>(
            "/can_rx", 10, std::bind(&Here3Node::handle_can_rx, this, _1));

        gps_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("here3_gps", 10);

        


        


    }

private:
  void handle_can_rx(const interfaces::msg::CanMsg::SharedPtr msg) {
    uint32_t can_id = msg->id;
    uint8_t can_dlc = msg->dlc;
    if (can_id == 0) { //0 is placeholder until can id is confirmed
      if(can_dlc < 8) {
        RCLCPP_WARN(this->get_logger(), "Received CAN message with incorrect DLC. Expected %u, got %u", 8, can_dlc);
        return;
      }
      handle_gps_info(*msg);
    }
  }

private:
  void handle_gps_info(interfaces::msg::CanMsg msg){
    auto here3_msg = sensor_msgs::msg::NavSatFix();
    

    
  }



}