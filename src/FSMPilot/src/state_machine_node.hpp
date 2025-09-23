#ifndef STATEMACHINE_NODE_HPP
#define STATEMACHINE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "FSMPilotStates.hpp"

class StateMachineNode : public rclcpp::Node
{
public:
    StateMachineNode();

    // Publishers
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr state_publisher;
private:

    // Subscribers for triggering events
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr initialize_subsystem_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr arm_motor_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr manual_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr relinquish_manual_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr disarm_event_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr shutdown_event_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr on_fire_event_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr fire_suppressed_event_sub_;
};

void pubish_state(std::shared_ptr<StateMachineNode> node,FSMPilotStates message);

#endif