#ifndef STATEMACHINE_NODE_HPP
#define STATEMACHINE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "flight_state_machine.hpp"

class StateMachineNode : public rclcpp::Node
{
public:
    StateMachineNode();

private:

    FlightStateMachine state_machine;

    // Subscribers for triggering events
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr initialize_subsystem_sub;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr arm_motor_sub;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr manual_control_sub;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr relinquish_manual_control_sub;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr disarm_event_sub;
    rclcpp::Subscription<std_msgs::msg::Empty >::SharedPtr shutdown_event_sub;
};

#endif