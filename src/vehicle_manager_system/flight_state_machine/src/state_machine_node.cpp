#include "transition_verifier.hpp"
#include "flight_state_machine.hpp"
#include "state_machine_node.hpp"
#include "sub_systems.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <chrono>
#include <string>

using namespace std::chrono_literals;

StateMachineNode::StateMachineNode()
//We are doing somthing a little risky here where we are passing the this pointer around during initialisation. 
//The base class constructor (regardless of the order they are listed in the constructor) will be run first, then initialize state_machine.
//We explicitly cast this to Node& so it is clear that those constructors can't use uninitialized fields.
: Node("flight_state_machine")
, state_machine(TransitionVerifier(static_cast<Node&>(*this)),this->create_publisher<std_msgs::msg::UInt8>("fsm/current_state",10))
{

    // === Subscribers for Events ===
    this->initialize_subsystem_sub = this->create_subscription<std_msgs::msg::UInt8>(
        "fsm/events/initialize_subsystem", rclcpp::QoS(SUBSYSTEM_COUNT).transient_local(),
        [this](const std_msgs::msg::UInt8 & msg) {
            this->state_machine.react(InitializeSubsystem{msg.data});
        });

    this->arm_motor_sub = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/arm_motor", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            this->state_machine.react(ArmMotor{});
        });

    this->manual_control_sub = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/manual_control", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            this->state_machine.react(ManualControl{});
        });

    this->relinquish_manual_control_sub = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/relinquish_manual_control", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            this->state_machine.react(RelinquishManualControl{});
        });
    
    this->disarm_event_sub = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/disarm", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            this->state_machine.react(DisarmEvent{});
        });

    this->shutdown_event_sub = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/shutdown", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            this->state_machine.react(ShutdownEvent{});
        });
    
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMachineNode>());
  rclcpp::shutdown();
  return 0;
}