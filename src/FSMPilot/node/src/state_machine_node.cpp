#include "state_machine_node.hpp"
#include "FSMPilotStates.hpp"
#include "FSMPilot.hpp"
#include "tinyfsm.hpp"
#include <chrono>
#include <string>
using namespace std::chrono_literals;

StateMachineNode::StateMachineNode()
: Node("state_machine_node")
{

    state_publisher = this->create_publisher<std_msgs::msg::UInt64>("fsm/current_state",10);

    // === Subscribers for Events ===
    initialize_subsystem_sub_ = this->create_subscription<std_msgs::msg::UInt64>(
        "fsm/events/initialize_subsystem", rclcpp::QoS(SUBSYSTEM_COUNT).transient_local(),
        [this](const std_msgs::msg::UInt64 & msg) {
            RCLCPP_INFO(this->get_logger(), "Event: InitializeSubsystem (ID: %lu)", msg.data);
            if (msg.data < SUBSYSTEM_COUNT) {
                send_event(InitializeSubsystem(static_cast<SubSystems>(msg.data)));
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid subsystem ID received: %lu", msg.data);
            }
        });

    arm_motor_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/arm_motor", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Event: ArmMotor");
            send_event(ArmMotor{});
        });

    manual_control_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/manual_control", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Event: ManualControl");
            send_event(ManualControl{});
        });

    relinquish_manual_control_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/relinquish_manual_control", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Event: RelinquishManualControl");
            send_event(RelinquishManualControl{});
        });
    
    disarm_event_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/disarm", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Event: Disarm");
            send_event(DisarmEvent{});
        });

    shutdown_event_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/shutdown", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Event: Shutdown");
            send_event(ShutdownEvent{});
        });

    on_fire_event_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/on_fire", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Event: OnFire");
            send_event(OnFireEvent{});
        });
    
    fire_suppressed_event_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "fsm/events/fire_suppressed", 10,
        [this](const std_msgs::msg::Empty & msg) {
            (void)msg;
            RCLCPP_INFO(this->get_logger(), "Event: FireSuppressed");
            send_event(FireSuppressedEvent{});
        });

    
}

void pubish_state(std::shared_ptr<StateMachineNode> node,FSMPilotStates message){
    auto msg = std_msgs::msg::UInt64();

    msg.data = static_cast<unsigned char>(message);

    RCLCPP_INFO(node->get_logger(), "Event: InState %s", std::string(get_state_name(message)).c_str());

    output_node->state_publisher->publish(msg);
};
