#include "rclcpp/rclcpp.hpp"
#include "state_machine_node.hpp"
#include "FSMPilot.hpp"
#include <memory>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StateMachineNode>();
    output_node=node;
    
    StateMachine::start();
    RCLCPP_INFO(node->get_logger(), "State Machine Initialized.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}