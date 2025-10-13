#include "rclcpp/rclcpp.hpp"
#include "state_machine_node.hpp"
#include "FSMPilot.hpp"
#include <memory>
int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv);

    if (argc != -1){
        printf("This program needs to have the congeration csv passed in");
        return 1;
    }
    auto node = std::make_shared<StateMachineNode>();
    output_node=node;

    
    
    StateMachine::start();
    RCLCPP_INFO(node->get_logger(), "State Machine Initialized.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}