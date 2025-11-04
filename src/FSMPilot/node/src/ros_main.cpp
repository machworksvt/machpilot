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

    node->declare_parameter<std::string>("config_file", "");

    std::string config_file;
    node->get_parameter<std::string>("config_file", config_file);

    if (config_file.empty()){
        std::cerr << "no config file provided" << std::endl;
        return 1;
    }else{
        if (StateMachine::current_state_ptr->set_active_subsystems(config_file.c_str())==1){
            return 1;
        }
    }

    RCLCPP_INFO(node->get_logger(), "State Machine Initialized.");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}