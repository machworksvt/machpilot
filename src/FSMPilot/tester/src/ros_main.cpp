#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "FSMPilotStates.hpp"
#include "sub_systems.hpp"
#include <iostream>
#include <memory>

int exit_code;
class FsmControllerNode : public rclcpp::Node
{
public:
    FsmControllerNode()
    : Node("fsm_pilot_tester") 
    {
        RCLCPP_INFO(this->get_logger(), "Node start up");

        state_id=0;

        // --- Subscriber ---
        state_sub_ = this->create_subscription<std_msgs::msg::UInt64>(
            "fsm/current_state", 10, std::bind(&FsmControllerNode::state_callback, this, std::placeholders::_1));

        // --- Publishers ---
        initialize_subsystem_ = this->create_publisher<std_msgs::msg::UInt64>("fsm/events/initialize_subsystem", rclcpp::QoS(2).transient_local());
        arm_motor_ = this->create_publisher<std_msgs::msg::Empty>("fsm/events/arm_motor", 10);
        manual_control_ = this->create_publisher<std_msgs::msg::Empty>("fsm/events/manual_control", 10);
        relinquish_manual_control_ = this->create_publisher<std_msgs::msg::Empty>("fsm/events/relinquish_manual_control", 10);
        disarm_event_ = this->create_publisher<std_msgs::msg::Empty>("fsm/events/disarm", 10);
        shutdown_event_ = this->create_publisher<std_msgs::msg::Empty>("fsm/events/shutdown", 10);
        on_fire_event_ = this->create_publisher<std_msgs::msg::Empty>("fsm/events/on_fire", 10);
        fire_suppressed_event_ = this->create_publisher<std_msgs::msg::Empty>("fsm/events/fire_suppressed", 10);
        

        //send messages that example subsystems are ready.
        auto msg = std_msgs::msg::UInt64();
        msg.data = SubSystems::SUBSYSTEM_0;
        initialize_subsystem_->publish(msg);

        msg = std_msgs::msg::UInt64();
        msg.data = SubSystems::SUBSYSTEM_1;
        initialize_subsystem_->publish(msg);

        std::cout<<"start_up"<< std::endl;
    }

private:
    int state_id=0;

    void state_callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        FSMPilotStates recived_state=static_cast<FSMPilotStates>(msg->data);

        RCLCPP_INFO(this->get_logger(), "received state: '%s'", std::string(get_state_name(recived_state)).c_str());

        if (recived_state==FSMPilotStates::UNINITIALIZED){
            if (state_id==0){
                return;
            }else{
                RCLCPP_INFO(this->get_logger(),"FAILURE: recived UNINITIALIZED in state %d",state_id);
                exit_code=1;
                rclcpp::shutdown();
            };

        }

        switch (state_id)
        {
        case 0:
            if (recived_state!=FSMPilotStates::INITIALIZED){
                RCLCPP_INFO(this->get_logger(), "FAILURE: received state: '%s' in state 0", std::string(get_state_name(recived_state)).c_str());
                exit_code=1;
                rclcpp::shutdown();
            }
            arm_motor_ ->publish(std_msgs::msg::Empty());
            break;
        case 1:
            if (recived_state!=FSMPilotStates::ARMED){
                RCLCPP_INFO(this->get_logger(), "FAILURE: received state: '%s' in state 1", std::string(get_state_name(recived_state)).c_str());
                exit_code=1;
                rclcpp::shutdown();
            }
            manual_control_ ->publish(std_msgs::msg::Empty());
            break;
        case 2:
            if (recived_state!=FSMPilotStates::MANUALFLIGHT){
                RCLCPP_INFO(this->get_logger(), "FAILURE: received state: '%s' in state 2", std::string(get_state_name(recived_state)).c_str());
                exit_code=1;
                rclcpp::shutdown();
            }
            relinquish_manual_control_ ->publish(std_msgs::msg::Empty());
            break;
        case 3:
            if (recived_state!=FSMPilotStates::ARMED){
                RCLCPP_INFO(this->get_logger(), "FAILURE: received state: '%s' in state 3", std::string(get_state_name(recived_state)).c_str());
                exit_code=1;
                rclcpp::shutdown();
            }
            disarm_event_ ->publish(std_msgs::msg::Empty());
            break;
        case 4:
            if (recived_state!=FSMPilotStates::DISARM){
                RCLCPP_INFO(this->get_logger(), "FAILURE: received state: '%s' in state 4", std::string(get_state_name(recived_state)).c_str());
                exit_code=1;
                rclcpp::shutdown();
            }
            shutdown_event_ ->publish(std_msgs::msg::Empty());
            break;
        case 5:
            if (recived_state!=FSMPilotStates::SHUTDOWN){
                RCLCPP_INFO(this->get_logger(), "FAILURE: received state: '%s' in state 5", std::string(get_state_name(recived_state)).c_str());
                exit_code=1;
            }
            rclcpp::shutdown();
            shutdown_event_ ->publish(std_msgs::msg::Empty());
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "FAILURE: state code %d not recognized",state_id);
            exit_code=1;
            rclcpp::shutdown();
        }

        state_id++;

    }

    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr state_sub_;
    
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr initialize_subsystem_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr arm_motor_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr manual_control_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr relinquish_manual_control_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr disarm_event_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr shutdown_event_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr on_fire_event_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr fire_suppressed_event_;
};


int main(int argc, char * argv[])
{
    // Initialize the ROS2 client library
    rclcpp::init(argc, argv);

    std::cout<<"tester_startup"<< std::endl;

    // Create an instance of the node and spin it, allowing callbacks to be processed
    rclcpp::spin(std::make_shared<FsmControllerNode>());

    // Shut down the ROS2 client library
    rclcpp::shutdown();
    return exit_code;
}