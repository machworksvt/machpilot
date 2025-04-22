#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/msg/engine_data.hpp"
#include "interfaces/msg/pump_rpm.hpp"
#include "interfaces/msg/errors.hpp"
#include "interfaces/msg/glow_plugs.hpp"
#include "interfaces/msg/fuel_ambient.hpp"
#include "interfaces/msg/last_run_info.hpp"
#include "interfaces/msg/ng_reg.hpp"
#include "interfaces/msg/statistics.hpp"
#include "interfaces/msg/system_info.hpp"
#include "interfaces/msg/system_info2.hpp"
#include "interfaces/msg/voltage_current.hpp"

#include "interfaces/action/starter_test.hpp"
#include "interfaces/action/prime.hpp"
#include "interfaces/action/igniter_test.hpp"
#include "interfaces/action/pump_test.hpp"
#include "interfaces/action/start.hpp"
#include "interfaces/action/throttle_profile.hpp"



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
using namespace rclcpp;

#define CAN_ID_BASE0 0x100
#define CAN_ID_BASE1 0x120
#define CAN_ID_BASE2 0x140

/*
  4/22/2025: Making the following modifications:
    - Changed logic of start action to start throttle control rather than only sending the keep alive command (was **maybe** causing an issue I don't believe radek entirely)
    - Changing the order of error byte decoding (was backwards)
*/

class CanInterfaceNode : public rclcpp::Node
{
public:
  CanInterfaceNode()
  : Node("h20pro_node")
  {

    can_tx_srv_ = this->create_client<interfaces::srv::SendCanMessage>("/can_tx");

    if(!can_tx_srv_->wait_for_service(10s)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to can_tx service. Ensure the can_bridge_node is running. Exiting.");
      exit(1);
    }

    setup_messsage_map();

    RCLCPP_INFO(this->get_logger(), "CAN interface node initialized");


    /*
      three processing groups are created to handle the different types of callbacks
      throttle_processing_group_ is used to handle the throttle command subscription and the throttle publish timer, reentrant type
      action_callback_group_ is used to handle the action servers, mutually exclusive type, since actions shouldn't happen simultanously
      can_processing_group_ is used to handle the can message subscription, reentrant type
    */
    
    action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    throttle_processing_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    SubscriptionOptions throttle_sub_options;
    throttle_sub_options.callback_group = throttle_processing_group_;
    can_processing_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    SubscriptionOptions can_sub_options;
    can_sub_options.callback_group = can_processing_group_;

    kill_service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // action options
    
    can_rx_sub_ = this->create_subscription<interfaces::msg::CanMsg>(
      "/can_rx", 10, std::bind(&CanInterfaceNode::handle_can_rx, this, _1), can_sub_options); //subscribe to the can_rx topic

    engine_data_pub_ = this->create_publisher<interfaces::msg::EngineData>("/h20pro/engine_data", 10);
    engine2_data_pub_ = this->create_publisher<interfaces::msg::PumpRpm>("/h20pro/engine_data2", 10);
    errors_current_pub_ = this->create_publisher<interfaces::msg::Errors>("/h20pro/errors", 10);
    fuel_ambient_pub_ = this->create_publisher<interfaces::msg::FuelAmbient>("/h20pro/fuel_ambient", 10);
    glow_plugs_pub_ = this->create_publisher<interfaces::msg::GlowPlugs>("/h20pro/glow_plugs", 10);
    last_run_info_pub_ = this->create_publisher<interfaces::msg::LastRunInfo>("/h20pro/last_run_info", 10);
    ng_reg_pub_ = this->create_publisher<interfaces::msg::NgReg>("/h20pro/ng_reg", 10);
    statistics_pub_ = this->create_publisher<interfaces::msg::Statistics>("/h20pro/statistics", 10);
    system_info_pub_ = this->create_publisher<interfaces::msg::SystemInfo>("/h20pro/system_info", 10);
    system_info2_pub_ = this->create_publisher<interfaces::msg::SystemInfo2>("/h20pro/system_info2", 10);
    voltage_current_pub_ = this->create_publisher<interfaces::msg::VoltageCurrent>("/h20pro/voltage_current", 10);

    throttle_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>("/h20pro/throttle_command", 10, std::bind(&CanInterfaceNode::throttle_command_callback, this, _1), throttle_sub_options); //this prolly isn't right

    this->starter_test_action_server_ = rclcpp_action::create_server<interfaces::action::StarterTest>(
      this,
      "/h20pro/starter_test",
      std::bind(&CanInterfaceNode::handle_starter_test_goal, this, _1, _2),
      std::bind(&CanInterfaceNode::handle_starter_test_cancel, this, _1),
      std::bind(&CanInterfaceNode::handle_starter_test_accepted, this, _1),
      rcl_action_server_get_default_options(),
      action_callback_group_
    ); //register and bind the action for the starter test.

    this->pump_test_action_server_ = rclcpp_action::create_server<interfaces::action::PumpTest>(
      this,
      "/h20pro/pump_test",
      std::bind(&CanInterfaceNode::handle_pump_test_goal, this, _1, _2),
      std::bind(&CanInterfaceNode::handle_pump_test_cancel, this, _1),
      std::bind(&CanInterfaceNode::handle_pump_test_accepted, this, _1),
      rcl_action_server_get_default_options(),
      action_callback_group_
    ); //register and bind the action for the pump test.

    this->igniter_test_action_server_ = rclcpp_action::create_server<interfaces::action::IgniterTest>(
      this,
      "/h20pro/igniter_test",
      std::bind(&CanInterfaceNode::handle_igniter_test_goal, this, _1, _2),
      std::bind(&CanInterfaceNode::handle_igniter_test_cancel, this, _1),
      std::bind(&CanInterfaceNode::handle_igniter_test_accepted, this, _1),
      rcl_action_server_get_default_options(),
      action_callback_group_
    ); //register and bind the action for the igniter test.

    this->prime_action_server_ = rclcpp_action::create_server<interfaces::action::Prime>(
      this,
      "/h20pro/prime",
      std::bind(&CanInterfaceNode::handle_prime_goal, this, _1, _2),
      std::bind(&CanInterfaceNode::handle_prime_cancel, this, _1),
      std::bind(&CanInterfaceNode::handle_prime_accepted, this, _1),
      rcl_action_server_get_default_options(),
      action_callback_group_
    ); //register and bind the action for the prime test.

    this->start_action_server_ = rclcpp_action::create_server<interfaces::action::Start>(
      this,
      "/h20pro/start",
      std::bind(&CanInterfaceNode::handle_start_goal, this, _1, _2),
      std::bind(&CanInterfaceNode::handle_start_cancel, this, _1),
      std::bind(&CanInterfaceNode::handle_start_accepted, this, _1),
      rcl_action_server_get_default_options(),
      action_callback_group_
    ); //register and bind the action for the start test.
    //TODO: The other 3 actions
    //the kill service/handling of running mode
    kill_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/h20pro/kill",
      std::bind(&CanInterfaceNode::handle_kill_service, this, _1, _2),
      rmw_qos_profile_default,
      kill_service_group_
    );

    // this timer broadcasts keep-alive and throttle commands at 20Hz
    throttle_timer_ = this->create_wall_timer( 
      50ms, std::bind(&CanInterfaceNode::send_throttle_command_callback, this), throttle_processing_group_);
    
      RCLCPP_INFO(this->get_logger(), "finished constructor");
  }

  ~CanInterfaceNode()
  {
    kill_all();
  }

private:
  void handle_can_rx(const interfaces::msg::CanMsg::SharedPtr msg) {
    //not locking the thread here because it doesn't need to be locked for the entire duration of message processing.
    //lock happens in each message handler
    uint32_t can_id = msg->id;
    uint8_t can_dlc = msg->dlc;
    if (message_map_.find(can_id) != message_map_.end()) {
      if(can_dlc < message_map_[can_id].expected_dlc) {
        RCLCPP_WARN(this->get_logger(), "Received CAN message with incorrect DLC. Expected %u, got %u", message_map_[can_id].expected_dlc, can_dlc);
        return;
      }
      message_map_[can_id].handler(*msg);
    }
  }

private:
  void throttle_command_callback(const std_msgs::msg::Float32 & msg) {
    //validate input
    if (msg.data < 0 || msg.data > 100) {
      RCLCPP_WARN(this->get_logger(), "Throttle command out of range (0-100%%): %f. Clamping.", msg.data);
    } 
    throttle_cmd_.store(std::max(0.0f, std::min(msg.data, 100.0f)));
  }

private:
  void send_throttle_command_callback() {
    if (!under_throttle_control_.load()) return; //if we are not under throttle control, don't send the command
    interfaces::msg::CanMsg throttle_message = interfaces::msg::CanMsg(throttle_off_msg_);
    //the first 2 bytes are the throttle command
    //byte 0 is the high byte, byte 1 is the low byte
    uint16_t throttle_cmd_value = static_cast<uint16_t>(throttle_cmd_.load() * 10); //convert to 0-1000 range
    throttle_message.data[0] = (throttle_cmd_value >> 8) & 0xFF;
    throttle_message.data[1] = throttle_cmd_value & 0xFF;

    send_keep_alive_command(); //send the keep alive command
                              // one of the things I changed here - keep alive and throttle are now synchronous always

    if (!send_can_msg(throttle_message)) {
      RCLCPP_WARN(this->get_logger(), "Failed to write throttle command to CAN bus. Disabling throttle control for safety.");
      if (!kill_control()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to disable throttle control. May be emergency situation.");
      }
    }
  }

private:
  void send_keep_alive_command() {
    if (!send_can_msg(keep_alive_msg_)) {
      RCLCPP_WARN(this->get_logger(), "Failed to write keep alive command to CAN bus. Disabling engine control for safety.");
      if (!kill_control()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to disable throttle control. May be emergency situation.");
      }
    }
  }

private:
  void start_throttle_control() {
    under_throttle_control_.store(true);
    throttle_cmd_.store(0.0f); //reset the throttle command
  }

private:
  void handle_kill_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (kill_all()) {
        response->success = true;
        response->message = "All tests and control modes have been stopped.";
      } else {
        response->success = false;
        response->message = "Failed to stop all tests and control modes.";
      }
    }

  // Handle the starter test
private:
  rclcpp_action::GoalResponse handle_starter_test_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const interfaces::action::StarterTest::Goal> goal) { //callback for handling request
      RCLCPP_INFO(this->get_logger(), "Received Request to run starter test.");
      {
        std::lock_guard<std::mutex> lock(state_mutex_); //lock because we are going to be reading the state of the engine
        if (current_engine_data_.state != 0) { //engine needs to be in state 0 to attempt start
          RCLCPP_WARN(this->get_logger(), "Engine is not in OFF state - Rejecting starter test request.");
          return rclcpp_action::GoalResponse::REJECT;
        }
      }
      RCLCPP_INFO(this->get_logger(), "Accepting request to run starter test.");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

private:
  void handle_starter_test_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::StarterTest>> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&CanInterfaceNode::execute_starter_test, this, goal_handle)}.detach(); //dispatch a thread to handle the actual execution
  }

private:
  void execute_starter_test(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::StarterTest>> goal_handle) {
    RCLCPP_DEBUG(this->get_logger(), "Starting starter test.");

    // create the feedback and result objects (java moment, they're actually shared pointers lol)
    // no feedback on this action
    auto result = std::make_shared<interfaces::action::StarterTest::Result>();

    // Generate the can messages that actually run this
    interfaces::msg::CanMsg starter_test_msg = interfaces::msg::CanMsg(test_off_msg_);
    starter_test_msg.data[5] = 1;

    RCLCPP_DEBUG(this->get_logger(), "Sending starter test frame");
    if (!send_can_msg(starter_test_msg)) {
      //failed to write the can message for some reason.
      result->success = false;
      goal_handle->abort(result); //aborting the test for safety.
      return;
    }

    auto start_time = this->now();

    bool has_passed = false;

    while(this->now() - start_time < 15s) {
      if(goal_handle->is_canceling()) { //if we are cancelled, stop the test
        result->success = has_passed;
        kill_tests();
        goal_handle->canceled(result);
        return;
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_); //lock because we are going to be reading the state of the engine
        if(current_engine_data_.real_rpm > 6000) { //if we hit the target RPM, declare that we passed
          has_passed = true;
        }
      }
      if(!check_all_healthy()) { //if errors are encountered, we fail the test.
        kill_tests();
        has_passed = false;
        break;
      }

      std::this_thread::sleep_for(100ms);
    }

    //regardless of if we succeded or not, unlatch the test byte
    if (!kill_tests()) { //I really hope this doesn't happen, this means the can bus failed to write the command to end the test
      //failed to write the can message for some reason.
      result->success = false;
      goal_handle->abort(result); //aborting the test for safety.
      return;
    }

    result->success = has_passed; //update the result with if we finished or not
    goal_handle->succeed(result); //finish ROS2 action
    return; //
  }

private:
  rclcpp_action::CancelResponse handle_starter_test_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::StarterTest>> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Cancelling starter test");
      (void)goal_handle; //uhhhhh
      kill_tests();
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  
  //Handle the pump test
private:
  rclcpp_action::GoalResponse handle_pump_test_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const interfaces::action::PumpTest::Goal> goal) { //callback for handling request
      RCLCPP_INFO(this->get_logger(), "Received Request to run pump test.");
      {
        std::lock_guard<std::mutex> lock(state_mutex_); //lock because we are going to be reading the state of the engine
        if (current_engine_data_.state != 0) { //engine needs to be in state 0 to attempt start
          RCLCPP_WARN(this->get_logger(), "Engine is not in OFF state - Rejecting pump test request.");
          return rclcpp_action::GoalResponse::REJECT;
        }
      }
      if (goal->fuel_ml <= 0 || goal->fuel_ml > 4500) { //reaonable check for range of acceptable requests
        RCLCPP_WARN(this->get_logger(), "Pump request of %u ml is out of range (0-4500) - Rejecting pump test request.", goal->fuel_ml);
        return rclcpp_action::GoalResponse::REJECT;
      }

      if (goal->pump_power_percent <= 0 || goal->pump_power_percent > 100) { //pump power needs to be reasonable
        RCLCPP_WARN(this->get_logger(), "Pump request of %f%% is out of range (0-100%%) - Rejecting pump test request.", goal->pump_power_percent);
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(this->get_logger(), "Accepting request to run pump test. Pumping %u ml at %f%% power.", goal->fuel_ml, goal->pump_power_percent);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

private:
  void handle_pump_test_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::PumpTest>> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&CanInterfaceNode::execute_pump_test, this, goal_handle)}.detach(); //dispatch a thread to handle the actual execution
  }

private:
  void execute_pump_test(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::PumpTest>> goal_handle) {
    RCLCPP_DEBUG(this->get_logger(), "Running pump test.");

    auto feedback = std::make_shared<interfaces::action::PumpTest::Feedback>();
    auto result = std::make_shared<interfaces::action::PumpTest::Result>();
    std::shared_ptr<const interfaces::action::PumpTest_Goal> goal = goal_handle->get_goal();

    interfaces::msg::CanMsg pump_test_msg = interfaces::msg::CanMsg(test_off_msg_);
    //encode the pump power requested strength into the frame
    pump_test_msg.data[0] = static_cast<uint8_t>(goal->pump_power_percent / 0.4); //hope this works, lol

    //assume inputs have been validated at this point, if not, rip
    uint32_t pump_start_count_ml;
    
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pump_start_count_ml = current_fuel_ambient_.fuel_consumed;
    }

    if (!send_can_msg(pump_test_msg)) {
      RCLCPP_WARN(this->get_logger(), "Failed to write to CAN bus for pump test.");
      result->success = false;
      goal_handle->abort(result); //abort out due to failure
      return;
    }

    //otherwise we have started the test

    //TODO: probably should add a timeout

    while(true) { //while we haven't pumped the desired amount of fuel yet
      //briefly lock to obtain the relevant data
      uint32_t fuel_consumed;
      uint32_t pump_rpm;
      uint32_t fuel_flow;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        fuel_consumed = current_fuel_ambient_.fuel_consumed;
        pump_rpm = current_pump_data_.pump_rpm;
        fuel_flow = current_fuel_ambient_.fuel_flow;
      }

      //check if we meed break condition
      if ((fuel_consumed - pump_start_count_ml) >= goal->fuel_ml) {
        break;
      }
      
      feedback->fuel_pumped_ml = (fuel_consumed - pump_start_count_ml);
      feedback->fuel_pump_rate = fuel_flow;

      if(goal_handle->is_canceling()) { //if we are cancelled, stop the test
        result->success = false;
        kill_tests();
        goal_handle->canceled(result);
        return;
      }

      goal_handle->publish_feedback(feedback); //update the requester

      RCLCPP_INFO(this->get_logger(), "Pumping fuel at %u RPM, %u/%u ml", pump_rpm, (fuel_consumed - pump_start_count_ml), goal->fuel_ml);

      if(!check_all_healthy()) { //if errors are encountered, fail the test
        RCLCPP_WARN(this->get_logger(), "Errors encountered during pump test. Stopping.");
        kill_tests();
        result->success = false;
        goal_handle->succeed(result);
        return;
      }

      std::this_thread::sleep_for(100ms); //avoid busy loop
    }

    //test has finished
    if (!kill_tests()) { //I really hope this doesn't happen, this means the can bus failed to write the command to end the test
      //failed to write the can message for some reason.
      result->success = false;
      goal_handle->abort(result); //aborting the test for safety.
      return;
    }

    result->success = true; //if we reach this part, we successfully pumped the right amount of fuel
    goal_handle->succeed(result);
    return;
  }

private:
  rclcpp_action::CancelResponse handle_pump_test_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::PumpTest>> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Cancelling pump test");
      (void)goal_handle; //uhhhhh
      kill_tests();
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  
  //Handle igniter test
private:
  rclcpp_action::GoalResponse handle_igniter_test_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const interfaces::action::IgniterTest::Goal> goal) { //callback for handling request
      std::lock_guard<std::mutex> lock(state_mutex_); //lock because we are going to be reading the state of the engine
      RCLCPP_INFO(this->get_logger(), "Received Request to run igniter test.");

      if (current_engine_data_.state != 0) { //engine needs to be in state 0 to attempt start
        RCLCPP_WARN(this->get_logger(), "Engine is not in OFF state - Rejecting igniter test request.");
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(this->get_logger(), "Accepting request to run igniter test. Running glow plugs.");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
  
private:
  void handle_igniter_test_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::IgniterTest>> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&CanInterfaceNode::execute_igniter_test, this, goal_handle)}.detach(); //dispatch a thread to handle the actual execution
  }

private:
  void execute_igniter_test(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::IgniterTest>> goal_handle) {
    RCLCPP_DEBUG(this->get_logger(), "Running igniter test.");

    auto result = std::make_shared<interfaces::action::IgniterTest::Result>();

    interfaces::msg::CanMsg igniter_test_msg = interfaces::msg::CanMsg(test_off_msg_);

    igniter_test_msg.data[4] = 1;

    if (!send_can_msg(igniter_test_msg)) {
      RCLCPP_WARN(this->get_logger(), "Failed to write to CAN bus for igniter test.");
      result->success = false;
      goal_handle->abort(result); //abort out due to failure
      return;
    }

    auto start_time = this->now();

    bool has_passed = false;

    while(this->now() - start_time < 15s) {
      if(goal_handle->is_canceling()) { //if we are cancelled, stop the test
        result->success = has_passed;
        kill_tests();
        goal_handle->canceled(result);
        return;
      }

      {
        std::lock_guard<std::mutex> lock(state_mutex_); //lock because we are going to be reading the state of the engine
        if(current_glow_plugs_.glow_plug_i[0] > 1 && current_glow_plugs_.glow_plug_i[1] > 1) { //if we see current on both connectors > 1A, we declare we passed.
          has_passed = true;
        }
      }

      if(!check_all_healthy()) { //if errors are encountered, we fail the test.
        kill_tests();
        has_passed = false;
        break;
      }

      std::this_thread::sleep_for(100ms);
    }

    //regardless of if we succeded or not, unlatch the test byte
    if (!kill_tests()) { //I really hope this doesn't happen, this means the can bus failed to write the command to end the test
      //failed to write the can message for some reason.
      result->success = false;
      goal_handle->abort(result); //aborting the test for safety.
      return;
    }

    result->success = has_passed; //update the result with if we finished or not
    goal_handle->succeed(result); //finish ROS2 action
    return; 
  }

private:
  rclcpp_action::CancelResponse handle_igniter_test_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::IgniterTest>> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Cancelling igniter test");
      (void)goal_handle; //uhhhhh
      kill_tests();
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  
  //Handle the prime action
private:
  rclcpp_action::GoalResponse handle_prime_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const interfaces::action::Prime::Goal> goal) { //callback for handling request
      std::lock_guard<std::mutex> lock(state_mutex_); //lock because we are going to be reading the state of the engine
      RCLCPP_INFO(this->get_logger(), "Received Request to run prime test.");

      if (current_engine_data_.state != 0) { //engine needs to be in state 0 to attempt start
        RCLCPP_WARN(this->get_logger(), "Engine is not in OFF state - Rejecting prime test request.");
        return rclcpp_action::GoalResponse::REJECT;
      }

      if (goal->pump_power_percent > 10 || goal->pump_power_percent <= 0) { //reaonable check for range of acceptable requests
        RCLCPP_WARN(this->get_logger(), "Prime request of at %f%% pump power out of range (0,10]%%. - Rejecting prime test request.", goal->pump_power_percent);
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(this->get_logger(), "Accepting request to run prime test. Pumping at %f%% power.", goal->pump_power_percent);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

private:
  void handle_prime_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Prime>> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&CanInterfaceNode::execute_prime, this, goal_handle)}.detach(); //dispatch a thread to handle the actual execution
  }

private:
  void execute_prime(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Prime>> goal_handle) {
    RCLCPP_DEBUG(this->get_logger(), "Running prime test.");

    auto result = std::make_shared<interfaces::action::Prime::Result>();

    std::shared_ptr<const interfaces::action::Prime_Goal> goal = goal_handle->get_goal(); //pull the goal (need the requested pump power)
    interfaces::msg::CanMsg prime_test_msg = interfaces::msg::CanMsg(test_off_msg_);
    prime_test_msg.data[6] = static_cast<uint8_t>(goal->pump_power_percent / 0.4);

    // whole test is supposed to take 15 seconds

    if (!send_can_msg(prime_test_msg)) {
      RCLCPP_WARN(this->get_logger(), "Failed to write to CAN bus for prime test.");
      result->success = false;
      goal_handle->abort(result); //abort out due to failure
      return;
    }

    auto start_time = this->now();
    
    bool has_passed = false;

    while(this->now() - start_time < 15s) {

      if(goal_handle->is_canceling()) { //if we are cancelled, stop the test
        result->success = has_passed;
        kill_tests();
        goal_handle->canceled(result);
        return;
      }

      if (!has_passed) { //if we haven't passed yet, check if we have fuel flow, avoiding locking when it's not necessary
        std::lock_guard<std::mutex> lock(state_mutex_); //lock because we are going to be reading the state of the engine
        if(current_fuel_ambient_.fuel_flow > 0) { //if we see fuel flow, we declare success
          has_passed = true;
        }
      }
      
      if(!check_all_healthy()) { //if errors are encountered, we fail the test.
        RCLCPP_WARN(this->get_logger(), "Errors encountered during priming. Stopping.");
        has_passed = false;
        break;
      }

      std::this_thread::sleep_for(100ms);
    }

    if (!kill_tests()) { //I really hope this doesn't happen, this means the can bus failed to write the command to end the test
      //failed to write the can message for some reason.
      result->success = false;
      goal_handle->abort(result); //aborting the test for safety.
      return;
    }

    //if we got this far, priming sequence executed normally.

    result->success = has_passed; //update the result with if we finished or not
    goal_handle->succeed(result); //finish ROS2 action
    return;
  }

private:
  rclcpp_action::CancelResponse handle_prime_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Prime>> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Cancelling prime test");
      (void)goal_handle; //uhhhhh
      kill_tests();
      return rclcpp_action::CancelResponse::ACCEPT;
    }

  //Handle the start action
private:
  rclcpp_action::GoalResponse handle_start_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const interfaces::action::Start::Goal> goal) { //callback for handling request
      std::lock_guard<std::mutex> lock(state_mutex_); //lock because we are going to be reading the state of the engine
      RCLCPP_INFO(this->get_logger(), "Received Request to start engine. This is NOT a test.");

      if (current_engine_data_.state != 0) { //engine needs to be in state 0 to attempt start
        RCLCPP_WARN(this->get_logger(), "Engine is not in OFF state - Rejecting startup request.");
        return rclcpp_action::GoalResponse::REJECT;
      }

      RCLCPP_INFO(this->get_logger(), "Accepting request to start engine. Best of luck.");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

private:
  void handle_start_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Start>> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&CanInterfaceNode::execute_start, this, goal_handle)}.detach(); //dispatch a thread to handle the actual execution
  }

private:
  void execute_start(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Start>> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Starting engine.");

      auto result = std::make_shared<interfaces::action::Start::Result>();

      if (!send_can_msg(keep_alive_msg_)) {
        RCLCPP_WARN(this->get_logger(), "Failed to write to CAN bus for start command.");
        result->success = false;
        goal_handle->abort(result); //abort out due to failure
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Sent start command. Waiting for engine to start.");

      auto start_time = this->now();
      bool has_passed = false;

      start_throttle_control(); //start throttle control, we are going to need it.

      while(this->now() - start_time < 90s) { //90 second timeout. We exit the loop if we reach state 5, running.
        
        if(goal_handle->is_canceling()) { //if we are cancelled, stop the test
          result->success = false;
          kill_control();
          kill_throttle();
          goal_handle->canceled(result);
          return;
        }

        {
          std::lock_guard<std::mutex> lock(state_mutex_); //lock because we are going to be reading the state of the engine}
          RCLCPP_INFO(this->get_logger(), "Engine state: %u", current_engine_data_.state);
          if(current_engine_data_.state == 11) { //if we see the engine state change to 11, we declare success
            has_passed = true;
            break;
          }
        }

        if(!check_all_healthy()) { //if errors are encountered, we fail the test.
          RCLCPP_WARN(this->get_logger(), "Errors encountered during start. Stopping.");
          has_passed = false;
          break;
        }

        std::this_thread::sleep_for(10ms); //small sleep to avoid busy loop
      }

      if (has_passed) {
        //we succeeded
        //we can safely enable throttle control
        result->success = true;
        goal_handle->succeed(result);
        return;
      } else {
          //we failed to start, or something else happened that caused cancellation, like the timeout
          kill_control(); //Attempt to kill the control completely.
          result->success = false;
          goal_handle->abort(result); //aborting the test for safety.
          return;
      }
    }

private:
  rclcpp_action::CancelResponse handle_start_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::Start>> goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Cancelling start");
      (void)goal_handle; //uhhhhh
      kill_control();
      return rclcpp_action::CancelResponse::ACCEPT;
    }

private: 
  bool kill_all() {
    return kill_throttle() && kill_tests() && kill_control();
  }

private:
  bool kill_tests() {
    return send_can_msg(test_off_msg_);
  }

private:
  bool kill_control() {
    under_throttle_control_.store(false);
    return send_can_msg(control_off_msg_);
  }

private:
  bool kill_throttle() {
    return send_can_msg(throttle_off_msg_);
  }

  struct CanMessageDescriptor {
    uint8_t expected_dlc;
    std::string name;
    std::function<void(const interfaces::msg::CanMsg & msg)> handler;
  };

  bool send_can_msg(interfaces::msg::CanMsg msg) {
    auto request = std::make_shared<interfaces::srv::SendCanMessage::Request>();
    request->msg = msg;
    auto future = can_tx_srv_->async_send_request(request);
  
    // Wait for the future with a timeout loop without calling spin.
    auto timeout = std::chrono::seconds(2);
    auto start_time = std::chrono::steady_clock::now();
    while (future.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for can_tx service response");
        return false;
      }
      std::this_thread::sleep_for(5ms);
    }
    return future.get()->success;
  }  

private:
  void setup_messsage_map() {
    message_map_[CAN_ID_BASE0] = {
      8,
      "Engine Data",
      std::bind(&CanInterfaceNode::handle_rpm_state_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 1] = {
      5,
      "Voltage/Current",
      std::bind(&CanInterfaceNode::handle_voltage_current_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 2] = {
      7,
      "Fuel/Ambient",
      std::bind(&CanInterfaceNode::handle_fuel_ambient_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 3] = {
      8,
      "Statistics",
      std::bind(&CanInterfaceNode::handle_statistics_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 4] = {
      8,
      "Last Run Info",
      std::bind(&CanInterfaceNode::handle_last_run_info_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 5] = {
      6,
      "System Info",
      std::bind(&CanInterfaceNode::handle_system_info_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 6] = {
      2,
      "Pump RPM",
      std::bind(&CanInterfaceNode::handle_pump_rpm_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 7] = {
      7,
      "Errors",
      std::bind(&CanInterfaceNode::handle_errors_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 8] = {
      6,
      "Glow Plugs",
      std::bind(&CanInterfaceNode::handle_glow_plugs_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 9] = {
      8,
      "NgReg",
      std::bind(&CanInterfaceNode::handle_ngreg_message, this, _1)
    };
    message_map_[CAN_ID_BASE0 + 10] = {
      8,
      "System Info 2",
      std::bind(&CanInterfaceNode::handle_system_info_2_message, this, _1)
    };

    control_off_msg_.id = CAN_ID_BASE1;
    control_off_msg_.dlc = 1;
    control_off_msg_.data = {0, 0, 0, 0, 0, 0, 0, 0};

    keep_alive_msg_.id = CAN_ID_BASE1;
    keep_alive_msg_.dlc = 1;
    keep_alive_msg_.data = {1, 0, 0, 0, 0, 0, 0, 0};

    test_off_msg_.id = CAN_ID_BASE1 + 4;
    test_off_msg_.dlc = 7;
    test_off_msg_.data = {0, 0, 0, 0, 0, 0, 0, 0};

    throttle_off_msg_.id = CAN_ID_BASE1 + 1;
    throttle_off_msg_.dlc = 2;
    throttle_off_msg_.data = {0, 0, 0, 0, 0, 0, 0, 0};

  }

private:
  void handle_rpm_state_message(interfaces::msg::CanMsg msg) {
    auto engine_pub_msg_ = interfaces::msg::EngineData();

    uint16_t frame_id = msg.id;
    engine_pub_msg_.can_msg = msg;
    engine_pub_msg_.header = create_header(frame_id);


    uint32_t set_rpm = ((static_cast<uint16_t>(msg.data[0]) << 8) | msg.data[1]) * 10;
    uint16_t real_rpm = ((static_cast<uint16_t>(msg.data[2]) << 8) | msg.data[3]) * 10;
    float egt = ((static_cast<int16_t>(msg.data[4]) << 8) | msg.data[5]) * 0.1;
    uint8_t state = msg.data[6];
    std::string state_name = state_name_map_[state];
    float pump_power = msg.data[7] * 0.5;

    engine_pub_msg_.set_rpm = set_rpm;
    engine_pub_msg_.real_rpm = real_rpm;
    engine_pub_msg_.egt = egt;
    engine_pub_msg_.state = state;
    engine_pub_msg_.state_name = state_name;
    engine_pub_msg_.pump_power = pump_power;

    engine_data_pub_->publish(engine_pub_msg_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_engine_data_ = engine_pub_msg_;
    }

    RCLCPP_DEBUG(this->get_logger(),
                "Engine Data (CAN ID 0x100): SetRPM: %u 1/min, RealRPM: %u 1/min, EGT: %.1f°C, State: %uv (%s), Pump Power: %.1f%%",
                set_rpm, real_rpm, egt, state, state_name.c_str(), pump_power);
  }

private:
  void handle_voltage_current_message(interfaces::msg::CanMsg msg) {
    auto voltage_current_msg_ = interfaces::msg::VoltageCurrent();

    uint16_t frame_id = msg.id;
    voltage_current_msg_.can_msg = msg;
    voltage_current_msg_.header = create_header(frame_id);

    float battery_voltage = static_cast<uint8_t>(msg.data[0]) * 0.1;
    float engine_current = static_cast<uint8_t>(msg.data[1]) * 0.2;
    uint8_t flags = static_cast<uint8_t>(msg.data[4]);

    voltage_current_msg_.battery_voltage = battery_voltage;
    voltage_current_msg_.battery_current = engine_current;
    voltage_current_msg_.flags = flags;

    voltage_current_pub_->publish(voltage_current_msg_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_voltage_current_ = voltage_current_msg_;
    }
    

    RCLCPP_DEBUG(this->get_logger(), "Voltage/Current Data (CAN ID 0x101): BV: %f V, EC: %f A", battery_voltage, engine_current);
  }

private:
  void handle_fuel_ambient_message(interfaces::msg::CanMsg msg) {
    auto fuel_ambient_msg_ = interfaces::msg::FuelAmbient();

    uint16_t frame_id = msg.id;
    fuel_ambient_msg_.can_msg = msg;
    fuel_ambient_msg_.header = create_header(frame_id);

    uint32_t fuel_flow = ((static_cast<uint16_t>(msg.data[0] << 8)) | msg.data[1]); //ml/min 
    uint32_t fuel_consumed = ((static_cast<uint16_t>(msg.data[2] << 8)) | msg.data[3]) * 10; //ml
    float engine_box_pressure = ((static_cast<uint16_t>(msg.data[4] << 8)) | msg.data[5]) * 0.02; //mbar
    int8_t ambient_temperature = msg.data[6];

    fuel_ambient_msg_.fuel_flow = fuel_flow;
    fuel_ambient_msg_.fuel_consumed = fuel_consumed;
    fuel_ambient_msg_.engine_box_pressure = engine_box_pressure;
    fuel_ambient_msg_.ambient_temperature = ambient_temperature;

    fuel_ambient_pub_->publish(fuel_ambient_msg_);


    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_fuel_ambient_ = fuel_ambient_msg_;
    }
    

    RCLCPP_DEBUG(this->get_logger(), "Fuel/Ambient (CAN ID 0x102): Fuel Rate: %u ml/min, Fuel Consumed: %u ml, Engine Pressure: %f mbar, Ambient Temp: %d C", 
                fuel_flow, fuel_consumed, engine_box_pressure, ambient_temperature);

  }

private:
  void handle_statistics_message(interfaces::msg::CanMsg msg) {
    auto statistics_msg_ = interfaces::msg::Statistics();

    uint16_t frame_id = msg.id;
    statistics_msg_.can_msg = msg;
    statistics_msg_.header = create_header(frame_id);

    uint16_t runs_ok = ((static_cast<uint16_t>(msg.data[0] << 8)) | msg.data[1]);
    uint16_t runs_aborted = ((static_cast<uint16_t>(msg.data[2] << 8)) | msg.data[3]); 
    uint32_t total_runtime = (static_cast<uint32_t>(msg.data[4] << 24)) | (static_cast<uint32_t>(msg.data[5] << 16)) | (static_cast<uint32_t>(msg.data[6] << 8)) | msg.data[7];

    statistics_msg_.runs_ok = runs_ok;
    statistics_msg_.runs_aborted = runs_aborted;
    statistics_msg_.total_runtime = total_runtime;

    statistics_pub_->publish(statistics_msg_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_statistics_ = statistics_msg_;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Statistics (CAN ID 0x103): Runs OK: %u, Runs Failed: %u, Total Runtime: %u s", runs_ok, runs_aborted, total_runtime);
  }

private:
  void handle_last_run_info_message(interfaces::msg::CanMsg msg) {
    auto last_run_info_msg_ = interfaces::msg::LastRunInfo();

    uint16_t frame_id = msg.id;
    last_run_info_msg_.can_msg = msg;
    last_run_info_msg_.header = create_header(frame_id);

    uint16_t last_run_time = ((static_cast<uint16_t>(msg.data[0] << 8)) | msg.data[1]);
    uint16_t last_off_rpm = ((static_cast<uint16_t>(msg.data[2] << 8)) | msg.data[3]);
    uint16_t last_off_egt = ((static_cast<uint16_t>(msg.data[4] << 8)) | msg.data[5]);
    float last_off_pump_power = static_cast<uint8_t>(msg.data[6]) * 0.5;
    uint8_t last_off_state = msg.data[7];
    std::string last_off_state_str = state_name_map_[last_off_state];
    
    last_run_info_msg_.last_runtime = last_run_time;
    last_run_info_msg_.last_off_rpm = last_off_rpm;
    last_run_info_msg_.last_off_egt = last_off_egt;
    last_run_info_msg_.last_off_pump_power = last_off_pump_power;
    last_run_info_msg_.last_off_state = last_off_state;
    last_run_info_msg_.last_off_state_str = last_off_state_str;

    last_run_info_pub_->publish(last_run_info_msg_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_last_run_info_ = last_run_info_msg_;
    }

    RCLCPP_DEBUG(this->get_logger(), "Last Run Info: (CAN ID 0x104), Last Runtime: %u s, Last Off RPM: %u 1/min, Last Off EGT: %u, Last Off Pump Power: %.1f%%, Last Off State: %u (%s)", 
                last_run_time, last_off_rpm, last_off_egt, last_off_pump_power, last_off_state, last_off_state_str.c_str());
  }

private:
  void handle_system_info_message(interfaces::msg::CanMsg msg) {
    auto system_info_message_ = interfaces::msg::SystemInfo();

    uint16_t frame_id = msg.id;
    system_info_message_.can_msg = msg;
    system_info_message_.header = create_header(frame_id);

    uint16_t serial_number = ((static_cast<uint16_t>(msg.data[0] << 8)) | msg.data[1]);
    uint16_t fw_version = ((static_cast<uint16_t>(msg.data[2] << 8)) | msg.data[3]);
    uint8_t engine_type = msg.data[4];
    uint8_t engine_subtype = msg.data[5];

    system_info_message_.serial_number = serial_number;
    system_info_message_.fw_version = fw_version;
    system_info_message_.engine_type = engine_type;
    system_info_message_.engine_subtype = engine_subtype;

    system_info_pub_->publish(system_info_message_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_system_info_ = system_info_message_;
    }

    RCLCPP_DEBUG(this->get_logger(), "System Info (CAN ID 0x105): Serial Number: %u, FW Version: %u, Engine Type: %u, Engine Subtype: %u", 
                serial_number, fw_version, engine_type, engine_subtype);
  }

private:
  void handle_pump_rpm_message(interfaces::msg::CanMsg msg) {
    auto pump_rpm_message_ = interfaces::msg::PumpRpm();

    uint16_t frame_id = msg.id;
    pump_rpm_message_.can_msg = msg;
    pump_rpm_message_.header = create_header(frame_id);

    uint32_t pump_rpm = static_cast<uint16_t>(msg.data[0] << 8) | msg.data[1];

    pump_rpm_message_.pump_rpm = pump_rpm;

    engine2_data_pub_->publish(pump_rpm_message_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_pump_data_ = pump_rpm_message_;
    }

    RCLCPP_DEBUG(this->get_logger(), "Pump RPM (CAN ID 0x106): Pump RPM: %u 1/min", pump_rpm);
  }
  
private:
  void handle_errors_message(interfaces::msg::CanMsg msg) {
    auto errors_message_ = interfaces::msg::Errors();

    uint16_t frame_id = msg.id;
    errors_message_.can_msg = msg;
    errors_message_.header = create_header(frame_id);

    uint64_t error_mask = (static_cast<uint64_t>(msg.data[0]) << 56) | (static_cast<uint64_t>(msg.data[1]) << 48) | (static_cast<uint64_t>(msg.data[2]) << 40) | (static_cast<uint64_t>(msg.data[3]) << 32) | (static_cast<uint64_t>(msg.data[4]) << 24) | (static_cast<uint64_t>(msg.data[5]) << 16) | (static_cast<uint64_t>(msg.data[6]) << 8) | msg.data[7];
    //another change here -- swapped the order
    std::vector<std::string> error_messages;

    bool reset_required = false;

    for (uint8_t bit = 0; bit < 64; ++bit) { //needed to flip the order this is traversed in
      // Check if the bit is set.
      if ((error_mask >> bit) & 1ULL) {
        // Look up the error in the map.
        auto it = error_map_.find(63 - bit);
        if (it != error_map_.end()) {
          // Format an error message. For example: "PreStart RPM high: Rotor speed too high for startup. ..."
          std::string err_message = it->second.name + ": " + it->second.description;
          error_messages.push_back(err_message);
          // If any active error requires a reset, mark the entire message as such.
          if (it->second.requires_reset) {
            reset_required = true;
          }
        }
      }
    }

    errors_message_.error_mask = error_mask;
    errors_message_.error_messages = error_messages;
    errors_message_.reset_required = reset_required;

    errors_current_pub_->publish(errors_message_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_errors_ = errors_message_;
    }

    //RCLCPP_WARN(this->get_logger(), "Errors (CAN ID 0x107): Error Mask: 0x%016lX", error_mask);
    //for (const auto & err : error_messages) {
    //  RCLCPP_WARN(this->get_logger(), "  %s", err.c_str());
    //}
  }

private:
  void handle_glow_plugs_message(interfaces::msg::CanMsg msg) {
    auto glow_plugs_message_ = interfaces::msg::GlowPlugs();

    uint16_t frame_id = msg.id;
    glow_plugs_message_.can_msg = msg;
    glow_plugs_message_.header = create_header(frame_id);

    float glow_plug_1_v = static_cast<uint8_t>(msg.data[0]) * 0.1;
    float glow_plug_1_i = static_cast<uint8_t>(msg.data[1]) * 0.1;
    float glow_plug_2_v = static_cast<uint8_t>(msg.data[2]) * 0.1;
    float glow_plug_2_i = static_cast<uint8_t>(msg.data[3]) * 0.1;
    int16_t sekevence = (static_cast<int16_t>(msg.data[4]) << 8) | msg.data[5];


    std::array<float, 2> glow_plug_v = {glow_plug_1_v, glow_plug_2_v};
    std::array<float, 2> glow_plug_i = {glow_plug_1_i, glow_plug_2_i};
  
    glow_plugs_message_.glow_plug_v = glow_plug_v;
    glow_plugs_message_.glow_plug_i = glow_plug_i;
    glow_plugs_message_.sekevence = sekevence;

    glow_plugs_pub_->publish(glow_plugs_message_);


    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_glow_plugs_ = glow_plugs_message_;
    }

    RCLCPP_DEBUG(this->get_logger(), "Glow Plugs (CAN ID 0x107): GP1 Voltage: %f V, GP1 Current: %f A, GP2 Voltage: %f V, GP2 Current: %f A, Sequence: %d", 
                glow_plug_1_v, glow_plug_1_i, glow_plug_2_v, glow_plug_2_i, sekevence);
  }

private:
  void handle_ngreg_message(interfaces::msg::CanMsg msg) {
    auto ng_reg_message_ = interfaces::msg::NgReg();

    uint16_t frame_id = msg.id;
    ng_reg_message_.can_msg = msg;
    ng_reg_message_.header = create_header(frame_id);

    float integrator = static_cast<uint16_t>(msg.data[0] << 8) | msg.data[1];
    uint16_t windup = static_cast<uint16_t>(msg.data[2] << 8) | msg.data[3];
    float error = static_cast<int16_t>(msg.data[4] << 8) | msg.data[5];
    float pump_power = static_cast<uint16_t>(msg.data[6]) | msg.data[7];

    ng_reg_message_.integrator = integrator;
    ng_reg_message_.windup = windup;
    ng_reg_message_.error = error;
    ng_reg_message_.pump_power = pump_power;

    ng_reg_pub_->publish(ng_reg_message_);


    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_ng_reg_ = ng_reg_message_;
    }

    RCLCPP_DEBUG(this->get_logger(), "NGReg (CAN ID 0x108): Integrator: %f %%, Windup: %u, Error: %f %%, Pump Power: %f %%", 
                integrator, windup, error, pump_power);

  }

private:
  void handle_system_info_2_message(interfaces::msg::CanMsg msg) {
    auto system_info2_message_ = interfaces::msg::SystemInfo2();

    uint16_t frame_id = msg.id;
    system_info2_message_.can_msg = msg;
    system_info2_message_.header = create_header(frame_id);

    uint32_t ecu_hw_serial_number = (static_cast<uint32_t>(msg.data[0] << 24)) | (static_cast<uint32_t>(msg.data[1] << 16)) | (static_cast<uint32_t>(msg.data[2] << 8)) | msg.data[3];
    uint16_t eiu_sw_version = (static_cast<uint16_t>(msg.data[4] << 8)) | msg.data[5];

    system_info2_message_.ecu_hw_serial_number = ecu_hw_serial_number;
    system_info2_message_.eiu_sw_version = eiu_sw_version;

    system_info2_pub_->publish(system_info2_message_);

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_system_info2_ = system_info2_message_;
    }

    RCLCPP_DEBUG(this->get_logger(), "System Info 2 (CAN ID 0x109): ECU HW Serial Number: %u, EIU SW Version: %u", 
                ecu_hw_serial_number, eiu_sw_version);
  }

private:
  bool check_all_healthy() {
    //check for any errors that prevent operation/startup
    std::lock_guard<std::mutex> lock(state_mutex_);
    for(uint8_t bit = 0; bit < 64; bit++) {
      if((current_errors_.error_mask >> bit) & 1ULL) {
        auto it = error_map_.find(bit);
        if(it != error_map_.end() && it->second.requires_reset) {
          RCLCPP_WARN(this->get_logger(), "Error %s requires reset, cannot start engine", it->second.name.c_str());
          return false;
        }
      }
    }
    return true;
  }

//function to create a header for a message easily
private:
  std_msgs::msg::Header create_header(uint16_t frame_id) {
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = message_map_[frame_id].name;
    return header;
  }

  // Publishers for various CAN messages
  rclcpp::Publisher<interfaces::msg::EngineData>::SharedPtr engine_data_pub_;
  rclcpp::Publisher<interfaces::msg::PumpRpm>::SharedPtr engine2_data_pub_;
  rclcpp::Publisher<interfaces::msg::Errors>::SharedPtr errors_current_pub_;
  rclcpp::Publisher<interfaces::msg::FuelAmbient>::SharedPtr fuel_ambient_pub_;
  rclcpp::Publisher<interfaces::msg::GlowPlugs>::SharedPtr glow_plugs_pub_;
  rclcpp::Publisher<interfaces::msg::LastRunInfo>::SharedPtr last_run_info_pub_;
  rclcpp::Publisher<interfaces::msg::NgReg>::SharedPtr ng_reg_pub_;
  rclcpp::Publisher<interfaces::msg::Statistics>::SharedPtr statistics_pub_;
  rclcpp::Publisher<interfaces::msg::SystemInfo>::SharedPtr system_info_pub_;
  rclcpp::Publisher<interfaces::msg::SystemInfo2>::SharedPtr system_info2_pub_;
  rclcpp::Publisher<interfaces::msg::VoltageCurrent>::SharedPtr voltage_current_pub_;

  //Subscriber for throttle signal
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_cmd_sub_;

  //Subsrciber for raw can messages
  rclcpp::Subscription<interfaces::msg::CanMsg>::SharedPtr can_rx_sub_;

  //Service client for sending raw can messages
  rclcpp::Client<interfaces::srv::SendCanMessage>::SharedPtr can_tx_srv_;

  //Service for insta-kill
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr kill_service_;

  // Action servers
  rclcpp_action::Server<interfaces::action::StarterTest>::SharedPtr starter_test_action_server_;
  rclcpp_action::Server<interfaces::action::Start>::SharedPtr start_action_server_;
  rclcpp_action::Server<interfaces::action::IgniterTest>::SharedPtr igniter_test_action_server_;
  rclcpp_action::Server<interfaces::action::Prime>::SharedPtr prime_action_server_;
  rclcpp_action::Server<interfaces::action::PumpTest>::SharedPtr pump_test_action_server_;
  rclcpp_action::Server<interfaces::action::ThrottleProfile>::SharedPtr throttle_profile_action_server_;

  std::map<uint16_t, CanMessageDescriptor> message_map_;

  // current/most recent engine telemetry for use from actions
  interfaces::msg::EngineData current_engine_data_;
  interfaces::msg::PumpRpm current_pump_data_;
  interfaces::msg::Errors current_errors_;
  interfaces::msg::FuelAmbient current_fuel_ambient_;
  interfaces::msg::GlowPlugs current_glow_plugs_;
  interfaces::msg::LastRunInfo current_last_run_info_;
  interfaces::msg::NgReg current_ng_reg_;
  interfaces::msg::Statistics current_statistics_;
  interfaces::msg::SystemInfo current_system_info_;
  interfaces::msg::SystemInfo2 current_system_info2_;
  interfaces::msg::VoltageCurrent current_voltage_current_;

  //Other pieces of information relevant to engine operation
  std::atomic<bool> under_throttle_control_{false};
  std::atomic<float> throttle_cmd_{0.0};

  //timer for sending throttle control commands
  rclcpp::TimerBase::SharedPtr throttle_timer_;

  //helper for throttle profile actions
  std::vector<std::pair<rclcpp::Time, float>> throttle_profile_;


  std::map<uint8_t, std::string> state_name_map_ = {
    {0, "ENGINE OFF"},
    {1, "START - WT FOR RPM"},
    {2, "IGNITION"},
    {3, "FUEL RAMP"},
    {5, "START ARMED"},
    {7, "OFF - COOLING"},
    {8, "AUTO COOLING"},
    {9, "PRE START COOLING"},
    {10, "TAKE CONTROL"},
    {11, "RUNNING"},
    {12, "PREHEAT"},
    {15, "IGNITORS PREHEAT"},
    {17, "FUEL VALVE TEST"},
    {18, "IGNITION VALVE TEST"},
    {20, "IGNITION PLUG TEST"},
    {21, "PUMP TEST"},
    {22, "FUEL PRIMING"},
    {23, "STARTER TEST"}
  };

  struct ErrorInfo {
    std::string name;
    std::string description;
    bool requires_reset;
  };

std::map<uint8_t, ErrorInfo> error_map_ = {
  {0,  {"PreStart RPM high",           "Rotor speed too high for startup. Waiting with startup, once speed drops the start is initiated if still commanded. (Reset condition: once RPM below limit)", false}},
  {1,  {"Glow Plug 1 Interrupted",     "Glow plug 1 (direcly soldered to the PCB) circuit open. Startup continues. (Reset condition: Startup command)", true}},
  {2,  {"Glow Plug 2 Interrupted",     "Glow plug 2 (connected through babana connector) circuit open. Startup continues. (Reset condition: Startup command)", true}},
  {3,  {"Stop-All Glow Plugs Interrupted", "Startup is interrupted. (Reset condition: Startup command)", true}},
  {4,  {"Plug 1 Glowup TO",            "Timeout for ignitor 1 (direcly soldered to the PCB) to glow up, 12s. Startup continues. (Reset condition: Startup command)", true}},
  {5,  {"Plug 2 Glowup TO",            "Timeout for ignitor 2 (connected through babana connector) to glow up, 12s. Startup continues. (Reset condition: Startup command)", true}},
  {6,  {"Plugs Glowup TO",             "Timeout for both ignitors to glow up, 12s. Startup interrupted. (Reset condition: Startup command)", true}},
  {7,  {"Ignition TO 1",               "Ignition not detected before 8s. Startup continues but with higher fuel flow and ignition voltage. (Reset condition: Startup command)", true}},
  {8,  {"Ignition TO 2",               "Ignition not detected before 13s. Startup interrupted. Autocooling commanded. (Reset condition: Startup command)", true}},
  {9,  {"Ignition low RPM",            "Engine ignition and fuel flow not commanded due to too low rotor speed. (Draggy rotor or starter problem) (Reset condition: Startup command)", true}},
  {10, {"RPM sens fault",              "Currently not implemented.", false}},
  {11, {"EGT sensor Fault",            "Informative error. Communication with EGT sensor lost. Start may not be completed due to EGT value freeze at 15°C or last value.", false}},
  {12, {"EGT TC Open Circ",            "EGT TC open circuit. Since FW123 only the startup is prohibited. Engine will not be shut down due to this error. EGT will get frozen at the latest value. (Reset condition: Once valid signal for longer than 1,25s)", false}},
  {13, {"EGT TC Short Circ",           "EGT TC short circuit to GND. Informative error since FW123. In case EGT thermocouple measurements fail, the reading will be frozen at the latest value.", false}},
  {14, {"EGT protection OFF",          "EGT protections are deactivated. (due to malfunction of EGT sensor) (Reset condition: Once valid signal for longer than 1,25s)", false}},
  {15, {"T0 sens out of range",         "Inlet temperature sensor out of range. Currently not used. (Reset condition: Once valid signal)", false}},
  {16, {"p0 sens out of range",         "Ambient pressure sensor out of range. Currently not used.", false}},
  {17, {"RC sig out of range",          "Currently not implemented.", false}},
  {18, {"Safety Idle-RC fault",         "Currently not implemented.", false}},
  {19, {"Safety Stop-RC fault",         "Currently not implemented. (Reset condition: Startup command)", true}},
  {20, {"MAN circuitry open",           "Analogue control circuitry open. Engine auto shut down after 1s and error #21 triggered. (Reset condition: Once valid signal for longer than 1s)", true}},
  {21, {"Safety Stop-MAN fault",        "Engine shut down due to analogue control signal out of range longer than 1s, see error #20. Autocooling commanded. (Reset condition: Startup command)", true}},
  {22, {"CAN PLA out of range",         "CAN throttle input out of range 0..100%. Engine auto return to idle, persistency 2s. Engine auto shut down, persistency 10s. Autocooling commanded. (Reset condition: Once PLA in the range of 0..100%)", false}},
  {23, {"CAN com fault",                "CAN signal lost. Engine auto return to idle, persistency 2s. Engine auto shut down, persistency 10s. Autocooling commanded. (Reset condition: Once valid packet within intervals of at least 250ms)", false}},
  {24, {"Safety Idle-CAN fault",        "Informative error, engine returned to idle due to CAN throttle input signal out of range 0..100% for persistency longer than 2s. Autocooling commanded. (Reset condition: Once PLA in the range of 0..100%)", false}},
  {25, {"Safety Stop-CAN fault",        "Informative error, engine stopped due to CAN throttle input signal out of range 0..100% for persistency longer than 10s. Autocooling commanded. (Reset condition: Once PLA in the range of 0..100%)", false}},
  {26, {"Start Stoped-PLA high",        "N/A since FW 0.116", false}},
  {27, {"N_pump_tracking_error",        "Not implemented in current FW release (0.90)", false}},
  {28, {"Bat Voltage Low",              "Low battery voltage. Startup interrupted if below A4003 (adjustable) longer than 2s. If voltage drops below 9V (fixed limit) the engine is shut down. Autocooling commanded. (Reset condition: Startup command)", true}},
  {29, {"Bat Voltage High",             "High battery voltage over 15V. Engine shut down from any state immediately. Autocooling commanded. (Reset condition: Once voltage within limits)", true}},
  {30, {"Bendix slip",                  "High delta between starter rotor and engine rotor speed detected. During ignition and preheat the starter connection is cycled 5x, then start is interrupted and Error 31 is triggered, engine continues in cooling. During ramp phase no action is triggered, other protections take care of safety. (Reset condition: Once no slip detected)", true}},
  {31, {"Start F Bendix",               "Startup was interrupted due to starter connection fail, see error #30. Engine continues in auto cooling. (Reset condition: Startup command)", true}},
  {32, {"Flameout",                     "Combustor flameout detected. Fuel stopped and auto cooling commanded. (Reset condition: Startup command)", true}},
  {33, {"EGT start stopped",            "EGT above limit A1205 (900°C) during startup, persistency 2s. Engine is then shut down, autocooling commanded. (Reset condition: Startup command)", true}},
  {34, {"EGT run stopped",              "EGT above limit A1211 (720°C) during run, persistency 1s. Engine is then shut down, autocooling commanded. (Reset condition: Startup command)", true}},
  {35, {"Overspeed stop",               "Engine rotor speed higher than 130kRPM. Engine stopped, autocooling commanded. (Reset condition: Startup command)", true}},
  {36, {"EGT start limited",            "EGT above limit A1204 (800°C) during startup. Engine acceleration (ramp phase) reduced to decrease EGT. (Reset condition: Once within limits)", false}},
  {37, {"EGT run limited",              "EGT above limit A1209 (652°C) during startup. Rotor speed reduced to keep EGT below the limit. Rotor speed is not reduced below idle speed setting. (Reset condition: Once within limits and the limiter not active)", false}},
  {38, {"Hung Start",                   "Acceleration during ramp phase too slow. Start is interrupted, autocooling commanded. (Reset condition: Once within limits)", false}},
  {39, {"Preheat Timeout",              "Preheat time longer than 25s. Start interrupted, autocooling commanded. (Reset condition: Once within limits)", false}},
  {40, {"Ng min stop",                  "Rotor speed below 25kRPM. Engine shut down, autocooling commanded. (Reset condition: Once within limits)", false}},
  {41, {"Glow 1 Short",                 "Ignitor 1 (direcly soldered to the PCB) short circuit. Start interrupted, autocooling commanded. (Reset condition: Once within limits)", true}},
  {42, {"Glow 2 Short",                 "Ignitor 2 (connected through babana connector) short circuit. Start interrupted, autocooling commanded. (Reset condition: Once within limits)", true}},
  {43, {"Safety Idle LIN PLA fault",    "LIN bus throttle signal (ECU-EIU comm) invalid longer than limit 1. Engine return to idle. (Reset condition: Once correct LIN packets received with time limit)", false}},
  {44, {"Safety Stop LIN PLA fault",    "LIN bus throttle signal (ECU-EIU comm) invalid longer than limit 2. Engine shut down, autocooling commanded. (Reset condition: Once correct LIN packets received with time limit)", false}},
  {45, {"Fuel valve failure",           "Fuel valve control failure. Startup interrupted, autocooling commanded. During run the engine likely flames out (Error #32). (Reset condition: Once valid feedback received for longer than 1s)", false}},
  {46, {"Ignition valve failure",       "Ignition valve control failure. Start most likely will timeout in ignition or preheat phase if valve inop. (Reset condition: Once valid feedback received for longer than 1s)", false}},
  {47, {"Max Fuel Flow achieved",       "Fuel pump max power limit reached. Informative error. Usually caused by lockage of fuel filter, or another problem in the fuel installation. (Reset condition: Once fuel pump power within limits)", false}}
};

private:
  interfaces::msg::CanMsg control_off_msg_ = interfaces::msg::CanMsg();

private:
  interfaces::msg::CanMsg keep_alive_msg_ = interfaces::msg::CanMsg();

private:
  interfaces::msg::CanMsg throttle_off_msg_ = interfaces::msg::CanMsg();

private:
  interfaces::msg::CanMsg test_off_msg_ = interfaces::msg::CanMsg();

private:
  std::mutex state_mutex_; //This could be improved to have a more fine-grained locking strategy

private:
  CallbackGroup::SharedPtr action_callback_group_;
  CallbackGroup::SharedPtr throttle_processing_group_;
  CallbackGroup::SharedPtr can_processing_group_;
  CallbackGroup::SharedPtr kill_service_group_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanInterfaceNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
