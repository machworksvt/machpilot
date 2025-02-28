#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/header.hpp"
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
#include "interfaces/action/run_starter_test.hpp"

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

#define CAN_ID_BASE0 0x100
#define CAN_ID_BASE1 0x120
#define CAN_ID_BASE2 0x140

/*
  TODO:
    Can add some kind of timer/timeout for nominal message frequencies to see if data is old
    Add some kind of multi-threaded executor to ensure message reception isn't blocked by service or action calls

*/

class CANHandler {
  public:
    CANHandler(const std::string & interface_name) 
    : interface_name_(interface_name), running_(false)
    {
      //Open the CAN socket
      socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (socket_fd_ < 0) {
        throw std::runtime_error("Failed to open CAN socket");
        return;
      }
      //set non-blocking mode
      int flags = fcntl(socket_fd_, F_GETFL, 0);
      fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  
      //get interface index
      struct ifreq ifr;
      std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ);
      if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        throw std::runtime_error("Error in ioctl when getting interface index for can0");
        close(socket_fd_);
        return;
      }
  
      //bind the socket to the interface
      struct sockaddr_can addr;
      std::memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        throw std::runtime_error("Error binding socket to can0");
        close(socket_fd_);
        return;
      }
    }
  
    ~CANHandler() {
      if (socket_fd_ >= 0) {
        close(socket_fd_);
      }
    }
  
        //This function starts a dedicated thread to read from the CAN socket
    void start(std::function<void(struct can_frame)> frame_callback) {
      running_ = true;
      read_thread_ = std::thread([this, frame_callback]() {
        struct can_frame frame;
        while(running_) {
          //lock for thread-safe socket access
          {
            std::lock_guard<std::mutex> lock(mutex_);
            ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
            if (nbytes > 0) {
              frame_callback(frame);
            }
          }
          std::this_thread::sleep_for(10ms);
        }
      });
    }
  
    void stop() {
      running_ = false;
      if (read_thread_.joinable()) {
        read_thread_.join();
      }
    }
  
    ssize_t send_frame(const struct can_frame & frame) {
      std::lock_guard<std::mutex> lock(mutex_);
      return write(socket_fd_, &frame, sizeof(struct can_frame));
    }
  
    private:
      std::string interface_name_;
      int socket_fd_{-1};
      std::thread read_thread_;
      std::mutex mutex_;
      std::atomic<bool> running_;
  };
  
class CanInterfaceNode : public rclcpp::Node
{
public:
  CanInterfaceNode()
  : Node("can_interface_node")
  {
    this->declare_parameter("can_interface", "can0");

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

    this->run_starter_test_action_server_ = rclcpp_action::create_server<interfaces::action::RunStarterTest>(
      this,
      "run_starter_test",
      std::bind(&CanInterfaceNode::handle_run_starter_test_goal, this, _1, _2),
      std::bind(&CanInterfaceNode::handle_run_starter_test_cancel, this, _1),
      std::bind(&CanInterfaceNode::handle_run_starter_test_accepted, this, _1)
    );


    try {
      can_handler_ = std::make_shared<CANHandler>(this->get_parameter("can_interface").as_string());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create CANHandler: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    can_handler_->start([this](const struct can_frame & frame) {
      handle_can_message(frame);
    });

    setup_messsage_map();

    RCLCPP_INFO(this->get_logger(), "CAN interface node initialized");
  }

  ~CanInterfaceNode()
  {
    if (can_handler_) {
      can_handler_->stop();
    }
  }


  //function handler to accept a new action request
  rclcpp_action::GoalResponse handle_run_starter_test_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const interfaces::action::RunStarterTest::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to run starter test");
    // check conditions for accepting the goal
    if (current_engine_data_.state != 0) {
      RCLCPP_WARN(this->get_logger(), "Cannot run starter test while engine is running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // function to handle cancellation of a goal
  rclcpp_action::CancelResponse handle_run_starter_test_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::RunStarterTest>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel starter test");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_run_starter_test_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::RunStarterTest>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Starter test action has been accepted");
    using namespace std::placeholders;
    std::thread{std::bind(&CanInterfaceNode::execute_run_starter_test, this, _1), goal_handle}.detach();
  }

  void execute_run_starter_test(const std::shared_ptr<rclcpp_action::ServerGoalHandle<interfaces::action::RunStarterTest>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing starter test action");
    auto feedback = std::make_shared<interfaces::action::RunStarterTest::Feedback>();
    auto result = std::make_shared<interfaces::action::RunStarterTest::Result>();


    feedback->current_message = "Starting engine...";
    goal_handle->publish_feedback(feedback);

    struct can_frame run_frame;
    run_frame.can_id = CAN_ID_BASE1;
    run_frame.can_dlc = 1;
    run_frame.data[0] = 1;

    struct can_frame stop_frame;
    stop_frame.can_id = CAN_ID_BASE1;
    stop_frame.can_dlc = 1;
    stop_frame.data[0] = 0;

    std::this_thread::sleep_for(1s); //wait for a second before sending the start frame

    //repeatedly send the frame to start the test
    auto start_time = this->now();
    while (this->now() - start_time < 90s && current_engine_data_.state != 5) { //timeout after 30 seconds, or when engine reaches running state

      can_handler_->send_frame(run_frame); //repeatedly send the frame
      feedback->current_rpm = current_engine_data_.real_rpm;
      feedback->current_message = "Waiting for start completion... TIMEOUT: " + std::to_string(30 - (this->now() - start_time).seconds()) + "/30s";

      //check if we have an error on startup
      //there are certain erorrs that interuppt startup, so we check for those

      if (!check_all_healthy()) {
        result->success = false;
        result->message = "Error detected during startup, aborting";
        can_handler_->send_frame(stop_frame);
        goal_handle->succeed(result);
        return;
      }

      goal_handle->publish_feedback(feedback);

      std::this_thread::sleep_for(100ms); //recommended to keep this at 10Hz
    }

    if (current_engine_data_.state == 5) {
      RCLCPP_INFO(this->get_logger(), "Engine reached ON state succesfully. Congrats.");
      feedback->current_message = "Engine started successfully. HELL YEAH";
    } else {
      result->success = false;
      RCLCPP_INFO(this->get_logger(), "Engine startup timed out. Aborting for safety.");
      result->message = "Engine startup timed out. Aborting for safety.";
      can_handler_->send_frame(stop_frame);
      goal_handle->succeed(result);
      return;
    }

    std::this_thread::sleep_for(1s); //wait for a second before starting hold process.

    RCLCPP_INFO(this->get_logger(), "Holding engine at running state for 30 seconds");
    feedback->current_message = "Holding engine at running state for 30 seconds";
    goal_handle->publish_feedback(feedback);
    //hold at running state for 30 seconds

    auto running_time = this->now();
    while (this->now() - running_time < 30s) {
      feedback->current_message = "Holding engine at running state... TIMEOUT: " + std::to_string(30 - (this->now() - running_time).seconds()) + "/30s";
      feedback->current_rpm = current_engine_data_.real_rpm;
      goal_handle->publish_feedback(feedback);
      can_handler_->send_frame(run_frame);
      std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "Stopping engine...");
    feedback->current_message = "Stopping engine...";
    goal_handle->publish_feedback(feedback);

    can_handler_->send_frame(stop_frame);

    result->success = true;
    result->message = "Engine test completed successfully";
    goal_handle->succeed(result);
    return;
  }

  struct CanMessageDescriptor {
    uint8_t expected_dlc;
    std::string name;
    std::function<void(const struct can_frame & frame)> handler;
  };

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
  }

private:
  void handle_can_message(const struct can_frame & frame) {
    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    auto it = message_map_.find(frame_id);
    if (it != message_map_.end()) {
      if (frame.can_dlc < it->second.expected_dlc) {
        RCLCPP_WARN(this->get_logger(), "Received CAN frame (ID 0x%X) with insufficient data length: %d", frame_id, frame.can_dlc);
      } else {
        it->second.handler(frame);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Received CAN frame with unrecognized ID: 0x%X", frame_id);
    }
  }

private:
  void handle_rpm_state_message(const struct can_frame & frame) {
    auto engine_pub_msg_ = interfaces::msg::EngineData();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    engine_pub_msg_.header = create_header(frame_id);

    uint32_t set_rpm = ((static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1]) * 10;
    uint16_t real_rpm = ((static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3]) * 10;
    float egt = ((static_cast<int16_t>(frame.data[4]) << 8) | frame.data[5]) * 0.1;
    uint8_t state = frame.data[6];
    std::string state_name = state_name_map_[state];
    float pump_power = frame.data[7] * 0.5;

    engine_pub_msg_.set_rpm = set_rpm;
    engine_pub_msg_.real_rpm = real_rpm;
    engine_pub_msg_.egt = egt;
    engine_pub_msg_.state = state;
    engine_pub_msg_.state_name = state_name;
    engine_pub_msg_.pump_power = pump_power;

    engine_data_pub_->publish(engine_pub_msg_);

    current_engine_data_ = engine_pub_msg_;

    RCLCPP_DEBUG(this->get_logger(),
                "Engine Data (CAN ID 0x100): SetRPM: %u 1/min, RealRPM: %u 1/min, EGT: %.1f°C, State: %uv (%s), Pump Power: %.1f%%",
                set_rpm, real_rpm, egt, state, state_name.c_str(), pump_power);
  }

private:
  void handle_voltage_current_message(const struct can_frame & frame) {
    auto voltage_current_msg_ = interfaces::msg::VoltageCurrent();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    voltage_current_msg_.header = create_header(frame_id);

    float battery_voltage = static_cast<uint8_t>(frame.data[0]) * 0.1;
    float engine_current = static_cast<uint8_t>(frame.data[1]) * 0.2;
    uint8_t flags = static_cast<uint8_t>(frame.data[4]);

    voltage_current_msg_.battery_voltage = battery_voltage;
    voltage_current_msg_.battery_current = engine_current;
    voltage_current_msg_.flags = flags;

    voltage_current_pub_->publish(voltage_current_msg_);

    current_voltage_current_ = voltage_current_msg_;

    RCLCPP_DEBUG(this->get_logger(), "Voltage/Current Data (CAN ID 0x101): BV: %f V, EC: %f A", battery_voltage, engine_current);
  }

private:
  void handle_fuel_ambient_message(const struct can_frame & frame) {
    auto fuel_ambient_msg_ = interfaces::msg::FuelAmbient();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    fuel_ambient_msg_.header = create_header(frame_id);

    uint32_t fuel_flow = ((static_cast<uint16_t>(frame.data[0] << 8)) | frame.data[1]); //ml/min 
    uint32_t fuel_consumed = ((static_cast<uint16_t>(frame.data[2] << 8)) | frame.data[3]) * 10; //ml
    float engine_box_pressure = ((static_cast<uint16_t>(frame.data[4] << 8)) | frame.data[5]) * 0.02; //mbar
    int8_t ambient_temperature = frame.data[6];

    fuel_ambient_msg_.fuel_flow = fuel_flow;
    fuel_ambient_msg_.fuel_consumed = fuel_consumed;
    fuel_ambient_msg_.engine_box_pressure = engine_box_pressure;
    fuel_ambient_msg_.ambient_temperature = ambient_temperature;

    fuel_ambient_pub_->publish(fuel_ambient_msg_);

    current_fuel_ambient_ = fuel_ambient_msg_;

    RCLCPP_DEBUG(this->get_logger(), "Fuel/Ambient (CAN ID 0x102): Fuel Rate: %u ml/min, Fuel Consumed: %u ml, Engine Pressure: %f mbar, Ambient Temp: %d C", 
                fuel_flow, fuel_consumed, engine_box_pressure, ambient_temperature);

  }

private:
  void handle_statistics_message(const struct can_frame & frame) {
    auto statistics_msg_ = interfaces::msg::Statistics();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    statistics_msg_.header = create_header(frame_id);

    uint16_t runs_ok = ((static_cast<uint16_t>(frame.data[0] << 8)) | frame.data[1]);
    uint16_t runs_aborted = ((static_cast<uint16_t>(frame.data[2] << 8)) | frame.data[3]); 
    uint32_t total_runtime = (static_cast<uint32_t>(frame.data[4] << 24)) | (static_cast<uint32_t>(frame.data[5] << 16)) | (static_cast<uint32_t>(frame.data[6] << 8)) | frame.data[7];

    statistics_msg_.runs_ok = runs_ok;
    statistics_msg_.runs_aborted = runs_aborted;
    statistics_msg_.total_runtime = total_runtime;

    statistics_pub_->publish(statistics_msg_);

    current_statistics_ = statistics_msg_;

    RCLCPP_DEBUG(this->get_logger(), "Statistics (CAN ID 0x103): Runs OK: %u, Runs Failed: %u, Total Runtime: %u s", runs_ok, runs_aborted, total_runtime);
  }

private:
  void handle_last_run_info_message(const struct can_frame & frame) {
    auto last_run_info_msg_ = interfaces::msg::LastRunInfo();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    last_run_info_msg_.header = create_header(frame_id);

    uint16_t last_run_time = ((static_cast<uint16_t>(frame.data[0] << 8)) | frame.data[1]);
    uint16_t last_off_rpm = ((static_cast<uint16_t>(frame.data[2] << 8)) | frame.data[3]);
    uint16_t last_off_egt = ((static_cast<uint16_t>(frame.data[4] << 8)) | frame.data[5]);
    float last_off_pump_power = static_cast<uint8_t>(frame.data[6]) * 0.5;
    uint8_t last_off_state = frame.data[7];
    std::string last_off_state_str = state_name_map_[last_off_state];
    
    last_run_info_msg_.last_runtime = last_run_time;
    last_run_info_msg_.last_off_rpm = last_off_rpm;
    last_run_info_msg_.last_off_egt = last_off_egt;
    last_run_info_msg_.last_off_pump_power = last_off_pump_power;
    last_run_info_msg_.last_off_state = last_off_state;
    last_run_info_msg_.last_off_state_str = last_off_state_str;

    last_run_info_pub_->publish(last_run_info_msg_);

    current_last_run_info_ = last_run_info_msg_;

    RCLCPP_DEBUG(this->get_logger(), "Last Run Info: (CAN ID 0x104), Last Runtime: %u s, Last Off RPM: %u 1/min, Last Off EGT: %u, Last Off Pump Power: %.1f%%, Last Off State: %u (%s)", 
                last_run_time, last_off_rpm, last_off_egt, last_off_pump_power, last_off_state, last_off_state_str.c_str());
  }

private:
  void handle_system_info_message(const struct can_frame & frame) {
    auto system_info_message_ = interfaces::msg::SystemInfo();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    system_info_message_.header = create_header(frame_id);

    uint16_t serial_number = ((static_cast<uint16_t>(frame.data[0] << 8)) | frame.data[1]);
    uint16_t fw_version = ((static_cast<uint16_t>(frame.data[2] << 8)) | frame.data[3]);
    uint8_t engine_type = frame.data[4];
    uint8_t engine_subtype = frame.data[5];

    system_info_message_.serial_number = serial_number;
    system_info_message_.fw_version = fw_version;
    system_info_message_.engine_type = engine_type;
    system_info_message_.engine_subtype = engine_subtype;

    system_info_pub_->publish(system_info_message_);

    current_system_info_ = system_info_message_;

    RCLCPP_DEBUG(this->get_logger(), "System Info (CAN ID 0x105): Serial Number: %u, FW Version: %u, Engine Type: %u, Engine Subtype: %u", 
                serial_number, fw_version, engine_type, engine_subtype);
  }

private:
  void handle_pump_rpm_message(const struct can_frame & frame) {
    auto pump_rpm_message_ = interfaces::msg::PumpRpm();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    pump_rpm_message_.header = create_header(frame_id);

    uint32_t pump_rpm = static_cast<uint16_t>(frame.data[0] << 8) | frame.data[1];

    pump_rpm_message_.pump_rpm = pump_rpm;

    engine2_data_pub_->publish(pump_rpm_message_);

    current_pump_data_ = pump_rpm_message_;

    RCLCPP_DEBUG(this->get_logger(), "Pump RPM (CAN ID 0x106): Pump RPM: %u 1/min", pump_rpm);
  }
  
private:
  void handle_errors_message(const struct can_frame & frame) {
    auto errors_message_ = interfaces::msg::Errors();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    errors_message_.header = create_header(frame_id);

    uint64_t error_mask = (static_cast<uint64_t>(frame.data[0]) << 56) | (static_cast<uint64_t>(frame.data[1]) << 48) | (static_cast<uint64_t>(frame.data[2]) << 40) | (static_cast<uint64_t>(frame.data[3]) << 32) | (static_cast<uint64_t>(frame.data[4]) << 24) | (static_cast<uint64_t>(frame.data[5]) << 16) | (static_cast<uint64_t>(frame.data[6]) << 8) | frame.data[7];

    std::vector<std::string> error_messages;

    bool reset_required = false;

    for (uint8_t bit = 0; bit < 64; ++bit) {
      // Check if the bit is set.
      if ((error_mask >> bit) & 1ULL) {
        // Look up the error in the map.
        auto it = error_map_.find(bit);
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

    current_errors_ = errors_message_;

    RCLCPP_WARN(this->get_logger(), "Errors (CAN ID 0x107): Error Mask: 0x%016lX", error_mask);
    for (const auto & err : error_messages) {
      RCLCPP_WARN(this->get_logger(), "  %s", err.c_str());
    }
  }

private:
  void handle_glow_plugs_message(const struct can_frame & frame) {
    auto glow_plugs_message_ = interfaces::msg::GlowPlugs();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    glow_plugs_message_.header = create_header(frame_id);

    float glow_plug_1_v = static_cast<uint8_t>(frame.data[0]) * 0.1;
    float glow_plug_1_i = static_cast<uint8_t>(frame.data[1]) * 0.1;
    float glow_plug_2_v = static_cast<uint8_t>(frame.data[2]) * 0.1;
    float glow_plug_2_i = static_cast<uint8_t>(frame.data[3]) * 0.1;
    int16_t sekevence = (static_cast<int16_t>(frame.data[4]) << 8) | frame.data[5];


    std::array<float, 2> glow_plug_v = {glow_plug_1_v, glow_plug_2_v};
    std::array<float, 2> glow_plug_i = {glow_plug_1_i, glow_plug_2_i};
  
    glow_plugs_message_.glow_plug_v = glow_plug_v;
    glow_plugs_message_.glow_plug_i = glow_plug_i;
    glow_plugs_message_.sekevence = sekevence;

    glow_plugs_pub_->publish(glow_plugs_message_);

    current_glow_plugs_ = glow_plugs_message_;

    RCLCPP_DEBUG(this->get_logger(), "Glow Plugs (CAN ID 0x107): GP1 Voltage: %f V, GP1 Current: %f A, GP2 Voltage: %f V, GP2 Current: %f A, Sequence: %d", 
                glow_plug_1_v, glow_plug_1_i, glow_plug_2_v, glow_plug_2_i, sekevence);
  }

private:
  void handle_ngreg_message(const struct can_frame & frame) {
    auto ng_reg_message_ = interfaces::msg::NgReg();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    ng_reg_message_.header = create_header(frame_id);

    float integrator = static_cast<uint16_t>(frame.data[0] << 8) | frame.data[1];
    uint16_t windup = static_cast<uint16_t>(frame.data[2] << 8) | frame.data[3];
    float error = static_cast<int16_t>(frame.data[4] << 8) | frame.data[5];
    float pump_power = static_cast<uint16_t>(frame.data[6]) | frame.data[7];

    ng_reg_message_.integrator = integrator;
    ng_reg_message_.windup = windup;
    ng_reg_message_.error = error;
    ng_reg_message_.pump_power = pump_power;

    ng_reg_pub_->publish(ng_reg_message_);

    current_ng_reg_ = ng_reg_message_;

    RCLCPP_DEBUG(this->get_logger(), "NGReg (CAN ID 0x108): Integrator: %f %%, Windup: %u, Error: %f %%, Pump Power: %f %%", 
                integrator, windup, error, pump_power);

  }

private:
  void handle_system_info_2_message(const struct can_frame & frame) {
    auto system_info2_message_ = interfaces::msg::SystemInfo2();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    system_info2_message_.header = create_header(frame_id);

    uint32_t ecu_hw_serial_number = (static_cast<uint32_t>(frame.data[0] << 24)) | (static_cast<uint32_t>(frame.data[1] << 16)) | (static_cast<uint32_t>(frame.data[2] << 8)) | frame.data[3];
    uint16_t eiu_sw_version = (static_cast<uint16_t>(frame.data[4] << 8)) | frame.data[5];

    system_info2_message_.ecu_hw_serial_number = ecu_hw_serial_number;
    system_info2_message_.eiu_sw_version = eiu_sw_version;

    system_info2_pub_->publish(system_info2_message_);

    current_system_info2_ = system_info2_message_;

    RCLCPP_DEBUG(this->get_logger(), "System Info 2 (CAN ID 0x109): ECU HW Serial Number: %u, EIU SW Version: %u", 
                ecu_hw_serial_number, eiu_sw_version);
  }

private:
  bool check_all_healthy() {
    //check for any errors that prevent operation/startup
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

  // Low-level class to thread-safe handle CAN reception and transmission
  std::shared_ptr<CANHandler> can_handler_;

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

  // Action servers
  rclcpp_action::Server<interfaces::action::RunStarterTest>::SharedPtr run_starter_test_action_server_;

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

  /*
    TODO: Translate error descriptors from the page in the excel spreadsheet into the associated pieces of information
    Add graceful handling of these errors, using their reset conditions, etc.
  */
};

/*
class CanInterfaceNode : public rclcpp::Node
{
public:
  CanInterfaceNode()
  : Node("h20pro_canio")
  {  
    starter_service_ = this->create_service<std_srvs::srv::Trigger>("starter_test", std::bind(&CanInterfaceNode::run_starter_test, this, _1, _2, _3));
    enable_service_ = this->create_service<std_srvs::srv::Trigger>("enable_can", std::bind(&CanInterfaceNode::send_enable_command, this, _1, _2, _3));

    engine_data_pub_ = this->create_publisher<interfaces::msg::EngineData>("engine_data", 10);

    // Open a socket for CAN communications
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open CAN socket");
      return;
    }

    // Set the socket to non-blocking mode so our timer callback does not block
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    // Specify the CAN interface ("can0")
    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error in ioctl when getting interface index for can0");
      close(socket_fd_);
      return;
    }

    // Bind the socket to can0
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error binding socket to can0");
      close(socket_fd_);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully opened and bound CAN socket on can0");

    // Set up a timer that polls for CAN messages at regular intervals (100ms)
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&CanInterfaceNode::timer_callback, this)
    );
  }

  ~CanInterfaceNode()
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

  int socket_fd_{-1};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr starter_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;

  rclcpp::Publisher<interfaces::msg::EngineData>::SharedPtr engine_data_pub_;

  private: void timer_callback()
  {
    // Structure to hold a CAN frame
    struct can_frame frame;
    // Read as many frames as are available (non-blocking read)
    ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
    while (nbytes > 0) {
      // We use CAN_SFF_MASK to extract the standard 11-bit identifier.
      // According to the documentation, engine data is sent using:
      //   CAN_ID_BASE0 + 0, where CAN_ID_BASE0 = 0x100.
      if ((frame.can_id & CAN_SFF_MASK) == CAN_ID_BASE0) {
        // Check that the frame has 8 data bytes.
        if (frame.can_dlc < 8) {
          RCLCPP_WARN(this->get_logger(), "Received CAN frame (ID 0x100) with insufficient data length: %d", frame.can_dlc);
        } else {
          // Decode the data per the manufacturer's specification:
          //   - Bytes 0/1: Engine Set RPM (unsigned int; high byte is at lower index)
          //   - Bytes 2/3: Engine Real RPM (unsigned int)
          //   - Bytes 4/5: EGT (Exhaust Gas Temperature, signed int, in 0.1 °C units)
          //   - Byte 6: Engine State (unsigned char)
          //   - Byte 7: Pump Power (unsigned char, in steps of 0.5%)
          uint16_t set_rpm_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
          uint16_t real_rpm_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];

          // For signed 16-bit values, the cast ensures proper sign extension.
          int16_t egt_raw = (static_cast<int16_t>(frame.data[4]) << 8) | frame.data[5];

          uint8_t state = frame.data[6];
          uint8_t pump_power_raw = frame.data[7];

          // Convert the raw values to final values:
          //   - RPM values are in multiples of 10 (multiply by 10).
          //   - EGT is in 0.1 °C units (divide by 10 to get °C).
          //   - Pump Power is in multiples of 0.5% (multiply by 0.5).
          uint32_t set_rpm  = set_rpm_raw * 10;
          uint32_t real_rpm = real_rpm_raw * 10;
          float egt         = egt_raw / 10.0f;
          float pump_power  = pump_power_raw * 0.5f;

          RCLCPP_INFO(this->get_logger(),
                      "Engine Data (CAN ID 0x100): SetRPM: %u 1/min, RealRPM: %u 1/min, EGT: %.1f°C, State: %u, Pump Power: %.1f%%",
                      set_rpm, real_rpm, egt, state, pump_power);
        }
      } else if ((frame.can_id & CAN_SFF_MASK) == (CAN_ID_BASE0 + 1)) {
        //Voltage/current message
        if (frame.can_dlc < 2) {
          RCLCPP_WARN(this->get_logger(), "Received CAN frame (ID 0x101) with insufficient data length: %d", frame.can_dlc);
        } else {
          float battery_voltage = static_cast<uint8_t>(frame.data[0]) * 0.1;
          float engine_current = static_cast<uint8_t>(frame.data[1]) * 0.2;
          RCLCPP_DEBUG(this->get_logger(), "Voltage/Current Data (CAN ID 0x101): BV: %f V, EC: %f A", battery_voltage, engine_current);
        }
      } else if((frame.can_id & CAN_SFF_MASK) == (CAN_ID_BASE0 + 2)) {
        if (frame.can_dlc < 7) {
          RCLCPP_WARN(this->get_logger(), "Received CAN frame (ID 0x102) with insufficient data length: %d", frame.can_dlc);
        } else {
          uint16_t fuel_flow = ((static_cast<uint16_t>(frame.data[0] << 8)) | frame.data[1]); //ml/min 
          uint16_t fuel_consumed = ((static_cast<uint16_t>(frame.data[2] << 8)) | frame.data[3]); //ml
          float engine_pressure = ((static_cast<uint16_t>(frame.data[4] << 8)) | frame.data[5]) * 0.02; //mbar
          int8_t ambient_temperature = frame.data[6];

          RCLCPP_INFO(this->get_logger(), "Combustion Information: Fuel Rate: %u ml/min, Fuel Consumed: %u ml, Engine Pressure: %f mbar, Ambient Temp: %d C", 
                      fuel_flow, fuel_consumed, engine_pressure, ambient_temperature);
        }
      } else if((frame.can_id & CAN_SFF_MASK) == (CAN_ID_BASE0 + 3)) {
        // statistics message
        if (frame.can_dlc < 8) {
          RCLCPP_WARN(this->get_logger(), "Received CAN frame (ID 0x103) with insufficient data length: %d", frame.can_dlc);
        } else {
          uint16_t runs_ok = ((static_cast<uint16_t>(frame.data[0] << 8)) | frame.data[1]);
          uint16_t runs_aborted = ((static_cast<uint16_t>(frame.data[2] << 8)) | frame.data[3]); //ml
          uint32_t total_runtime = (static_cast<uint32_t>(frame.data[4] << 24)) | (static_cast<uint32_t>(frame.data[5] << 16)) | (static_cast<uint32_t>(frame.data[6] << 8)) | frame.data[7];

          RCLCPP_INFO(this->get_logger(), "Statistics: Runs OK: %u, Runs Failed: %u, Total Runtime: %u s", runs_ok, runs_aborted, total_runtime);
        }
      } else if((frame.can_id & CAN_SFF_MASK) == (CAN_ID_BASE0 + 4)) // Last run info message
      {
        if(frame.can_dlc < 8) {
          RCLCPP_WARN(this->get_logger(), "Recieved CAN frame (ID 0x104) with insufficient data length: %d", frame.can_dlc);
        } else {

        }

      }
      else {

        // You can add additional processing for other CAN IDs here.
        //RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: 0x%X", frame.can_id & CAN_SFF_MASK);
      }

      // Attempt to read the next frame.
      nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
    }
    // If no frame is available, read() returns -1 with errno set to EAGAIN/EWOULDBLOCK (which is fine for non-blocking mode).
  }

private: void send_enable_command(std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
              const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<std_srvs::srv::Trigger::Request> request) {

    std_srvs::srv::Trigger::Response response;

    //set user access level
    struct can_frame frame1;
    std::memset(&frame1, 0, sizeof(frame1));
    frame1.can_id = CAN_ID_BASE2 + 0;
    frame1.can_dlc = 6;

    frame1.data[0] = 20;
    frame1.data[1] = 39;
    frame1.data[2] = 41;
    frame1.data[3] = 241;
    frame1.data[4] = 34;
    frame1.data[5] = 150;

    int nbytes = write(socket_fd_, &frame1, sizeof(frame1));
    if (nbytes != sizeof(frame1)) {
      RCLCPP_WARN(this->get_logger(), "Failed to send CAN access level command");
      response.success = false;
      response.message = "Couldn't send access level command";
    } else {
      RCLCPP_INFO(this->get_logger(), "Starter access level command sent.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    struct can_frame frame2;
    std::memset(&frame2, 0, sizeof(frame2));
    frame2.can_id = CAN_ID_BASE2 + 0;
    frame2.can_dlc = 3;

    frame2.data[0] = 185;
    frame2.data[1] = 11;
    frame2.data[2] = 5;

    nbytes = write(socket_fd_, &frame2, sizeof(frame2));
    if (nbytes != sizeof(frame2)) {
      RCLCPP_WARN(this->get_logger(), "Failed to set control mode to CAN");
      response.success = false;
      response.message = "Failed to set control mode to CAN";
    } else {
      RCLCPP_INFO(this->get_logger(), "CAN control mode command sent.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    struct can_frame frame3;
    std::memset(&frame3, 0, sizeof(frame3));
    frame3.can_id = CAN_ID_BASE2 + 0;
    frame3.can_dlc = 7;
    frame3.data[0] = 16;
    frame3.data[1] = 39;
    frame3.data[3] = 188;
    frame3.data[4] = 245;
    frame3.data[5] = 166;
    frame3.data[6] = 37;

    nbytes = write(socket_fd_, &frame3, sizeof(frame3));
    if (nbytes != sizeof(frame3)) {
      RCLCPP_WARN(this->get_logger(), "Failed EEPROM store command");
      response.success = false;
      response.message = "Failed EEPROM store command";
    } else {
      RCLCPP_INFO(this->get_logger(), "EEPROM store command sent.");
      response.success = true;
      response.message = "All commands sent.";
    }

    service->send_response(*request_header, response);
  }

private: void run_starter_test(std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service,
              const std::shared_ptr<rmw_request_id_t> request_header,
              const std::shared_ptr<std_srvs::srv::Trigger::Request> request) {

    auto finish = std::chrono::system_clock::now() + 15s;
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = CAN_ID_BASE1 + 4;
    frame.can_dlc = 7;

    frame.data[0] = 0;
    frame.data[1] = 0;
    frame.data[2] = 0;
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 5;
    frame.data[6] = 0;

    std_srvs::srv::Trigger::Response response;
    do {

    int nbytes = write(socket_fd_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
      RCLCPP_WARN(this->get_logger(), "Error sending CAN frame");
      response.success = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Starter test command sent.");
      response.success = true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));


    } while (std::chrono::system_clock::now() < finish);

    frame.data[0] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;

    int nbytes = write(socket_fd_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
      RCLCPP_WARN(this->get_logger(), "Error sending CAN frame");
      response.success = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Starter test command sent.");
      response.success = true;
    }

    service->send_response(*request_header, response);
  }

  struct MessageDescriptor {
    uint8_t dlc;
    std::string name;
    uint8_t nom_freq; //Hz
    //associated ros2 message
    //pointer to function that processes frame into ros2 message
    void (*handler)(struct can_frame frame); //tried to make this more efficient by using a pointer to the correct publisher & function pointer with ros2 msg return type :(
  };

  const std::map<uint16_t, MessageDescriptor> message_sizes = {
    {CAN_ID_BASE0, {8, "RPM/State", 10}},
    {CAN_ID_BASE0 + 1, {5, "Voltage/Current", 10}},
    {CAN_ID_BASE0 + 2, {7, "Fuel/Ambient", 5}},
    {CAN_ID_BASE0 + 3, {8, "Statistics", 1}},
    {CAN_ID_BASE0 + 4, {8, "Last Run Info", 1}},
    {CAN_ID_BASE0 + 5, {6, "System Info", 1}},
    {CAN_ID_BASE0 + 6, {2, "Pump RPM", 10}},
    {CAN_ID_BASE0 + 7, {7, "Errors", 10}},
    {CAN_ID_BASE0 + 8, {6, "Glow Plugs", 10}}, 
    {CAN_ID_BASE0 + 9, {8, "NgReg", 10}}, 
    {CAN_ID_BASE0 + 10, {8, "System Info 2", 1}}
  };


private: bool handle_can_message(struct can_frame frame) {
    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    rclcpp::Time timestamp = this->now();

    //Check if this is a message that's recognized
    struct CanInterfaceNode::MessageDescriptor descriptor;

    try {
      descriptor = message_sizes.at(frame_id);
    } catch (const std::out_of_range& e) {
      RCLCPP_WARN(this->get_logger(), "CAN Message recevied with unrecognized frame: (%x), dlc = %d", frame_id, frame.can_dlc);
      return false;
    }

    //Check if the message is the right size
    if (frame.can_dlc < descriptor.dlc) {
      RCLCPP_WARN(this->get_logger(), "CAN Message (%x) %s received with dlc (%d), expected (%d)", frame_id, descriptor.name.c_str(), frame.can_dlc, descriptor.dlc);
      return false;
    }
    // now actually process the message

    switch(frame_id) {
      case CAN_ID_BASE0:
        // RPM/State message
        uint16_t set_rpm_raw = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
        uint16_t real_rpm_raw = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
        int16_t egt_raw = (static_cast<int16_t>(frame.data[4]) << 8) | frame.data[5];
        uint8_t state = frame.data[6];
        uint8_t pump_power_raw = frame.data[7];

        uint32_t set_rpm  = set_rpm_raw * 10;
        uint32_t real_rpm = real_rpm_raw * 10;
        float egt         = egt_raw / 10.0f;
        float pump_power  = pump_power_raw * 0.5f;

        auto engine_pub_msg_ = interfaces::msg::EngineData();
        engine_pub_msg_.header.stamp = timestamp;
        engine_pub_msg_.header.frame_id = "engine_frame";

        engine_pub_msg_.set_rpm = set_rpm;
        engine_pub_msg_.real_rpm = real_rpm;
        engine_pub_msg_.egt = egt;
        engine_pub_msg_.state = state;
        engine_pub_msg_.pump_power = pump_power;

        RCLCPP_INFO(this->get_logger(),
                    "Engine Data (CAN ID 0x100): SetRPM: %u 1/min, RealRPM: %u 1/min, EGT: %.1f°C, State: %u, Pump Power: %.1f%%",
                    set_rpm, real_rpm, egt, state, pump_power);
        break;
      case CAN_ID_BASE0 + 1:
        //Voltage/current message
        float battery_voltage = static_cast<uint8_t>(frame.data[0]) * 0.1;
        float engine_current = static_cast<uint8_t>(frame.data[1]) * 0.2;
        RCLCPP_DEBUG(this->get_logger(), "Voltage/Current Data (CAN ID 0x101): BV: %f V, EC: %f A", battery_voltage, engine_current);
        break;
    }
    
}



  /*
    function to process a CAN message
      handlers for each message type then perform the publishing

    actions to run various tests

    services to change settings/parameters
  
  
  




};

*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
