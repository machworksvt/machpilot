#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/header.hpp"
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

class CanInterfaceNode : public rclcpp::Node
{
public:
  CanInterfaceNode()
  : Node("can_interface_node")
  {
    engine_data_pub_ = this->create_publisher<interfaces::msg::EngineData>("/h2pro/engine_data", 10);
    engine2_data_pub_ = this->create_publisher<interfaces::msg::PumpRpm>("/h2pro/engine_data2", 10);
    errors_current_pub_ = this->create_publisher<interfaces::msg::Errors>("/h2pro/errors", 10);
    fuel_ambient_pub_ = this->create_publisher<interfaces::msg::FuelAmbient>("/h2pro/fuel_ambient", 10);
    glow_plugs_pub_ = this->create_publisher<interfaces::msg::GlowPlugs>("/h2pro/glow_plugs", 10);
    last_run_info_pub_ = this->create_publisher<interfaces::msg::LastRunInfo>("/h2pro/last_run_info", 10);
    ng_reg_pub_ = this->create_publisher<interfaces::msg::NgReg>("/h2pro/ng_reg", 10);
    statistics_pub_ = this->create_publisher<interfaces::msg::Statistics>("/h2pro/statistics", 10);
    system_info_pub_ = this->create_publisher<interfaces::msg::SystemInfo>("/h2pro/system_info", 10);
    system_info2_pub_ = this->create_publisher<interfaces::msg::SystemInfo2>("/h2pro/system_info2", 10);
    voltage_current_pub_ = this->create_publisher<interfaces::msg::VoltageCurrent>("/h2pro/voltage_current", 10);

    try {
      can_handler_ = std::make_shared<CANHandler>("can0");
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
    uint8_t pump_power = frame.data[7] * 0.5;

    engine_pub_msg_.set_rpm = set_rpm;
    engine_pub_msg_.real_rpm = real_rpm;
    engine_pub_msg_.egt = egt;
    engine_pub_msg_.state = state;
    engine_pub_msg_.pump_power = pump_power;

    engine_data_pub_->publish(engine_pub_msg_);

    RCLCPP_DEBUG(this->get_logger(),
                "Engine Data (CAN ID 0x100): SetRPM: %u 1/min, RealRPM: %u 1/min, EGT: %.1f°C, State: %u, Pump Power: %.1f%%",
                set_rpm, real_rpm, egt, state, pump_power);
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

    RCLCPP_INFO(this->get_logger(), "Combustion Information: Fuel Rate: %u ml/min, Fuel Consumed: %u ml, Engine Pressure: %f mbar, Ambient Temp: %d C", 
                fuel_flow, fuel_consumed, engine_box_pressure, ambient_temperature);

  }

private:
  void handle_statistics_message(const struct can_frame & frame) {
    auto statistics_msg_ = interfaces::msg::Statistics();

    uint16_t frame_id = (frame.can_id & CAN_SFF_MASK);
    statistics_msg_.header = create_header(frame_id);

    uint16_t runs_ok = ((static_cast<uint16_t>(frame.data[0] << 8)) | frame.data[1]);
    uint16_t runs_aborted = ((static_cast<uint16_t>(frame.data[2] << 8)) | frame.data[3]); //ml
    uint32_t total_runtime = (static_cast<uint32_t>(frame.data[4] << 24)) | (static_cast<uint32_t>(frame.data[5] << 16)) | (static_cast<uint32_t>(frame.data[6] << 8)) | frame.data[7];

    statistics_msg_.runs_ok = runs_ok;
    statistics_msg_.runs_aborted = runs_aborted;
    statistics_msg_.total_runtime = total_runtime;

    statistics_pub_->publish(statistics_msg_);

    RCLCPP_INFO(this->get_logger(), "Statistics: Runs OK: %u, Runs Failed: %u, Total Runtime: %u s", runs_ok, runs_aborted, total_runtime);
  }

private:
  void handle_last_run_info_message(const struct can_frame & frame) {
  }

private:
  void handle_system_info_message(const struct can_frame & frame) {
  }

private:
  void handle_pump_rpm_message(const struct can_frame & frame) {
  }
  
private:
  void handle_errors_message(const struct can_frame & frame) {
  }

private:
  void handle_glow_plugs_message(const struct can_frame & frame) {
  }

private:
  void handle_ngreg_message(const struct can_frame & frame) {
  }

private:
  void handle_system_info_2_message(const struct can_frame & frame) {
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

  std::map<uint16_t, CanMessageDescriptor> message_map_;

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
