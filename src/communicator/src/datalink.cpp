#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mavsdk/mavsdk.h>
#include <mavsdk/connection_result.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/param_server/param_server.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry_server/telemetry_server.h>
#include <mavsdk/plugins/action_server/action_server.h>
#include <mavsdk/plugins/mission_raw_server/mission_raw_server.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <interfaces/action/run_starter_test.hpp>
#include <interfaces/msg/engine_data.hpp>
#include <interfaces/msg/pump_rpm.hpp>
#include <interfaces/msg/errors.hpp>
#include <interfaces/msg/fuel_ambient.hpp>
#include <interfaces/msg/glow_plugs.hpp>
#include <interfaces/msg/last_run_info.hpp>
#include <interfaces/msg/ng_reg.hpp>
#include <interfaces/msg/statistics.hpp>
#include <interfaces/msg/system_info.hpp>
#include <interfaces/msg/system_info2.hpp>
#include <interfaces/msg/voltage_current.hpp>


using namespace std::chrono_literals;
using namespace mavsdk;
using namespace rclcpp;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DataLinkNode : public rclcpp::Node
{
  public:
    DataLinkNode()
    : Node("datalink")
    {
      //declare parameter for path to the serial device
      this->declare_parameter("connection_path", "serial:///dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A10O8MJG-if00-port0:57600"); //defaults to what I'm pretty sure it is

      // Setup mavlink connection
      ConnectionResult connection_result = mavsdk_.add_any_connection(this->get_parameter("connection_path").as_string());
      if (connection_result != ConnectionResult::Success) {
        RCLCPP_ERROR(this->get_logger(), "Connection failed.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Connection succeeded. Waiting for ground station detection.");
      // subscribe to system found event
      mavsdk_.subscribe_on_new_system(std::bind(&DataLinkNode::system_found_callback, this));
    }

  public:
    ~DataLinkNode() noexcept override = default;

  private:
    void system_found_callback() {
      RCLCPP_INFO(this->get_logger(), "System detected. Verifying it is a ground station.");
      link_setup(mavsdk_.systems().at(0));
    }

  private:
    void link_setup(const std::shared_ptr<System>& system) {
      RCLCPP_INFO(this->get_logger(), "Setting up link to ground station.");
      telemetry_server_ = std::make_shared<TelemetryServer>(system);
      action_server_ = std::make_shared<ActionServer>(system);
      param_server_ = std::make_shared<ParamServer>(system);
      RCLCPP_INFO(this->get_logger(), "Telemetry, parameter and action servers setup.");
      
      // ROS2 - subscribe to all relevant telemetry topics

      engine_data_sub_ = create_subscription<interfaces::msg::EngineData>(
        "/h20pro/engine_data", 10, [this](const interfaces::msg::EngineData::SharedPtr msg) {
          //lambda message repeater
          
        });

      engine_starter_test_client_ = create_client<interfaces::action::RunStarterTest>("/h20pro/run_starter_test");
    }
    
    Mavsdk mavsdk_{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::Autopilot}};

    std::shared_ptr<mavsdk::TelemetryServer> telemetry_server_;
    std::shared_ptr<mavsdk::ActionServer> action_server_;
    std::shared_ptr<mavsdk::ParamServer> param_server_;

    rclcpp::Client<interfaces::action::RunStarterTest>::SharedPtr engine_starter_test_client_; //action client for running the engine starter test

    rclcpp::Subscription<interfaces::msg::EngineData>::SharedPtr engine_data_sub_;
    rclcpp::Subscription<interfaces::msg::PumpRpm>::SharedPtr engine2_data_sub_;
    rclcpp::Subscription<interfaces::msg::Errors>::SharedPtr errors_current_sub_;
    rclcpp::Subscription<interfaces::msg::FuelAmbient>::SharedPtr fuel_ambient_sub_;
    rclcpp::Subscription<interfaces::msg::GlowPlugs>::SharedPtr glow_plugs_sub_;
    rclcpp::Subscription<interfaces::msg::LastRunInfo>::SharedPtr last_run_info_sub_;
    rclcpp::Subscription<interfaces::msg::NgReg>::SharedPtr ng_reg_sub_;
    rclcpp::Subscription<interfaces::msg::Statistics>::SharedPtr statistics_sub_;
    rclcpp::Subscription<interfaces::msg::SystemInfo>::SharedPtr system_info_sub_;
    rclcpp::Subscription<interfaces::msg::SystemInfo2>::SharedPtr system_info2_sub_;
    rclcpp::Subscription<interfaces::msg::VoltageCurrent>::SharedPtr voltage_current_sub_;

  };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DataLinkNode>());
  rclcpp::shutdown();
  return 0;
}