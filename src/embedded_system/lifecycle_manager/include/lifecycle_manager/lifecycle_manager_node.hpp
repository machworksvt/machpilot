#include "lifecycle_interface.hpp"

#include <std_srvs/srv/trigger.hpp>
#include <bitset>

#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>

constexpr uint SERVICE_TIMEOUT_MS = 10000;
constexpr uint MAX_NODES = 64;
constexpr uint STATE_CHANGE_TIMEOUT_MS = 10000;

using namespace rclcpp_lifecycle::node_interfaces;



class LifecycleManagerNode : public Device
{
public:

LifecycleManagerNode();
~LifecycleManagerNode();

private:
    bool __callback_routine(uint8_t state, uint8_t transition, bool is_fallback);

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

    int client_response(rclcpp::FutureReturnCode res);
    int loop_change_state_clients(uint8_t transition);
    int loop_get_state_clients(uint8_t state);
    int scan_and_add_devices();

    std::shared_ptr<rclcpp::Executor> srvs_exec_;
    std::thread srvs_exec_thread_;

    std::bitset<MAX_NODES> success_flags_{0x0000};

    rclcpp::CallbackGroup::SharedPtr cbg_{nullptr};

    std::vector<std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>>> client_get_state_;
    std::vector<std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> client_change_state_;
};
