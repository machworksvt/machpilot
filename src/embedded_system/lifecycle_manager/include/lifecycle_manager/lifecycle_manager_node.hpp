#include "lifecycle_interface.hpp"

#include <std_srvs/srv/trigger.hpp>

#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>

#define STATE_CHANGE_TIMEOUT_MS 1000

using namespace rclcpp_lifecycle::node_interfaces;



class LifecycleManagerNode : public Device
{
public:

LifecycleManagerNode();
~LifecycleManagerNode();

private:
    

    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &state) override;

    int client_response(rclcpp::FutureReturnCode res);
    int loop_through_clients(uint8_t transition);
    int scan_and_add_devices();

    int device_count_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_[16];
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_[16];
};
