#include "lifecycle_manager_node.hpp"

#include <cstdlib>

using std::placeholders::_1;



LifecycleManagerNode::LifecycleManagerNode() : Device("lifecycle_manager_node")
{
    if (this->scan_and_add_devices()) {
        RCLCPP_ERROR(this->get_logger(), "Error scanning and adding devices");
    }

    this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

LifecycleManagerNode::~LifecycleManagerNode()
{

}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    int erc = loop_through_clients(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    if (erc == -1) {
        RCLCPP_INFO(this->get_logger(), "Not all transitions successful");
        return CallbackReturn::FAILURE;
    }

    if (erc == -2) {
        RCLCPP_INFO(this->get_logger(), "Error in client loop");
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully sent transitions");
    
    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_activate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    // Implementation of activation logic goes here

    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    // Implementation of deactivation logic goes here

    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    // Implementation of cleanup logic goes here

    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    // Implementation of shutdown logic goes here

    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    // Implementation of error handling logic goes here

    return CallbackReturn::SUCCESS;
}

int LifecycleManagerNode::scan_and_add_devices()
{
    // Not sure this will work in realtime context, may need to pool allocate services
    auto services = get_service_names_and_types();

    device_count_ = 0;

    for (const auto& [key, value] : services) {

        if (key.find("change_state") == std::string::npos && key.find("get_state") == std::string::npos) {
            continue;
        }

        if (key.find(this->get_name()) != std::string::npos) {
            // Skip own services, will be handled with trigger_transition() calls
            RCLCPP_INFO(get_logger(), "Found own service: %s", key.c_str());
            continue;
        }

        // #TODO: test whether the device services are added correctly, doing this differently would be better
        if (key.find("change_state") != std::string::npos) {
            client_change_state_[device_count_] = this->create_client<lifecycle_msgs::srv::ChangeState>(key);
            RCLCPP_INFO(get_logger(), "Added ChangeState client for service: %s", key.c_str());
        }

        if (key.find("get_state") != std::string::npos) {
            client_get_state_[device_count_] = this->create_client<lifecycle_msgs::srv::GetState>(key);
            RCLCPP_INFO(get_logger(), "Added GetState client for service: %s", key.c_str());
            device_count_++;
        }

    }

    if (device_count_ == 0) {
        RCLCPP_WARN(get_logger(), "No lifecycle devices found");
        return -1;
    }

    return 0;
}

int LifecycleManagerNode::loop_through_clients(uint8_t transition)
{
    uint32_t success_flags = 0;

    for (int i = 0; i < device_count_; i++) {
        auto client = client_change_state_[i];

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for get_state service to appear...");
        }

        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition;

        auto future = client_change_state_[i]->async_send_request(req);

        rclcpp::FutureReturnCode res = spin_until_future_complete(
            this->get_node_base_interface(), 
            future, 
            std::chrono::milliseconds(STATE_CHANGE_TIMEOUT_MS));


        success_flags |= (!client_response(res) << i);
    }

    // On full success, success_flags should look like 0b00...01111 or something,
    // the number of 1s will be the same as device_count_,
    // this will have the value 2^(device_count_) - 1, the same as (1 << device_count_) - 1,
    // on failure it will necessarily be different

    if ((((uint32_t)1 << device_count_) - 1) != success_flags) {
        return -1;
    }

    return 0;
}

int LifecycleManagerNode::client_response(rclcpp::FutureReturnCode res)
{

    if (res == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Successfully sent configure transition to device");
        return 0;
    }
    else if (res == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(this->get_logger(), "Timeout while calling configure transition on device");
        return -2;
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call configure transition on device");
        return -1;
    }

}

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    auto node = std::make_shared<LifecycleManagerNode>();

    exe.add_node(node->get_node_base_interface());
    exe.spin();

    rclcpp::shutdown();
    return 0;
}