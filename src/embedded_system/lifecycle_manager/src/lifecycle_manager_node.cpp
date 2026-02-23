#include "lifecycle_manager_node.hpp"

#include <cstdlib>

using std::placeholders::_1;



LifecycleManagerNode::LifecycleManagerNode() 
    : Device("lifecycle_manager_node")
{
    cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    srvs_exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    srvs_exec_->add_callback_group(cbg_, this->get_node_base_interface());

    if (this->scan_and_add_devices()) {
        RCLCPP_ERROR(this->get_logger(), "Error scanning and adding devices");
        exit(1);
    }

    int erc = loop_get_state_clients(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    if (erc == -1) {
        RCLCPP_INFO(this->get_logger(), "All nodes not in correct state");
        exit(1);
    }

    RCLCPP_INFO(this->get_logger(), "All nodes in the correct state, proceding ...");
}

LifecycleManagerNode::~LifecycleManagerNode()
{
    srvs_exec_->cancel();
    if (srvs_exec_thread_.joinable()) srvs_exec_thread_.join();
}

/**
 * Triggers a transition in all hardware nodes, then checks the result
 * @param[in] result_state the intended state
 * @param[in] transition the transition to the intended state
 * @param[in] is_fallback whether or not state is being rolled back because of an error, changes when this function returns
 */
bool LifecycleManagerNode::__callback_routine(uint8_t result_state, uint8_t transition, bool is_fallback) {

    int erc = loop_change_state_clients(transition);

    if (erc == -1 && is_fallback) {
        RCLCPP_INFO(this->get_logger(), "Not all transitions sent successfully");
        return 1;
    }

    if (erc == -2 && is_fallback) {
        RCLCPP_INFO(this->get_logger(), "Error in client loop");
        return 1;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully sent transitions");

    erc = loop_get_state_clients(result_state);

    if (erc == -1) {
        RCLCPP_INFO(this->get_logger(), "Not all transitions successful");
        return 1;
    }

    RCLCPP_INFO(this->get_logger(), "All transitions successful");

    return 0;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    if (__callback_routine(
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, 
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        false)) {
        
        // switch back to state before callback by performing the opposite transition
        if (__callback_routine(
            state.id(), 
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
            true)) {
                return CallbackReturn::ERROR;
            }

        return CallbackReturn::FAILURE;
    }
    
    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_activate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    if (__callback_routine(
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, 
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
        false)) {
        
        // switch back to state before callback by performing the opposite transition
        if (__callback_routine(
            state.id(), 
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
            true)) {
                return CallbackReturn::ERROR;
            }

        return CallbackReturn::FAILURE;
    }
    
    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    if (__callback_routine(
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, 
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
        false)) {
        
        // switch back to state before callback by performing the opposite transition
        if (__callback_routine(
            state.id(), 
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
            true)) {
                return CallbackReturn::ERROR;
            }

        return CallbackReturn::FAILURE;
    }
    
    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    if (__callback_routine(
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, 
        lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
        false)) {
        
        // switch back to state before callback by performing the opposite transition
        if (__callback_routine(
            state.id(), 
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
            true)) {
                return CallbackReturn::ERROR;
            }

        return CallbackReturn::FAILURE;
    }
    
    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    uint8_t transition = 0;

    // to guarantee the right transition
    switch (state.id())
    {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
        transition = lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
        break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        transition = lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN;
        break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        transition = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN;
        break;
    default:
        break;
    }

    // no recourse if this transition fails, but this should never fail
    if (__callback_routine(
        lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, 
        transition,
        false)) {
            
        RCLCPP_ERROR(get_logger(), "Shutdown transition failed, system in undetermined state");
        return CallbackReturn::ERROR;
    }
    
    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());
    /**
     * This will be very sub-node dependent and involved, reserved for future
     */

    return CallbackReturn::SUCCESS;
}

/**
 * Checks for all exposed get_state and set_state services available at the time, and adds them to
 * client_get_state_ and client_set_state_
 */
int LifecycleManagerNode::scan_and_add_devices()
{

    // this function may not work every time, testing is needed
    auto services = get_service_names_and_types();

    for (const auto& [key, value] : services) {

        // check key is a state related service
        if (key.find("change_state") == std::string::npos && key.find("get_state") == std::string::npos) {
            continue;
        }

        // Skip own services, will be handled with trigger_transition() calls
        if (key.find(this->get_name()) != std::string::npos) {
            RCLCPP_INFO(get_logger(), "Found own service: %s", key.c_str());
            continue;
        }

        // #TODO: test whether the device services are added correctly, doing this differently would be better

        // add clients to all services
        if (key.find("change_state") != std::string::npos) {
            client_change_state_.push_back(
                this->create_client<lifecycle_msgs::srv::ChangeState>(
                    key,
                    rmw_qos_profile_services_default,
                    cbg_
                )
            );
            RCLCPP_INFO(get_logger(), "Added ChangeState client for service: %s", key.c_str());
        }

        if (key.find("get_state") != std::string::npos) {
            client_get_state_.push_back(
                this->create_client<lifecycle_msgs::srv::GetState>(
                    key,
                    rmw_qos_profile_services_default,
                    cbg_
                )
            );
            RCLCPP_INFO(get_logger(), "Added GetState client for service: %s", key.c_str());
        }

    }

    // check edge cases for state vectors
    if (client_get_state_.empty() || 
        client_change_state_.empty() || 
        client_get_state_.size() != client_change_state_.size() || 
        client_get_state_.size() > MAX_NODES - 1    
    ) {
        RCLCPP_WARN(get_logger(), "No lifecycle devices found, or the number of services registered is different than expected");
        return -1;
    }

    return 0;
}

/**
 * Triggers a transition in all hardware node state
 * @param[in] transition the state, the current lifecycle manager state
 */
int LifecycleManagerNode::loop_change_state_clients(uint8_t transition)
{
    for (auto client : client_change_state_) {

        while (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for get_state service to appear...");
        }

        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition;

        auto future = client->async_send_request(req);

        rclcpp::FutureReturnCode res = srvs_exec_->spin_until_future_complete(
            future, 
            std::chrono::milliseconds(SERVICE_TIMEOUT_MS));


        success_flags_ = success_flags_ << 1;
        success_flags_ |= !client_response(res);
    }

    // On full success, success_flags should look like 0b00...01111 or something,
    // the number of 1s will be the same as device_count_,
    // this will have the value 2^(device_count_) - 1, the same as (1 << device_count_) - 1,
    // on failure it will necessarily be different

    if (client_change_state_.size() != success_flags_.count()) {
        return -1;
    }

    return 0;
}

/**
 * Checks the state of each node and checks if it matches
 * @param[in] state the state, the current lifecycle manager state
 */
int LifecycleManagerNode::loop_get_state_clients(uint8_t state) {

    for (auto client : client_get_state_) {

        RCLCPP_INFO(this->get_logger(), "Hello");

        while (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for get_state service to appear...");
        }

        auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();


        auto future = client->async_send_request(req);

        rclcpp::FutureReturnCode res = srvs_exec_->spin_until_future_complete(
            future, 
            std::chrono::milliseconds(SERVICE_TIMEOUT_MS));


        if (client_response(res)) continue;

        // compare between intended and actual state
        if (future.get()->current_state.id == state) return -1;
    }

    return 0;
}

int LifecycleManagerNode::client_response(rclcpp::FutureReturnCode res)
{

    if (res == rclcpp::FutureReturnCode::SUCCESS) {
        return 0;
    }
    else if (res == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_ERROR(this->get_logger(), "Timeout while requesting from device");
        return 1;
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Failed to request from device");
        return 1;
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