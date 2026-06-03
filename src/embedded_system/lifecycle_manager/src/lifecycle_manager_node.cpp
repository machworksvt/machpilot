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

    // if is_fallback is true it should skip the change_state verification
    if (erc == -1 && !is_fallback) {
        RCLCPP_INFO(this->get_logger(), "Not all transitions sent successfully");
        return 1;
    }

    if (erc == -2 && !is_fallback) {
        RCLCPP_INFO(this->get_logger(), "Error in client loop");
        return 1;
    }

    RCLCPP_INFO(this->get_logger(), "Successfully sent transitions");

    erc = loop_get_state_clients(result_state);

    if (erc == -1) {
        RCLCPP_INFO(this->get_logger(), "Not all transitions successful");
        state_check_comp();
        return 2;
    }

    RCLCPP_INFO(this->get_logger(), "All transitions successful");

    return 0;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    // trigger and check the transition
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

    // trigger and check the transition
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

    // #TODO add publishers of changes to hardware nodes, maybe via the health monitor?

    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        [this]() -> void {
            if (loop_get_state_clients(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)) {
                RCLCPP_ERROR(this->get_logger(), "Some client(s) not communicated with");
                trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR);
            }

            if (state_check_comp()) {
                RCLCPP_ERROR(this->get_logger(), "Some client state(s) inaccessible or different from expected");
                trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR);
            }
        }
    );
    return CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    // trigger and check the transition
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

    // trigger and check the transition
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

/**
 * Handles the two main cases that can arise during the operation of this node. Sub-nodes are supposed to handle
 * their own errors, and them transitioning to Error does not cause this node to do the same. 
 * Removes the erring node from the service and state vectors upon failure to handle these errors.
 * @param state a reference to the current (origin) state for this transition
 */
LifecycleNodeInterface::CallbackReturn LifecycleManagerNode::on_error(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "%s is in state: %s", this->get_name(), state.label().c_str());

    uint8_t index;

    if (current_err_type_ == STATE_GET_UNRESPONSIVE) {
        goto unresponsive_handling; // wierd ahh goto
        unresponsive_handling:
            while (index = handle_no_comms() + 1) {
                index--; //remove the +1 effect
                client_states_.erase(client_states_.begin() + index);
                client_get_state_.erase(client_get_state_.begin() + index);
                client_change_state_.erase(client_change_state_.begin() + index);
            }

            if (client_states_.empty()) {
                RCLCPP_FATAL(get_logger(), "All nodes not communicating or erroniously removed from manager");

                return CallbackReturn::FAILURE;
            }

            loop_get_state_clients(state.id()); // updates client_states_
            state_check_comp(); // updates current_err_type_
            if (current_err_type_ == STATE_GET_UNRESPONSIVE) {
                RCLCPP_ERROR(get_logger(), "Somehow another node stopped communicating in the meantime");
                goto unresponsive_handling; 
            }
    }

    if (current_err_type_ == STATE_MISMATCH) {
        goto mismatch_handling; // wierd ahh goto
        mismatch_handling:
        while (index = handle_errant_states() + 1) {
            index--; //remove the +1 effect
            client_states_.erase(client_states_.begin() + index);
            client_get_state_.erase(client_get_state_.begin() + index);
            client_change_state_.erase(client_change_state_.begin() + index);
        }

        if (client_states_.empty()) {
            RCLCPP_FATAL(get_logger(), "All nodes in unrecoverable state or erroniously removed from manager");

            return CallbackReturn::FAILURE;
        }

        loop_get_state_clients(state.id()); // updates client_states_
        state_check_comp(); // updates current_err_type_
        if (current_err_type_ == STATE_GET_UNRESPONSIVE) { // the only reason to use goto
            RCLCPP_ERROR(get_logger(), "Somehow another node stopped communicating in the meantime");
            goto unresponsive_handling; 
        }

        if (current_err_type_ == STATE_MISMATCH) {
            RCLCPP_ERROR(get_logger(), "Somehow another node entered a mismatched state in the meantime");
            goto mismatch_handling; 
        }
    }

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
            client_states_.push_back(0); // initialize states vector
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
 * @param[in] transition the current lifecycle manager state
 */
int LifecycleManagerNode::loop_change_state_clients(uint8_t transition)
{
    success_flags_.reset();

    for (auto client : client_change_state_) {

        while (!client->wait_for_service(std::chrono::milliseconds(SERVICE_TIMEOUT_MS))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for get_state service to appear...");
        }

        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = transition;

        auto future = client->async_send_request(req);

        rclcpp::FutureReturnCode res = srvs_exec_->spin_until_future_complete(
            future, 
            std::chrono::milliseconds(STATE_SERVICE_TIMEOUT_MS));

        success_flags_ = success_flags_ << 1;
        success_flags_ |= !client_response(res);
    }

    // On full success, success_flags should look like 0b00...01111 or something,
    // the number of 1s will be the same as device_count_,
    // this will have the value 2^(device_count_) - 1, the same as (1 << device_count_) - 1,
    // on failure it will necessarily be different

    if (client_change_state_.size() != success_flags_.count()) {
        // #TODO: add logging to determine which node(s) failed the transitions
        return -1;
    }

    return 0;
}

/**
 * Checks the state of each node and checks if it matches
 * @param[in] state the state, the current lifecycle manager state
 */
int LifecycleManagerNode::loop_get_state_clients(uint8_t state) {

    int rc = 0, i = 0;

    for (auto client : client_get_state_) {

        RCLCPP_INFO(this->get_logger(), "Hello");

        while (!client->wait_for_service(std::chrono::milliseconds(SERVICE_TIMEOUT_MS))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for get_state service to appear...");
        }

        auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        auto future = client->async_send_request(req);

        rclcpp::FutureReturnCode res = srvs_exec_->spin_until_future_complete(
            future, 
            std::chrono::milliseconds(STATE_SERVICE_TIMEOUT_MS));

        if (client_response(res)) {
            rc = -1;
            client_states_[i] = STATE_UNKNOWN; // set the state to a value not associated with a state to denote error
            i++;
            // skip next step if errant response, but keep reading states of other clients
            continue;
        }

        // compare between intended and actual state
        if (future.get()->current_state.id != state) rc = -1; // sets rc to -1 on first occurrence of bad state
        client_states_[i] = future.get()->current_state.id;
        i++;
    }

    return rc;
}

/**
 * Switch returns based on the result of the future
 */
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

/**
 * Prints based on what's in the client_states_ vector, and 
 * returns different values to guide this node's error procedures
 */
uint LifecycleManagerNode::state_check_comp()
{
    uint current_err_type_ = 0; //takes the value of the most severe state comp failure

    uint8_t m_state = this->get_current_state().id();
    for (uint i = 0; i < client_states_.size(); i++) {
        uint8_t c_state = client_states_[i];
        if (c_state != m_state) {
            if (c_state == STATE_UNKNOWN) {
                RCLCPP_ERROR(this->get_logger(), "State from node %d unknown, error in communication with get_state service", i);
                current_err_type_ = 2;
                continue;
            }

            RCLCPP_ERROR(this->get_logger(), "State from node %d: code %d different from expected: code %d", i, c_state, m_state);
            if (current_err_type_ < 2) current_err_type_ = 1;
        }
    }

    if (current_err_type_ == 0) RCLCPP_INFO(this->get_logger(), "No problems found");

    return current_err_type_; // return it for convenience
}

/**
 * Tries to get a bad node back into the desired state,
 * this function needs to be ran multiple times in succession to clear all errors,
 * but there shouldn't be many when this runs
 * @return the index of the bad node on failure, -1 on success
 */
int LifecycleManagerNode::handle_errant_states()
{
    int i = 0, rc;
    uint8_t bad_state;
    for (;i < client_states_.size(); i++) {
        bad_state = client_states_[i];
        if (bad_state != this->get_current_state().id()) break;
    }

    // this part sucked bad, and only works if there is 1 state difference
    auto nearby_states = this->get_available_states();
    for (rclcpp_lifecycle::State state : nearby_states) {
        if (state.id() == bad_state) {
            int key = 2 * state.id() - this->get_current_state().id();
            int transition = t_map.find(key)->second;

            rc = __callback_routine(this->get_current_state().id(),
                                    transition,
                                    false);
            if (rc != 0) return i;
        }
    }

    return -1;
}

/**
 * Tries to get a bad node to communicate again,
 * this function needs to be ran multiple times in succession to clear all errors,
 * but there shouldn't be many when this runs
 * @return the index of the bad node on failure, -1 on success
 */
int LifecycleManagerNode::handle_no_comms()
{
    int i = 0;
    uint8_t bad_state;
    for (;i < client_get_state_.size(); i++) {
        if (client_states_[i] == STATE_UNKNOWN) {
            
            while (!client_get_state_[i]->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "Waiting for get_state service to appear...");
            }

            auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
            auto future = client_get_state_[i]->async_send_request(req);

            rclcpp::FutureReturnCode res = srvs_exec_->spin_until_future_complete(
                future, 
                std::chrono::milliseconds(SERVICE_TIMEOUT_MS));

            if (client_response(res)) return i;

            // compare between intended and actual state
            if (future.get()->current_state.id != this->get_current_state().id()) 
                RCLCPP_WARN(this->get_logger(), "State is wrong, but comms re-established");
            client_states_[i] = future.get()->current_state.id;
        }
    }
    return -1;
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