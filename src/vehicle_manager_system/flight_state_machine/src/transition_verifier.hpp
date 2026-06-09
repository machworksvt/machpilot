#ifndef TRANSITION_VERIFIER_HPP
#define TRANSITION_VERIFIER_HPP

#include <array>
#include <string_view>
#include "rclcpp/rclcpp.hpp"
#include "flight_state_machine_interface/srv/transition_request.hpp"

#include "states_enum.hpp"

/// The size of SERVICE_NAMES
const std::size_t SERVICES_COUNT = 0;

/// An array of ros2 client names that we verify
const std::array<std::string_view,SERVICES_COUNT> SERVICE_NAMES{{}};

class TransitionVerifier{
    public:
        /**
         * @brief initialize TransitionVerifier
         * 
         * @param [in] parrent the node that it will send messages on behalf of
         */
        TransitionVerifier(rclcpp::Node &parrent);
        
        /**
         * @brief sends message to all serices in SERVICE_NAMES and returns true if all of them accept the say ready
         * 
         * @param [in] timeout how long to wait until we exit and return false
         * 
         * @param [in] new_state the state we are try requesting to trasition to
         * 
         * @return if all services return true within the timeout 
         */
        bool request_transition(std::chrono::milliseconds timeout,StateEnum new_state);
    private:

        /// The array of clients that we verfiy request with
        std::array<rclcpp::Client<flight_state_machine_interface::srv::TransitionRequest>::SharedPtr,SERVICES_COUNT> services;

};

#endif