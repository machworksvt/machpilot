#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

/**
 * A class declaration used for every hardware device, is implemented per device.
 * Includes functions overridden from the LCN interfaces class.
 * #TODO include other /msgs formats
 */

#define DT 0.005 // seconds, should be used for timers

class Device : public rclcpp_lifecycle::LifecycleNode {
public:
    /**
     * Creates a named node, include the code in the top of the node's .cpp file
     * @param[in] name the node's name, should be the name of the actuator
     */
    Device(std::string name) : LifecycleNode(name) {};
    virtual ~Device() = default;

    /**
     * Called at the end of configure(), changes state based on success or failure
     * on SUCCESS, set state to Inactive (1) and on FAILURE, set state to Unconfigured (0)
     * if an error is raised, set state to ErrorProcessing (9)
     * @param[in] state the state enum to be changed 
     */
    virtual CallbackReturn on_configure(const rclcpp_lifecycle::State &state) = 0;

    /**
     * Called at the end of cleanup(), changes state based on success or failure
     * on SUCCESS, set state to Unconfigured (0) and on FAILURE, set state to Inactive (1)
     * if an error is raised, set state to ErrorProcessing (9)
     * @param[in] state the state enum to be changed 
     */
    virtual CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) = 0;

    /**
     * Called at the end of shutdown(), changes state based on success or failure
     * on SUCCESS, set state to Finalized (3) and on FAILURE, set state to last (0, 1, or 2)
     * if an error is raised, set state to ErrorProcessing (9)
     * @param[in] state the state enum to be changed 
     */
    virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) = 0;

    /**
     * Called at the end of activate(), changes state based on success or failure
     * on SUCCESS, set state to Active (2) and on FAILURE, set state to Inactive (1)
     * if an error is raised, set state to ErrorProcessing (9)
     * @param[in] state the state enum to be changed 
     */
    virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &state) = 0;

    /**
     * Called at the end of deactivate(), changes state based on success or failure
     * on SUCCESS, set state to Inactive (1) and on FAILURE, set state to Active (2)
     * if an error is raised, set state to ErrorProcessing (9)
     * @param[in] state the state enum to be changed 
     */
    virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) = 0;

private:
    
};