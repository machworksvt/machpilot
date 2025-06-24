#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

enum states {
    Unconfigured = 0,
    Inactive = 1,
    Active = 2,
    Finalized = 3,
    Configuring = 4,
    CleaningUp = 5,
    ShuttingDown = 6,
    Activating = 7,
    Deactivating = 8,
    ErrorProcessing = 9
};

/**
 * A class declaration used for every controller or output node, is implemented per device.
 * Includes functions to initialize, reset, run, stop.
 * #TODO include other /msgs formats
 */

#define DT 0.005 // seconds, should be used for timers

class Controller : public rclcpp::Node {
public:
    /**
     * Creates a named node, 
     * @param[in] name the node's name, should be the name of the controller
     */
    Controller(std::string name) : Node(name) {};
    ~Controller() {};

    /**
     * Creates an unconfigured node, initializes states
     * set state to Unconfigured (0)
     * maybe unneeded since class constructor exists
     */
    void create();

    /**
     * Configures a node, takes from config and launch files to init actuator
     * -read launch to add actuator, test comms, init bus-
     * -initialize timers, callbacks, publishers, subscribers, services, etc-
     * set state to Configuring (4)
     */
    void configure();

    /**
     * Called at the end of configure(), changes state based on success or failure
     * on SUCCESS, set state to Inactive (1) and on FAILURE, set state to Unconfigured (0)
     * if an error is raised, set state to ErrorProcessing (9)
     * -@param[in] statusses the results of the processes in configure()-
     */
    void onConfigure();

    /**
     * Cleans up the stuff added in configure()
     * -resets all variables associated with actuator, bus, launch, config-
     * set state to CleaningUp (5)
     */
    void cleanup();

    /**
     * Called at the end of cleanup(), changes state based on success or failure
     * on SUCCESS, set state to Unconfigured (0) and on FAILURE, set state to Inactive (1)
     * if an error is raised, set state to ErrorProcessing (9)
     * -@param[in] statusses the results of the processes in cleanup()-
     */
    void onCleanup();

    /**
     * Cleans up all unnecessary variables, stops all timers, callbacks, etc
     * set state to ShuttingDown (6)
     * -@param[in] the current state, to decide what to do-
     */
    void shutdown();

    /**
     * Called at the end of shutdown(), changes state based on success or failure
     * on SUCCESS, set state to Finalized (3) and on FAILURE, set state to last (0, 1, or 2)
     * if an error is raised, set state to ErrorProcessing (9)
     * -@param[in] statusses the results of the processes in shutdown()-
     */
    void onShutdown();

    /**
     * Runs the node, calls callbacks and other processes
     * set state to Activating (7)
     * -calls spin() ?-
     */
    void activate();

    /**
     * Called at the end of activate(), changes state based on success or failure
     * on SUCCESS, set state to Active (2) and on FAILURE, set state to Inactive (1)
     * if an error is raised, set state to ErrorProcessing (9)
     * -@param[in] statusses the results of the processes in activate()-
     */
    void onActivate();

    /**
     * Stops all processes associated with activate()
     * set state to Deactivating (8)
     */
    void deactivate();

    /**
     * Called at the end of deactivate(), changes state based on success or failure
     * on SUCCESS, set state to Inactive (1) and on FAILURE, set state to Active (2)
     * if an error is raised, set state to ErrorProcessing (9)
     * -@param[in] statusses the results of the processes in deactivate()-
     */
    void onDeactivate();

    /**
     * Error callback, handles errors and resets bad information
     * on SUCCESS, set state to Unconfigured (0) and on FAILURE, set state to Finalized (3)
     * if an error is raised, this is considered a FAILURE
     */
    void doError();

    /**
     * Destroys the node, deletes all data associated
     * maybe unneeded since class destructor exists
     */
    void destroy();

private:
    
};


