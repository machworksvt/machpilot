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

/**
 * A class declaration used for every sensor node, is implemented per sensor.
 * Includes functions to initialize, reset, calibrate, run, stop, and publish data.
 * #TODO include other /msgs formats
 */
class Sensor : public rclcpp::Node {
public:
    /**
     * Creates a named node, 
     * @param[in] name the node's name, should be the name of the controller
     */
    Sensor(std::string name) : Node(name) {};
    ~Sensor() {};
    
    /**
     * Use a ROS2 service to initialize the sensor connection
     * 
     * @param[out] res the response to the service request
     * @param[in] req the request sent to the server
     * @returns bool the status of the service
     */
    bool initSrvs(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    /**
     * Use a ROS2 service to reset the sensor connection
     * 
     * @param[out] res the response to the service request
     * @param[in] req the request sent to the server
     * @returns the status of the service
     */
    bool resetSrvs(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    /**
     * Use a ROS2 action to calibrate the sensor connection
     */
    void calibAct();

    /**
     * Run the sensor node
     */
    void run();

    /**
     * Stop the sensor node
     */
    void stop();

    /**
     * Run the sensor node
     * @returns the publication status
     */
    bool publish();

private:
};


