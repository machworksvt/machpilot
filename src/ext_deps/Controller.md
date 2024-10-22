# Controller
This class is an implementation of the ROS2 `Node` class with functions related to controller connection, calibration, and data interpretation. This document also serves as a guide for implementing this superclass.

## Functions
- `Constructor([name])`: initializes a given controller, doing the following
    - Declares and sets applicable parameters, which can include the device address, sampling rate, etc. Sets these to members of the implementing class.
    - Creates subscribers to topics with names `"<sensorname>_<datatype>"`, where the `<datatype>` is related to the message type in the sensor's publisher constructor. Each publisher is also set as a member.
    - Creates `"init"` and `"reset"` services and sets them as members.
    - Checks errors along the way, and prints any statuses to a diagnostic publisher.
- `initSrvs([request], [result])`: uses a ROS2 `service` to initialize the controller connection over the preferred DCC.
    - Runs an `init()` function and checks on the status of init.
    - Sets the `success` and `message` attributes of the `result` of the service.
    - Returns the success status.
- `resetSrvs([request], [result])`: uses a ROS2 `service` to reset the controller connection over the preferred DCC.
    - Runs a `reset()` function and checks on the status of reset.
    - Sets the `success` and `message` attributes of the `result` of the service.
    - Returns the success status.
- `run()`: handles the timing and the conditions of the `publish()` function.
    - Checks the result of `rclcpp::ok()`.
    - Sleeps until the rate turns over, using `rate->sleep()`.
    - Runs the `publish()` function to collect controller data.
- `stop()`: stops the publishing loop.
    - Runs `<pubname>.reset()` on every publisher assigned to the controller.
- `subCallBack1([msg])`: a private function only used in the creation of subscribers and ran only when topics are published to.
    - Extracts content from msg.
    - Processes extracted data and uses it in input for controller.
    - Sends commands over DCC telling what the physical controller should do (this might have to be a service).
    - If the previous point includes a service, make sure to initialize it in the constructor, not here.

## Members
- Any parameters applicable to the sensor.
- Subscribers the controller uses.
- Services the sensor uses.

## The `main()` Function
The `main()` function serves as an entry point for running the node by itself, and is useful for testing. This should at least include the following:
- Use the `rclcpp::init()` function to pass parameter arguments into the controler node. These arguments are created in the `"<sensorname>.launch"` file.
- Create a `Node` object by instantiating the sensor and making it a shared pointer using the `std::make_shared<Sensor>()` function.
- Run the controller with `Controller->run()`.
- Stop the controller with `rclcpp::shutdown()`.
- Return error codes.
