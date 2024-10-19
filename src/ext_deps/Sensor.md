# Sensor
This class is an implementation of the ROS2 `Node` class with functions related to sensor connection, calibration, and data collection. This document also serves as a guide for implementing this superclass.

## Functions
- `Constructor([name])`: initializes a given sensor doing the following
    - Declares and sets applicable parameters, which can include the device address, sampling rate, etc. Sets these to members of the implementing class.
    - Creates publishers over topics with names `"<sensorname>_<datatype>"`, where the `<datatype>` is related to the message type in the publisher constructor. Each publisher is also set as a member.
    - Creates `"init"` and `"reset"` services and sets them as members.
    - Checks errors along the way, and prints any statuses to a diagnostic publisher.
- `initSrvs([request], [result])`: uses a ROS2 `service` to initialize the sensor connection over the preferred DCC.
    - Runs an `init()` function and checks on the status of init.
    - Sets the `success` and `message` attributes of the `result` of the service.
    - Returns the success status.
- `resetSrvs([request], [result])`: uses a ROS2 `service` to reset the sensor connection over the preferred DCC.
    - Runs a `reset()` function and checks on the status of reset.
    - Sets the `success` and `message` attributes of the `result` of the service.
    - Returns the success status.
- `run()`: handles the timing and the conditions of the `publish()` function.
    - Checks the result of `rclcpp::ok()`.
    - Sleeps until the rate turns over, using `rate->sleep()`.
    - Runs the `publish()` function to collect sensor data.
- `stop()`: stops the publishing loop.
    - Runs `<pubname>.reset()` on every publisher assigned to the sensor.
- `publish()`: publishes data to each topic assigned to each publisher.
    - Read data from the sensor.
    - Assigns sensor record data to messages and stamps each message with the time, using `this->now()`.
    - Publishes the messages over the topics using `publish(data)`.
    - Publishes statusses over the status topic using `publish(status)`.
    - Returns the status of the publish, checks errors along the way.

## Members
- Any parameters applicable to the sensor.
- Publishers the sensor uses.
- Services the sensor uses.

## The `main()` Function
The `main()` function serves as an entry point for running the node by itself, and is useful for testing. This should at least include the following:
- Use the `rclcpp::init()` function to pass parameter arguments into the sensor node. These arguments are created in the  `"<sensorname>.launch"` file.
- Create a `Node` object by instantiating the sensor and making it a shared pointer using the `std::make_shared<Sensor>()` function.
- Run the sensor with `Sensor->run()`.
- Stop the sensor with `rclcpp::shutdown()`.
- Return error codes.
