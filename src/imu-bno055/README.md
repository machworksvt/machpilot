# ROS1/ROS2 C++ driver for Bosch BNO055 IMU (I2C)

This is a light weight, C++ ROS node for the BNO055 IMU that communicates via I2C and without any dependencies besides libi2c-dev. It is specifically targeted at using a BNO055 with NVIDIA Jetson (Xavier, Orin, etc.) platforms but should work with a Raspberry Pi 5, or in an earlier Raspberry Pi using the software I2C mode.

The BNO055 supports I2C and UART communication. This driver supports I2C only. If you are looking for a UART driver, see [this driver](https://github.com/mdrwiega/bosch_imu_driver) by [mdrwiega](https://github.com/mdrwiega) instead.

## Where to buy

* [Adafruit](https://www.adafruit.com/product/4646) or [Adafruit](https://www.adafruit.com/product/2472)

## Installation

Install the prerequisites:
```
sudo apt install libi2c-dev
```

and then you are ready to drop this package into a catkin (ROS1) or colcon (ROS2) workspace.

## How to run
```
rosrun imu_bno055 bno055_i2c_node        # for ROS1
ros2 run imu_bno055 bno055_i2c_node      # for ROS2
```

## Parameters:

* **device** -- the path to the i2c device. Default is /dev/i2c-1. Use i2cdetect in the i2c-tools package to find out which bus your IMU is on.
* **address** -- the i2c address of the IMU. Default is 0x28.

## Outputs topics:
* **/data** (sensor\_msgs/Imu) -- fused IMU data
* **/raw** (sensor\_msgs/Imu) -- raw accelerometer data
* **/mag** (sensor\_msgs/MagneticField) -- raw magnetic field data
* **/temp** (sensor\_msgs/Temperature) -- temperature data
* **/status** (diagnostic\_msgs/DiagnosticStatus) -- a DiagnosticStatus object showing the current calibration, interrupt, system status of the IMU

## Service calls:
* **/reset** (std\_srvs/Trigger) -- resets the IMU
* **/calibrate** (std\_srvs/Trigger) -- not yet implemented

# Usage notes

## Raspberry Pi

The Raspberry Pi <=4 hardware I2C does not support clock stretching. You have a few options:

* Upgrade to Raspberry Pi 5
* [Use software I2C](https://github.com/fivdi/i2c-bus/blob/master/doc/raspberry-pi-software-i2c.md) instead which supports clock stretching but will increase CPU usage slightly.
* [Slow down the I2C clock drastically](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching), since the Pi does not support clock stretching. I have not tested this method.

## NVIDIA Jetson platforms

You may need to add your user to the i2c group, e.g. `sudo usermod -aG i2c nvidia`. It should just work after that.
