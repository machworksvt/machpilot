# Icarus Pilot
A driver for all interactive electronic components onboard the Icarus UAV. This project is ran through the ROS2 (Robot Operating System) and driven by C++ code written by the avionics subteam for MachWorks.
## Contents
- [Digital Communication Protocol](#digital_communication_protocol)
- [API](#api)

## API
- [Class Sensor](./src/ext_deps/Sensor.md)
- 

## Digital Communication Protocol
Almost every sensor, communicator, or driver electronic on the Icarus uses a digital communication protocol for sending or recieving data. Since this is true, we had to use various libraries for communicating in different protocols, including:
- CAN
- USB
- I²C
- UART

