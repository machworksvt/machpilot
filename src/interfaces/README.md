<!-- filepath: /home/ncollins22/machpilot/interfaces/README.md -->
# Interfaces Package

## Overview

The `interfaces` package defines the custom ROS2 messages and actions used by the MachPilot project. These definitions allow the system to exchange detailed telemetry and command information between the CAN interface node and other components of the system.

## Message Types

The following custom message types are defined in the package:

- **EngineData**: Contains detailed information about the engine, including:
  - Set and real RPM (scaled by a factor of 10)
  - Exhaust Gas Temperature (EGT)
  - Engine state and its human-readable name
  - Pump power

- **PumpRpm**: Provides the current pump RPM for the fuel system.

- **Errors**: Encapsulates an error mask along with a list of error messages and a flag indicating whether a reset is required.

- **GlowPlugs**: Contains voltage and current measurements for the glow plugs as well as sequence data.

- **FuelAmbient**: Includes fuel flow (ml/min), total fuel consumed (ml), engine box pressure (mbar), and ambient temperature (°C).

- **LastRunInfo**: Details information regarding the last engine run including runtime, off RPM, off EGT, pump power, and engine state.

- **NgReg**: Defines controller data such as the integrator value, windup, error percentage, and pump power.

- **Statistics**: Provides statistics on engine performance over time, including the number of successful and aborted runs and total runtime.

- **SystemInfo**: Contains system identifiers and operating information like the serial number, firmware version, engine type, and subtype.

- **SystemInfo2**: Provides additional system information, such as the ECU hardware serial number and the EIU software version.

- **VoltageCurrent**: Contains battery voltage, engine current, and any associated flags.

## Action Types

The package also defines several ROS2 actions to handle various tests and states for engine operation:

- **StarterTest**: Commands a starter test by sending the appropriate CAN frame and monitors the engine RPM to determine if the test passes.
- **PumpTest**: Executes a fuel pump test by monitoring the pumped fuel volume and pump RPM.
- **IgniterTest**: Initiates a test of the igniter (glow plugs) by checking that sufficient current is drawn.
- **Prime**: Handles the prime sequence for the engine control by verifying fuel flow.
- **Start**: A command action to start the engine (this is not treated as a test).
- **ThrottleProfile**: *(Implementation pending)* Intended to run a predefined throttle profile for the engine.

## Directory Structure

Here is an overview of the directory structure for the package:
