<!-- filepath: /home/ncollins22/machpilot/interfaces/INTERFACES_DOCUMENTATION.md -->
# Interfaces Package Documentation

This document details the custom ROS2 messages, actions, and services defined in the `interfaces` package. These definitions allow different subsystems of the MachPilot project to exchange detailed telemetry and command information through ROS topics, actions, and services.

---

## Message Types

### CanMsg.msg

**Purpose:**  
A generic container for raw CAN messages.

**Fields:**
- `uint16 id`  
  The unique CAN frame identifier.
- `uint8 dlc`  
  Data length code, indicating how many data bytes are valid.
- `uint8[8] data`  
  An array containing up to 8 bytes of the CAN frame payload.

---

### EngineData.msg

**Purpose:**  
Provides detailed engine telemetry.

**Fields:**
- `std_msgs/Header header`  
  Standard ROS header (time stamp and frame id).
- `CanMsg`
  Associated CAN message.
- `uint32 set_rpm`
  Current RPM setpoint.
- `uint32 real_rpm`
  Current actual RPM.
- `float32 egt`  
  Exhaust Gas Temperature. (°C)
- `uint8 state`  
  Engine state code.
- `string state_name`  
  Readable name for the current engine state.
- `float32 pump_power`  
  Power output of the fuel pump. (%)

---

### PumpRpm.msg

**Purpose:**  
Conveys the instantaneous RPM of the fuel pump.

**Fields:**
- `uint16 rpm`  
  The current revolutions per minute of the pump.

---

### Errors.msg

**Purpose:**  
Encapsulates error information reported by the engine.

**Fields:**
- `uint32 error_mask`  
  A bitmask representing the current error conditions.
- `string[] error_messages`  
  A list of human-readable error messages.
- `bool reset_required`  
  Indicates if a reset condition must be met before recovery.

---

### GlowPlugs.msg

**Purpose:**  
Provides telemetry data for the glow plug system.

**Fields:**
- `float32 voltage`  
  The voltage measured across the glow plugs.
- `float32 current`  
  The current draw of the glow plugs.
- `uint8 sequence`  
  Optional sequence number or status flag for glow plug operation.

---

### FuelAmbient.msg

**Purpose:**  
Reports ambient conditions relevant to engine performance.

**Fields:**
- `float32 fuel_flow`  
  Fuel flow in milliliters per minute.
- `float32 total_fuel`  
  Cumulative fuel consumption in milliliters.
- `float32 engine_box_pressure`  
  Pressure inside the engine box (in mbar).
- `float32 ambient_temperature`  
  Ambient temperature (in °C).

---

### LastRunInfo.msg

**Purpose:**  
Details the results from the last engine run.

**Fields:**
- `float32 runtime`  
  Duration of the engine run.
- `uint16 off_rpm`  
  Engine RPM when the engine was turned off.
- `float32 off_egt`  
  Exhaust Gas Temperature at engine shutdown.
- `float32 pump_power`  
  Pump power reading at shutdown.
- `uint8 engine_state`  
  Engine state at last run (numeric code).

---

### NgReg.msg

**Purpose:**  
Contains regulator-related controller data.

**Fields:**
- `float32 integrator_value`  
  The current integrator reading.
- `float32 windup`  
  The amount of integrator windup.
- `float32 error_percentage`  
  The percentage error.
- `float32 pump_power`  
  Pump power associated with the error/regulation.

---

### Statistics.msg

**Purpose:**  
Provides statistical data on engine performance over time.

**Fields:**
- `uint32 successful_runs`  
  Number of runs that completed successfully.
- `uint32 aborted_runs`  
  Number of runs that were aborted.
- `float32 total_runtime`  
  Total runtime of the engine over all runs (in seconds).

---

### SystemInfo.msg

**Purpose:**  
Conveys system identification and basic operational info.

**Fields:**
- `string serial_number`  
  Unique serial number of the system.
- `string firmware_version`  
  The version of the installed firmware.
- `string engine_type`  
  Type or model of the engine.
- `string engine_subtype`  
  Sub-type or revision of the engine.

---

### SystemInfo2.msg

**Purpose:**  
Provides extended system information.

**Fields:**
- `string ecu_hardware_serial`  
  ECU (Engine Control Unit) hardware serial number.
- `string eiu_software_version`  
  EIU (Engine Interface Unit) software version.

---

### VoltageCurrent.msg

**Purpose:**  
Presents power and current measurements.

**Fields:**
- `float32 battery_voltage`  
  The current voltage of the battery.
- `float32 engine_current`  
  The current drawn by the engine.
- `uint8 flags`  
  Any additional flags or status bits related to electrical measurements.

---

## Action Types

Each action uses the standard ROS2 action format with goal, result, and feedback messages. The following actions are defined:

### StarterTest.action

**Purpose:**  
Commands a test to verify the starter circuit and engine RPM behavior.

**Goal:**  
_Parameters for initiating the test (if any)._  
*(Typically empty if no parameters are needed.)*

**Result:**  
- `bool success`  
  Indicates if the starter test passed.
- `string message`  
  Additional feedback or error message.

**Feedback:**  
- `uint8 progress`  
  Progress indicator (e.g., percentage complete).
- `string status`  
  A short status message describing the test phase.

---

### PumpTest.action

**Purpose:**  
Executes a fuel pump test by monitoring pump performance.

**Goal:**  
*(Can be empty or include test parameters such as test duration.)*

**Result:**  
- `bool success`  
  True if the pump test was successful.
- `string message`  
  Explanation of the result or any errors.

**Feedback:**  
- `uint8 progress`  
  Test progress indicator.
- `string status`  
  Status update on pump performance.

---

### IgniterTest.action

**Purpose:**  
Initiates a test of the igniter (glow plug) circuit, checking for proper current draw.

**Goal:**  
*(Typically empty unless specific configuration values are needed.)*

**Result:**  
- `bool success`  
  Indicates whether the igniter test passed.
- `string message`  
  Detailed result message.

**Feedback:**  
- `uint8 progress`  
  Progress percentage of the igniter test.
- `string status`  
  Short status description.

---

### Prime.action

**Purpose:**  
Runs the fuel priming sequence. May need to be used multiple times.

**Goal:**  
*(Empty)*

**Result:**  
- `bool success`  
  Indicates if priming was successful.
- `string message`  
  Additional feedback or error details.

**Feedback:**  
- `uint8 progress`  
  Progress of the priming operation.
- `string status`  
  Current state message during priming.

---

### Start.action

**Purpose:**  
Issues a command to start the engine. Engine persists as on after execution typically.

**Goal:**  
*(Empty)*

**Result:**  
- `bool success`  
  True if the engine start command was successfully executed.
- `string message`  
  Diagnostic or confirmation message.

**Feedback:**  
- `uint8 progress`  
  Feedback on initiation progress.
- `string status`  
  Status description (e.g., "Starting", "Running").

---

### ThrottleProfile.action

**Purpose:**  
Intended to run a predefined throttle profile to control engine performance.  
**Status:** Implementation pending.

**Goal:**  
- `string csv_file_path`
  Path to a .csv file containing a predfined throttle profile. Intended for use during engine testing.

**Result:**  
- `bool success`  
  Indicates if the throttle profile was executed correctly.
- `string message`  
  Explanation or diagnostic details.

**Feedback:**  
- `uint8 progress`  
  Indicates percentage completion.
- `string status`  
  Describes current throttle profile activity.

---

## Build and Usage Instructions

To incorporate these custom message and action definitions into your ROS2 projects:

1. **Building the Package:**
   ```bash
   cd /home/ncollins22/machpilot
   colcon build --packages-select interfaces
   source install/setup.bash