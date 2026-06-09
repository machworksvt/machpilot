# flight state machine

This is a the statemachine that manages the aircrafts states

The state is a wrapper around a tag union

## files

# events
holds enum to represent events

# flight_state_machine
file holding the statemachine

# has_react
contains meta programming tests

# state_machine_node
a wrapper around state machine node for ROS2 intigration

# states_enum
enum holding all subsystems

# transition_verifier
allows for other ros2 nodes to prevent a transition that do not want