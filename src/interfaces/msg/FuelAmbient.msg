# FuelAmbient.msg
std_msgs/Header header
CanMsg can_msg
uint32 fuel_flow # ml/min
uint32 fuel_consumed # ml
float32 engine_box_pressure # mbar
int16 ambient_temperature # C