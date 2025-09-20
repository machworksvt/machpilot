# VoltageCurrent.msg
std_msgs/Header header
CanMsg can_msg
float32 battery_voltage # V
float32 battery_current # A
uint8 flags # not really sure what this means yet