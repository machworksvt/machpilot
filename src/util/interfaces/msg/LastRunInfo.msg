# LastRunInfo.msg
std_msgs/Header header
CanMsg can_msg
uint16 last_runtime # time the last run took (s)
uint16 last_off_rpm # RPM the engine was running at when it was turned off
uint16 last_off_egt # EGT the engine was running at when it was turned off
float32 last_off_pump_power # pump power the engine was running at when it was turned off
uint8 last_off_state # state the engine was in when it was turned off
string last_off_state_str # state the engine was in when it was turned off