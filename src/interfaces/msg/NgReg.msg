# NgReg.msg
std_msgs/Header header
CanMsg can_msg
float32 integrator # Ng_Reg integrator, %
uint16 windup # windup code
float32 error # Ng_Reg error, %
float32 pump_power # high-res pump power, %