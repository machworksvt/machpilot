# EngineData.msg
std_msgs/Header header
uint32 set_rpm #1/min
uint32 real_rpm #1/min
float32 egt #C
uint8 state #enum, from above
float32 pump_power #%