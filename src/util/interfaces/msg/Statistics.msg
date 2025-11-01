# Statistics.msg
std_msgs/Header header
CanMsg can_msg
uint32 runs_ok # number of runs that were successful
uint32 runs_aborted # number of runs that failed
uint64 total_runtime # total time the engine has been running (s)