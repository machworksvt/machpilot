function [data, info] = lastRunInfo
%LastRunInfo gives an empty data for interfaces/LastRunInfo
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'interfaces/LastRunInfo';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.last_runtime, info.last_runtime] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0);
[data.last_off_rpm, info.last_off_rpm] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0);
[data.last_off_egt, info.last_off_egt] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0);
[data.last_off_pump_power, info.last_off_pump_power] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.last_off_state, info.last_off_state] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.last_off_state_str, info.last_off_state_str] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
info.MessageType = 'interfaces/LastRunInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'last_runtime';
info.MatPath{7} = 'last_off_rpm';
info.MatPath{8} = 'last_off_egt';
info.MatPath{9} = 'last_off_pump_power';
info.MatPath{10} = 'last_off_state';
info.MatPath{11} = 'last_off_state_str';
