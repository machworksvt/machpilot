function [data, info] = voltageCurrent
%VoltageCurrent gives an empty data for interfaces/VoltageCurrent
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'interfaces/VoltageCurrent';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.battery_voltage, info.battery_voltage] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.battery_current, info.battery_current] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.flags, info.flags] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
info.MessageType = 'interfaces/VoltageCurrent';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'battery_voltage';
info.MatPath{7} = 'battery_current';
info.MatPath{8} = 'flags';
