function [data, info] = ngReg
%NgReg gives an empty data for interfaces/NgReg
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'interfaces/NgReg';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.integrator, info.integrator] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.windup, info.windup] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.error, info.error] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.pump_power, info.pump_power] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
info.MessageType = 'interfaces/NgReg';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'integrator';
info.MatPath{7} = 'windup';
info.MatPath{8} = 'error';
info.MatPath{9} = 'pump_power';
