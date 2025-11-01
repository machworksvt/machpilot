function [data, info] = fuelAmbient
%FuelAmbient gives an empty data for interfaces/FuelAmbient
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'interfaces/FuelAmbient';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.fuel_flow, info.fuel_flow] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.fuel_consumed, info.fuel_consumed] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.engine_box_pressure, info.engine_box_pressure] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
[data.ambient_temperature, info.ambient_temperature] = ros.internal.ros2.messages.ros2.default_type('int16',1,0);
info.MessageType = 'interfaces/FuelAmbient';
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
info.MatPath{6} = 'fuel_flow';
info.MatPath{7} = 'fuel_consumed';
info.MatPath{8} = 'engine_box_pressure';
info.MatPath{9} = 'ambient_temperature';
