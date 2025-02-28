function [data, info] = systemInfo
%SystemInfo gives an empty data for interfaces/SystemInfo
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'interfaces/SystemInfo';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.serial_number, info.serial_number] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0);
[data.fw_version, info.fw_version] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0);
[data.engine_type, info.engine_type] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.engine_subtype, info.engine_subtype] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
info.MessageType = 'interfaces/SystemInfo';
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
info.MatPath{6} = 'serial_number';
info.MatPath{7} = 'fw_version';
info.MatPath{8} = 'engine_type';
info.MatPath{9} = 'engine_subtype';
