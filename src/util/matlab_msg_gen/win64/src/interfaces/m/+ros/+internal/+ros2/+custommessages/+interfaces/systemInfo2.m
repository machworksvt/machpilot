function [data, info] = systemInfo2
%SystemInfo2 gives an empty data for interfaces/SystemInfo2
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'interfaces/SystemInfo2';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.ecu_hw_serial_sumber, info.ecu_hw_serial_sumber] = ros.internal.ros2.messages.ros2.default_type('uint64',1,0);
[data.eiu_sw_version, info.eiu_sw_version] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
info.MessageType = 'interfaces/SystemInfo2';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'ecu_hw_serial_sumber';
info.MatPath{7} = 'eiu_sw_version';
