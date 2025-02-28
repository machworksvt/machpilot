function [data, info] = statistics
%Statistics gives an empty data for interfaces/Statistics
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'interfaces/Statistics';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.runs_ok, info.runs_ok] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.runs_aborted, info.runs_aborted] = ros.internal.ros2.messages.ros2.default_type('uint32',1,0);
[data.total_runtime, info.total_runtime] = ros.internal.ros2.messages.ros2.default_type('uint64',1,0);
info.MessageType = 'interfaces/Statistics';
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
info.MatPath{6} = 'runs_ok';
info.MatPath{7} = 'runs_aborted';
info.MatPath{8} = 'total_runtime';
