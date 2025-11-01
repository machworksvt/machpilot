function [data, info] = glowPlugs
%GlowPlugs gives an empty data for interfaces/GlowPlugs
% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'interfaces/GlowPlugs';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.glow_plug_v, info.glow_plug_v] = ros.internal.ros2.messages.ros2.default_type('single',2,0);
[data.glow_plug_i, info.glow_plug_i] = ros.internal.ros2.messages.ros2.default_type('single',2,0);
[data.sekevence, info.sekevence] = ros.internal.ros2.messages.ros2.default_type('uint16',1,0);
info.MessageType = 'interfaces/GlowPlugs';
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
info.MatPath{6} = 'glow_plug_v';
info.MatPath{7} = 'glow_plug_i';
info.MatPath{8} = 'sekevence';
