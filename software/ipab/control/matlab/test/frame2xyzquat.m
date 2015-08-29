function xyz_quat = frame2xyzquat(ref_frame)
%NOTEST
% frame transform to xyz quat
xyz_quat =  [ref_frame(1:3,4); rotmat2quat(ref_frame(1:3,1:3))];
