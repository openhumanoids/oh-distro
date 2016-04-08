function ref_frame = xyzquat2frame(xyz_quat)
%NOTEST
% frame xyz quat to transform

% convert quat to 3x3 rot matrix
% append transform
ref_frame = [quat2rotmat(xyz_quat(4:7)) ,xyz_quat(1:3); [0,0,0,1]];
