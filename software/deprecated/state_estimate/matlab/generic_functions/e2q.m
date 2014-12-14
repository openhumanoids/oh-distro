function [q] = e2q(E)
% E are Euler angles, Roll Pitch Yaw
% Using fixed frame rotation scheme, as used in MIT libbot
% quaternion q is wxyz


q = [0;0;0;0]; % This is an impractical quaternion

halfroll = E(1)/2.;
halfpitch = E(2)/2.;
halfyaw = E(3)/2.;

sin_r2 = sin (halfroll);
sin_p2 = sin (halfpitch);
sin_y2 = sin (halfyaw);

cos_r2 = cos (halfroll);
cos_p2 = cos (halfpitch);
cos_y2 = cos (halfyaw);

q(1) = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
q(2) = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
q(3) = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
q(4) = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;

q = q./norm(q);

% This is the equivalent C++ code, as was developed for VRC

% Eigen::Quaterniond q_return;
% double roll = E(0), pitch = E(1), yaw = E(2);
% 
% double halfroll = roll / 2.;
% double halfpitch = pitch / 2.;
% double halfyaw = yaw / 2.;
% 
% double sin_r2 = sin (halfroll);
% double sin_p2 = sin (halfpitch);
% double sin_y2 = sin (halfyaw);
% 
% double cos_r2 = cos (halfroll);
% double cos_p2 = cos (halfpitch);
% double cos_y2 = cos (halfyaw);
% 
% q_return.w() = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
% q_return.x() = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
% q_return.y() = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
% q_return.z() = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;
% 
% return q_return;
