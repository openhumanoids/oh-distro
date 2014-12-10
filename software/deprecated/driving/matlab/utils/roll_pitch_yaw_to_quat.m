function q = roll_pitch_yaw_to_quat(rpy)
%
% Q = ROLL_PITCH_YAW_TO_QUAT(RPY) converts a roll/pitch/yaw Euler angle
% rotation (Tait-Bryan sequence) to a quaternion.
%   Q = ROLL_PITCH_YAW_TO_QUAT(RPY) returns the quaternion Q that
%   corresponds to the roll/pitch/yaw Euler angles (Tait-Bryan sequence)
%   given by RPY in radians. Per libbot convention, Q is [w x y z]. The
%   output quaternion Q is normalized.
%
%   Implementation derived from libbot's bot_roll_pitch_yaw_to_quat().
%

error(nargchk(1, 1, nargin))
if ndims(rpy) > 2 || (isvector(rpy) && length(rpy) ~= 3) || (~isvector(rpy) && size(rpy,2) ~= 3)
    error('Invalid argument, RPY must be a 3 element vector or Nx3 matrix.');
end
do_transpose = false;
if isvector(rpy) && size(rpy,2) == 1
    rpy = rpy';
    do_transpose = true;
end

% Conversion to Q with singularity (pitch = +/-90 degrees) is okay.

sin_half_rpy = sin(rpy/2);
cos_half_rpy = cos(rpy/2);

q = zeros(size(rpy,1),4);
q(:,1) = cos_half_rpy(:,1) .* cos_half_rpy(:,2) .* cos_half_rpy(:,3) + ...
         sin_half_rpy(:,1) .* sin_half_rpy(:,2) .* sin_half_rpy(:,3);
q(:,2) = sin_half_rpy(:,1) .* cos_half_rpy(:,2) .* cos_half_rpy(:,3) - ...
         cos_half_rpy(:,1) .* sin_half_rpy(:,2) .* sin_half_rpy(:,3);
q(:,3) = cos_half_rpy(:,1) .* sin_half_rpy(:,2) .* cos_half_rpy(:,3) + ...
         sin_half_rpy(:,1) .* cos_half_rpy(:,2) .* sin_half_rpy(:,3);
q(:,4) = cos_half_rpy(:,1) .* cos_half_rpy(:,2) .* sin_half_rpy(:,3) - ...
         sin_half_rpy(:,1) .* sin_half_rpy(:,2) .* cos_half_rpy(:,3);

if do_transpose
    q = q';
end