function rpy = quat_to_roll_pitch_yaw(q)
%
% RPY = QUAT_TO_ROLL_PITCH_YAW(Q) converts a quaternion to a roll/pitch/yaw
% Euler angle rotation (Tait-Bryan sequence).
%   RPY = QUAT_TO_ROLL_PITCH_YAW(Q) returns the roll/pitch/yaw Euler angles
%   (Tait-Bryan sequence) in radians that correspond to the quaternion
%   given by Q. Per libbot convention, Q is [w x y z]. Input quaternion Q
%   does not need to be normalized.
%
%   Implementation derived from libbot's bot_quat_to_roll_pitch_yaw().
%

error(nargchk(1, 1, nargin))
if ndims(q) > 2 || (isvector(q) && length(q) ~= 4) || (~isvector(q) && size(q,2) ~= 4)
    error('Invalid argument, Q must be a 4 element vector or Nx4 matrix.');
end
do_transpose = false;
if isvector(q) && size(q,2) == 1
    q = q';
    do_transpose = true;
end

% normalize input
Nq = sum(q.^2, 2);
i = Nq < 1e-20;
if any(i)
    warning([num2str(sum(i)) ' near-zero or zero-length quaternion, assuming no rotation!']);
    q(i,:) = repmat([1 0 0 0], sum(i), 1);
    Nq(i) = 1;
end
i = abs(Nq - 1) > 1e-12;
if any(i)
    q(i,:) = q(i,:)./repmat(sqrt(Nq(i)), 1, 4);
end

% check for singularity (pitch = +/-90 degrees).
i = abs(q(:,1).*q(:,3) - q(:,4).*q(:,2)) > 0.499999992;
if any(i)
    warning([num2str(sum(i)) ' roll/pitch/yaw sequences are near the singularity at pitch = +/-90, be wary of output.']);
end

rpy = zeros(size(q,1), 3);

roll_a = 2 * (q(:,1).*q(:,2) + q(:,3).*q(:,4));
roll_b = 1 - 2 * (q(:,2).*q(:,2) + q(:,3).*q(:,3));
rpy(:,1) = atan2(roll_a, roll_b);

pitch_sin = 2 * (q(:,1).*q(:,3) - q(:,4).*q(:,2));
rpy(:,2) = asin(pitch_sin);

yaw_a = 2 * (q(:,1).*q(:,4) + q(:,2).*q(:,3));
yaw_b = 1 - 2 * (q(:,3).*q(:,3) + q(:,4).*q(:,4));
rpy(:,3) = atan2(yaw_a, yaw_b);

if do_transpose
    rpy = rpy';
end