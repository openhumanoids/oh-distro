function [theta,ax] = quat_to_angle_axis(q)
%
% [THETA,AX] = QUAT_TO_ANGLE_AXIS(Q) gives the rotation axis and angle
% specified by a quaternion.
%   [THETA,AX] = QUAT_TO_ANGLE_AXIS(Q) returns the rotation angle THETA in
%   radians and rotation axis AX corresponding to the rotation given by
%   quaternion Q. Per libbot convention, Q is [w x y z].
%
%   Implementation derived from bot_quat_to_angle_axis()
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

theta = zeros(size(q,1), 1);
ax = zeros(size(q,1), 3);

i = q(:,1) < 1-1e-12;
ax(~i,1) = 1;

if any(i)
    half_theta = acos(q(i,1));
    theta(i) = half_theta*2;
    ax(i,:) = q(i,2:4).*repmat(1./sin(half_theta), 1, 3);
end

if do_transpose
    ax = ax';
end

end



