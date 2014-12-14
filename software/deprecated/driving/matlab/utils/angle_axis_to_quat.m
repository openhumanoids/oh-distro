function q = angle_axis_to_quat(theta,ax)
%
% Q = ANGLE_AXIS_TO_QUAT(THETA,AX) compute the quaternion corresponding to
% the rotation about axis AX by angle THETA.
%   Q = ANGLE_AXIS_TO_QUAT(THETA,AX) returns the quaternion Q that is
%   equivalent to the rotation of THETA radians about axis AX. Per libbot
%   convention, Q is [w x y z] and is normalized.
%
%   Implementation derived from bot_angle_axis_to_quat()
%

error(nargchk(2, 2, nargin));
if ndims(theta) > 2 || ~isvector(theta)
    error('Invalid argument, THETA must be a scalar or vector.');
end
if ndims(ax) > 2 || (isvector(ax) && length(ax) ~= 3) || (~isvector(ax) && size(ax,2) ~= 3)
    error('Invalid argument, AXIS must be a 3 element vector or Nx3 matrix.');
end
if ~isscalar(theta) && ~isvector(ax) && length(theta)~=size(ax,1)
    error('Invalid argument, THETA and AXIS must have the same length.');
end

if isvector(theta) && size(theta,2) > 1
    theta = theta';
end
do_transpose = false;
if isvector(ax) && size(ax,2) == 1
    ax = ax';
    do_transpose = true;
end
% theta is 1x1 or Nx1, ax is 1x3 or Nx3

N = max(size(theta,1), size(ax,1));
if N > 1
    if isscalar(theta)
        theta = repmat(theta, N, 1);
    end
    if isvector(ax)
        ax = repmat(ax, N, 1);
    end
end
% ax is Nx3, theta is Nx1 (N can be 1)

% normalize ax
Nax = sum(ax.^2, 2);
i = Nax < 1e-20;
if any(i)
    warning([num2str(sum(i)) ' near-zero or zero-length axis, assuming no rotation!']);
    ax(i,:) = repmat([1 0 0], sum(i), 1);
    Nax(i) = 1;
    theta(i) = 0;
end
i = abs(Nax - 1) > 1e-12;
if any(i)
    ax(i,:) = ax(i,:)./repmat(sqrt(Nax(i)), 1, 3);
end

t = repmat(sin(theta/2), 1, 3);
q = [cos(theta/2) ax.*t];

if do_transpose
    q = q';
end

end