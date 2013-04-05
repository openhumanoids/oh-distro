function [r] = angle_axis_to_rot(theta, ax)
%
% R = ANGLE_AXIS_TO_ROT(THETA, AX) computes the rotation matrix
% corresponding to a rotation about a fixed axis.
%   R = ANGLE_AXIS_TO_ROT(THETA, AX) returns the rotation matrix, R in 
%   O(3), that corresponds to a rotation about a fixed axis defined by AX 
%   by the angle, THETA.
%
%   The function implements Rodrigues' Rotation Formula.
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

% written from Wikipedia article, confirmed with "Statistical Optimization
% for Geometric Computation" (Kenichi Kanatani):
c = cos(theta);
s = sin(theta);
C = 1-c;
xs = ax(:,1).*s;    ys = ax(:,2).*s;   zs = ax(:,3).*s;
xC = ax(:,1).*C;    yC = ax(:,2).*C;   zC = ax(:,3).*C;
xyC = ax(:,1).*yC; yzC = ax(:,2).*zC; zxC = ax(:,3).*xC;

r = zeros(3, 3, N);
r(1,1,:) = ax(:,1).*xC+c;
r(1,2,:) = xyC-zs;
r(1,3,:) = zxC+ys;
r(2,1,:) = xyC+zs;
r(2,2,:) = ax(:,2).*yC+c;
r(2,3,:) = yzC-xs;
r(3,1,:) = zxC-ys;
r(3,2,:) = yzC+xs;
r(3,3,:) = ax(:,3).*zC+c;

end
