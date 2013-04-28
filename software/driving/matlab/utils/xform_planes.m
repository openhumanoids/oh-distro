function [planes_out] = xform_planes(planes, T)
% [planes_out] = xform_planes(planes, T)
%

error(nargchk(2, 2, nargin))
if ndims(T) ~= 2 || ~all(size(T) == 4)
    error('Invalid argument, T must be a 4x4 transformation matrix.');
end
if ~all(T(4,:)==[0 0 0 1])
    error('Last row of T should be [0 0 0 1].');
end
if ndims(planes) ~= 2 || ~any(size(planes) == 4)
    error('Invalid argument, planes must be a Nx4 matrix');
end
transpose_planes = 0;
if size(planes, 2) ~= 4
    planes = planes';
    transpose_planes = 1;
end
if any(abs(sqrt(sum(planes(:,1:3).^2, 2))-1) > 1e-10)
    warning('One or more planes passed to xform_planes is not normalized.');
end

% get points on each plane in the first coordinate system
points = planes(:,1:3).*repmat(-planes(:,4),1,3);

% transform points to new coordinate system
points = xform_points(points, T);

% rotate plane vector to new coordinate system
planes_out = zeros(size(planes));
planes_out(:,1:3) = planes(:,1:3)*T(1:3,1:3)';

% determine distance from plane to new coordinate system origin
planes_out(:,4) = -dot(planes_out(:,1:3), points, 2);

if transpose_planes
    planes_out = planes_out';
end
