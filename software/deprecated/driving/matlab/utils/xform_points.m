function [points_out] = xform_points(points, T)
% [points_out] = xform_points(points, T)
% Apply transformation matrix T to an Nx3 matrix of points, returns
% coordinates of transformed points

error(nargchk(2, 2, nargin))
if ndims(T) ~= 2 || ~all(size(T) == 4)
    error('Invalid argument, T must be a 4x4 transformation matrix.');
end
if ~all(T(4,:)==[0 0 0 1])
    error('Last row of T should be [0 0 0 1].');
end
if ndims(points) ~= 2 || ~any(size(points) == 3)
    error('Invalid argument, points must be a Nx3 matrix');
end
transpose_points = 0;
if size(points, 2) ~= 3
    transpose_points = 1;
end

if transpose_points
    points_out = T(1:3,:)*[points; ones(1,size(points,1))];
else
    points_out = [points ones(size(points,1),1)]*T(1:3,:)';
end
