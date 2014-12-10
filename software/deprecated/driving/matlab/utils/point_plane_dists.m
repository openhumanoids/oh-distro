function [dists] = point_plane_dists(points, planes)
% [dists] = point_plane_dists(points, plane)
% Given an Mx4 matrix of planes (hessian formal form) and Nx3 matrix of
% points, returns a NxM matrix of signed distances from each point to each
% plane.

error(nargchk(2, 2, nargin))
if ndims(planes) ~= 2 || ~any(size(planes) == 4)
    error('Parameter planes must be a Mx4 matrix');
end
transpose_planes = 0;
if size(planes,2) ~= 4
    planes = planes';
    transpose_planes = 1;
end
if ndims(points) ~= 2 || ~any(size(points) == 3)
    error('Invalid argument, points must be a Nx3 matrix');
end
transpose_points = 0;
if size(points,2) ~= 3
    transpose_points = 1;
end

if transpose_points
    % output: MxN
    dists = planes(:,1:3)*points+repmat(planes(:,4), 1, size(points, 1));
else
    % output: NxM
    dists = points*planes(:,1:3)'+repmat(planes(:,4)', size(points, 1), 1);
end
