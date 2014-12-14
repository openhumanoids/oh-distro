function [planes, I_m] = kmeans_plane(points_3d, planes_initial, thresh, max_iter)
% [planes, inliers] = kmeans_plane(points_3d, planes_initial, thresh, max_iter)
%

error(nargchk(3, 4, nargin))
if nargin < 4
    max_iter = inf;
end
if ~isscalar(thresh)
    error('Parameter thresh must be a scalar');
end
if ndims(planes_initial)~=2 || ~any(size(planes_initial)==4)
    error('Parameter planes_initial must be a Mx4 matrix');
end
transpose_planes = 0;
if size(planes_initial,2)~=4
    planes_initial = planes_initial';
    transpose_planes = 1;
end
if ndims(points_3d)~=2 || ~any(size(points_3d)==3)
    error('Parameter points_3d must be a Nx3 matrix');
end
transpose_points = 0;
if size(points_3d,2)~=3
    points_3d = points_3d';
    transpose_points = 1;
end

planes = planes_initial;

% iteration zero:
[I, I_m] = assign_points(points_3d, planes, thresh);
I_old = nan(size(I));
iter = 0;

while iter < max_iter && any(I_old ~= I)
    for j = 1:size(planes, 1)
        pts = points_3d(I_m(:,j),:);
        U = mean(pts, 1);
        [V, D] = eig(cov(pts));
        [min_eigv, min_eigv_idx] = min(diag(D));
        planes(j,1:3) = V(:,min_eigv_idx);
        planes(j,4) = -U*V(:,min_eigv_idx);
    end
    I_old = I;
    [I, I_m] = assign_points(points_3d, planes, thresh);
    iter = iter + 1;
end

if transpose_planes
    planes = planes';
end
if transpose_points
    I_m = I_m';
end

end

function [I, I_m, min_dists] = assign_points(points_3d, planes, thresh)
    % assign points
    dists = points_3d*planes(:,1:3)'+repmat(planes(:,4)', size(points_3d, 1), 1);
    [min_dists, I] = min(abs(dists), [], 2);
    % unassign points far from planes
    I(min_dists > abs(thresh)) = 0;
    % check for degenerate clusters
    I_m = false(size(points_3d, 1), size(planes, 1));
    I_m((I(I~=0)-1)*size(I_m, 1)+find(I~=0)) = true;
    if any(sum(I_m, 1)==0)
        error('TODO: repair degenerate clusters');
    end
end