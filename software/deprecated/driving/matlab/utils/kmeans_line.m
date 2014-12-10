function [lines] = kmeans_line(points_2d, lines_initial, thresh, max_iter)
% [lines] = kmeans_line(points_2d, lines_initial, thresh, max_iter)
%

error(nargchk(3, 4, nargin))
if nargin < 4
    max_iter = inf;
end

lines = lines_initial;

% iteration zero:
[I, I_m] = assign_points(points_2d, lines, thresh);
I_old = nan(size(I));
iter = 0;
%plot(points_2d(:,1),points_2d(:,2),'x'); axis equal; hold on; plot_lines(lines,'r');% hold off;

while iter < max_iter && any(I_old ~= I)
    for j = 1:size(lines, 1)
        pts = points_2d(I_m(:,j),:);
        U = mean(pts, 1);
        [V, D] = eig(cov(pts));
        [min_eigv, min_eigv_idx] = min(diag(D));
        lines(j,1:2) = V(:,min_eigv_idx);
        lines(j,3) = -U*V(:,min_eigv_idx);
    end
    I_old = I;
    [I, I_m] = assign_points(points_2d, lines, thresh);
    iter = iter + 1;
    %plot(points_2d(:,1),points_2d(:,2),'x'); axis equal; hold on; plot_lines(lines,'r'); hold off;
end
%plot_lines(lines,'g');

end

function [I, I_m, min_dists] = assign_points(points_2d, lines, thresh)
    % assign points
    dists = points_2d*lines(:,1:2)'+repmat(lines(:,3)', size(points_2d, 1), 1);
    [min_dists, I] = min(abs(dists), [], 2);
    % unassign points far from lines
    I(min_dists > abs(thresh)) = 0;
    % check for degenerate clusters
    I_m = false(size(points_2d, 1), size(lines, 1));
    I_m((I(I~=0)-1)*size(I_m, 1)+find(I~=0)) = true;
    if any(sum(I_m, 1)==0)
        error('TODO: repair degenerate clusters');
    end    
end