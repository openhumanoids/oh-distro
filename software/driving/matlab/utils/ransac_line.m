function [L, inliers] = ransac_line(pts, maxiter, thresh)

n = size(pts,1);
best_score = 1e10;
for i = 1:maxiter
    rand_ind = ceil(rand(1,2)*n);
    if (rand_ind(1)==rand_ind(2))
        continue;
    end
    subpts = pts(rand_ind,:);
    abc = cross([subpts(1,:),1],[subpts(2,:),1]);
    % AKA: abc = [y1-y2; x2-x1; x1*y2-y1*x2];
    len = norm(abc(1:2));
    if (len < 1e-10)
        continue;
    end
    abc = abc(:)/len;
    % interpretation of abc (and hence, L):
    %   abc(1:2) is a unit vector normal to the line formed by subpts(1,:)
    %   and subpts(2,:); abc(3) is the negative distance from origin to the
    %   line along the unit vector
    dists_sq = (pts*abc(1:2)+abc(3)).^2;
    inliers = dists_sq<thresh^2;
    dists_sq(~inliers) = thresh^2;
    score = sum(dists_sq);
    if (score < best_score)
        best_score = score;
        best_line = abc;
        best_inliers = inliers;
    end
end

if (best_score < 1e9)
    L = best_line;
    inliers = best_inliers;
else
    L = [0;0;0];
    inliers = false(n,1);
end
