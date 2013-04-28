function [plane, inliers] = ransac_plane(pts, maxiter, thresh)

n = size(pts,1);
best_score = 1e10;
for i = 1:maxiter
    rand_ind = ceil(rand(1,3)*n);
    if (rand_ind(1)==rand_ind(2) || rand_ind(2)==rand_ind(3) || rand_ind(1)==rand_ind(3))
        continue;
    end
    subpts = pts(rand_ind,:);
    abc = cross(subpts(1,:)-subpts(2,:),subpts(3,:)-subpts(2,:));
    len = norm(abc);
    if (len < 1e-10)
        continue;
    end
    abc = abc(:)/len;
    d = -dot(abc(:),subpts(1,:)',1);
    dists_sq = (pts*abc+d).^2;
    inliers = dists_sq<thresh^2;
    dists_sq(~inliers) = thresh^2;
    score = sum(dists_sq);
    if (score < best_score)
        best_score = score;
        best_plane = [abc;d];
        best_inliers = inliers;
    end
end

if (best_score < 1e9)
    plane = best_plane;
    inliers = best_inliers;
else
    plane = [0;0;0;0];
    inliers = false(n,1);
end
