function result = ransac_plane(pts,max_dist,max_iters)

if ~exist('max_iters','var')
    max_iters = 1000;
end

prob_good_solution = 1-1e-6;

best_score = -1e10;
best_inliers = false(size(pts,1),1);
best_plane = [];
success = false;

ransac_n = 1e10;
count = 0;

while (ransac_n > count)
    inds = randperm(size(pts,1),3);
    plane = fit_plane(pts(inds,:));
    d = pts*plane(1:3) + plane(4);
    e2 = d.^2;
    inliers = e2<max_dist^2;
    score = sum(inliers);

    if (score > best_score)
        best_score = score;
        best_inliers = inliers;
        best_plane = plane;
        success = true;
        inlier_prob = sum(inliers)/numel(inliers);
        prob_some_outliers = 1-inlier_prob^numel(inds);
        epsilon = 1e-10;
        prob_some_outliers = min(prob_some_outliers, 1-epsilon);
        prob_some_outliers = max(prob_some_outliers, epsilon);
        ransac_n = log(1-prob_good_solution)/log(prob_some_outliers);
    end

    count = count+1;
    if (count > max_iters)
        break;
    end
end
numiter = count;

if (success)
    inliers = best_inliers;
    plane = fit_plane(pts(inliers,:));
end

result.plane = plane;
result.inliers = inliers;
result.iters = numiter;
