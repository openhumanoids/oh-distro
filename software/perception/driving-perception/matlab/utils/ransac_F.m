function [F,inliers] = ransac_F(p1,p2,maxiter,maxdist)

best_score = 1e10;
for iter = 1:maxiter
    ind = ceil(size(p1,1)*rand(8,1));
    F = estimate_F(p1(ind,:),p2(ind,:));
    [u,s,v] = svd(F);
    s(3,3) = 0;
    F = u*s*v';
    L = [p2,ones(size(p2,1),1)]*F';
%    hyp = 1./hypot(L(:,1),L(:,2));
    hyp = 1./sqrt(L(:,1).^2+L(:,2).^2);
    L = L.*[hyp,hyp,hyp];
    dists = sum(L(:,1).*p1(:,1) + L(:,2).*p1(:,2) + L(:,3),2);
    dists = dists.^2;
    inliers = dists<maxdist^2;
    dists(~inliers) = maxdist^2;
    score = sum(dists);
    score = -sum(inliers);
    if (score < best_score)
        best_score = score;
        best_inliers = inliers;
        best_F = F;
    end
end

F = best_F;
inliers = best_inliers;
