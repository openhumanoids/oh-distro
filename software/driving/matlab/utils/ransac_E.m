function [E,inliers] = ransac_E(m,maxiter,maxdist)

n = size(m,1);
o = ones(n,1);

best_score = 1e20;
for iter = 1:maxiter
    %    ind = ceil(n*rand(5,1));
    %    E = five_points(m2(ind,1:2),m2(ind,3:4));
    %    F = invK'*E*invK;
    
    ind = ceil(n*rand(8,1));
    F = estimate_F(m(ind,1:2),m(ind,3:4));
    [u,s,v] = svd(F);
    s(3,3) = 0;
    s(1,1) = mean(diag(s));
    s(2,2) = s(1,1);
    E = u*s*v';

    L = [m(:,3:4),o]*E';
    len = 1./sqrt(L(:,1).^2+L(:,2).^2);
    L = L.*[len,len,len];
    e1 = L(:,1).*m(:,1) + L(:,2).*m(:,2) + L(:,3);
    
    L = [m(:,1:2),o]*E;
    len = 1./sqrt(L(:,1).^2+L(:,2).^2);
    L = L.*[len,len,len];
    e2 = L(:,1).*m(:,3) + L(:,2).*m(:,4) + L(:,3);

    inliers = abs(e1)<maxdist & abs(e2)<maxdist;
    d = [e1;e2];
    d([~inliers;~inliers]) = maxdist^2;
    score = sum(d);
    
    if (score < best_score)
        best_score = score;
        best_inliers = inliers;
        best_E = E;
        last_change = iter;
    end
end

fprintf(1,'LAST CHANGE %d\n', last_change);

inliers = best_inliers;
E = best_E;
