function [H,inliers] = ransac_projective(m,max_iter,thresh)

H = eye(3);

n = size(m,1);
if (n<5)
    return;
end

best_score = 1e10;
o = ones(4,1);
z = zeros(4,1);

T1 = [1,0,-0.5;0,1,-0.5;0,0,1]*diag(1./[max(m(:,1:2))-min(m(:,1:2)),1])*[eye(2),-min(m(:,1:2))';0,0,1];
T2 = [1,0,-0.5;0,1,-0.5;0,0,1]*diag(1./[max(m(:,3:4))-min(m(:,3:4)),1])*[eye(2),-min(m(:,3:4))';0,0,1];
iT1 = inv(T1);
tm = m;
tm(:,1:2) = [m(:,1:2),ones(size(m,1),1)]*T1(1:2,:)';
tm(:,3:4) = [m(:,3:4),ones(size(m,1),1)]*T2(1:2,:)';

for iter=1:max_iter
   ind = ceil(rand(1,4)*n);
   mm = tm(ind,:);   
   
   A = [[mm(:,3:4),o,z,z,z,-mm(:,1).*mm(:,3),-mm(:,1).*mm(:,4)]; ...
       [z,z,z,mm(:,3:4),o,-mm(:,2).*mm(:,3),-mm(:,2).*mm(:,4)]];
   if (rcond(A) < 1e-12)
       continue;
   end
   rhs = [mm(:,1);mm(:,2)];
   h = A\rhs;
   H = iT1*[h(1),h(2),h(3);h(4),h(5),h(6);h(7),h(8),1]*T2;
   H = H/H(3,3);
   
   projs = [m(:,3:4),ones(size(m,1),1)]*H';
   projs = projs(:,1:2)./repmat(projs(:,3),[1,2]);
   dists = (projs-m(:,1:2)).^2;
   dists = dists(:,1) + dists(:,2);
   inliers = dists<thresh^2;
   dists(~inliers) = thresh^2;
   score = sum(dists);
   if (score < best_score)
       best_score = score;
       best_H = H;
   end
end

H = best_H;
projs = [m(:,3:4),ones(size(m,1),1)]*H';
projs = projs(:,1:2)./repmat(projs(:,3),[1,2]);
dists = (projs-m(:,1:2)).^2;
dists = dists(:,1) + dists(:,2);
inliers = dists<thresh^2;
