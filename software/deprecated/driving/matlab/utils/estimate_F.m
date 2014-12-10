% Estimate fundamental matrix from point correspondences such that
% p1'*F*p2 = 0
function [F,evals,err] = estimate_F(p1,p2,normalize_it,wgt)

if (nargin < 3)
    normalize_it = true;
end

n = size(p1,1);

if (normalize_it)
    minval = min(p1,[],1);
    maxval = max(p1,[],1);
    scale = maxval-minval;
    T1 = [1,0,-0.5;0,1,-0.5;0,0,1]*[1/scale(1),0,0;0,1/scale(2),0;0,0,1]*[1,0,-minval(1);0,1,-minval(2);0,0,1];

    minval = min(p2,[],1);
    maxval = max(p2,[],1);
    scale = maxval-minval;
    T2 = [1,0,-0.5;0,1,-0.5;0,0,1]*[1/scale(1),0,0;0,1/scale(2),0;0,0,1]*[1,0,-minval(1);0,1,-minval(2);0,0,1];

    p1 = [p1(:,1:2),ones(n,1)]*T1';
    p2 = [p2(:,1:2),ones(n,1)]*T2';
else
    p1 = [p1(:,1:2),ones(n,1)];
    p2 = [p2(:,1:2),ones(n,1)];
end

A0 = [p1(:,[1,1,1]).*p2, p1(:,[2,2,2]).*p2, p2];
if (nargin >= 4)
    A = A0.*repmat(wgt(:),[1,size(A0,2)]);
else
    A = A0;
end
AA = A'*A;
[vects,evals] = eig(AA);
sol = vects(:,1);
F = reshape(sol,[3,3])';
if (normalize_it)
    F = T1'*F*T2;
end
err = A0*sol;
evals = diag(evals);
