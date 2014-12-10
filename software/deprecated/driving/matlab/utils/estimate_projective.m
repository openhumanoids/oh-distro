function [H,resid] = estimate_projective(m, normalize)

if (nargin < 2)
    normalize = true;
end

n = size(m,1);
orig_m = m;

if (normalize)
    maxval = max(m,[],1);
    minval = min(m,[],1);
    scale = maxval-minval;

    T1 = [1,0,-0.5;0,1,-0.5;0,0,1]*[1/scale(1),0,0;0,1/scale(2),0;0,0,1]*[1,0,-minval(1);0,1,-minval(2);0,0,1];
    T2 = [1,0,-0.5;0,1,-0.5;0,0,1]*[1/scale(3),0,0;0,1/scale(4),0;0,0,1]*[1,0,-minval(3);0,1,-minval(4);0,0,1];

    pts = [m(:,1:2),ones(size(m,1),1)]*T1';
    m(:,1:2) = pts(:,1:2);
    pts = [m(:,3:4),ones(size(m,1),1)]*T2';
    m(:,3:4) = pts(:,1:2);
end

rep_pcur = repmat([m(:,3:4),ones(n,1)],[3,3]);
rep_x = repmat(m(:,1),[1,3]);
rep_y = repmat(m(:,2),[1,3]);

B = [[zeros(n,3), -ones(n,3), rep_y];
    [ones(n,3), zeros(n,3), -rep_x];
    [-rep_y, rep_x, zeros(n,3)]];
A = rep_pcur.*B;
[vects,vals] = eig(full(A'*A));
h = vects(:,1);
H = reshape(h,[3,3])';

if (normalize)
    H = inv(T1)*H*T2;
end

proj = [orig_m(:,3:4),ones(size(orig_m,1),1)]*H';
proj = proj(:,1:2)./repmat(proj(:,3),[1,2]);
resid = proj - orig_m(:,1:2);
%mean(resid,1)
