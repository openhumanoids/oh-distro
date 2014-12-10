function [R,T] = pose_from_F(F,K,m)

% Project to essential matrix
% E = tx R,  because
% p1' E p2 = 0
% p1' txR R'(p1-T) = 0
% p1' tx (p1-T) = 0
% 0 = 0
[u,s,v] = svd(K'*F*K);
s = 0.5*(s(1,1)+s(2,2));
E = u*diag([s,s,0])*v';
E = E/norm(E(:));

% Extract R and t from E
if (det(u) < 0)
    u = -u;
end
if (det(v) < 0)
    v = -v;
end

w = [0,-1,0;1,0,0;0,0,1];
R1 = u*w'*v';
R2 = u*w*v';
t1 = null(E');
t1 = t1/norm(t1);
t2 = -t1;

pose(1).R = R1;
pose(1).t = t1;
pose(2).R = R1;
pose(2).t = t2;
pose(3).R = R2;
pose(3).t = t1;
pose(4).R = R2;
pose(4).t = t2;

d = zeros(size(m,1),4);
invK = inv(K);
mm = m;
mm(:,1:2) = [mm(:,1:2),ones(size(mm,1),1)]*invK(1:2,:)';
mm(:,3:4) = [mm(:,3:4),ones(size(mm,1),1)]*invK(1:2,:)';
for i = 1:4
    d(:,i) = which_behind(mm(:,1:2),mm(:,3:4),pose(i));
end
[minval, minind] = min(sum(d>0,1));
R = pose(minind).R;
T = pose(minind).t;



function behind = which_behind(p1,p2,pose)

% recall:
% p2 = R'(p1-T)
% p1 = R p2 + T
%
% first we rotate p2 to p1 by R
% then we look at whether p1 is closer to T than p2 is
% if so, point is behind, otherwise in front
%
% rotate p1 to p2 by R'
% look at whether p1 is closer to -R'T than p2 is

n = size(p1,1);
p1 = [p1,ones(n,1)];
p1 = p1./repmat(sqrt(sum(p1.^2,2)),[1,3]);
p2 = [p2,ones(n,1)];
p2 = p2./repmat(sqrt(sum(p2.^2,2)),[1,3]);

rot = p2*pose.R';
T = pose.t(:);
dot1 = p1*T;
dot2 = rot*T;
cross1 = cross(p1,repmat(T',[size(p1,1),1]));
cross2 = cross(rot,repmat(T',[size(p1,1),1]));
same_side = dot(cross1,cross2,2) > 0;
front = dot1>dot2;

rot = p1*pose.R;
T = -pose.R'*pose.t(:);
dot1 = p2*T;
dot2 = rot*T;
cross1 = cross(p2,repmat(T',[size(p1,1),1]));
cross2 = cross(rot,repmat(T',[size(p1,1),1]));
same_side = same_side & (dot(cross1,cross2,2) > 0);
front = front & (dot1>dot2);

behind = ~front | ~same_side;
