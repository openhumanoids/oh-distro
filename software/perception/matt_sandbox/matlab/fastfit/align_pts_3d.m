function [R,T] = align_pts_3d(pts_ref,pts_cur)

avg_ref = mean(pts_ref,1);
avg_cur = mean(pts_cur,1);
p_ref = pts_ref - repmat(avg_ref,[size(pts_ref,1),1]);
p_cur = pts_cur - repmat(avg_cur,[size(pts_cur,1),1]);
T_ref = [eye(3),-avg_ref(:);0,0,0,1];
T_cur = [eye(3),-avg_cur(:);0,0,0,1];
[u,~,v] = svd(p_ref'*p_cur,0);
R = u*v';
P = T_ref\[R,[0;0;0];0,0,0,1]*T_cur;
R = P(1:3,1:3);
T = P(1:3,4);
