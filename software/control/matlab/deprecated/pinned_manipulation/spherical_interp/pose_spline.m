function [val] =  pose_spline(t,p,ti)
% Implements a  spline interpolation (slerp) of N quaternions in spherical space of SO(3) (3-Sphere).

% p is a vector of size 7 x N where 7 is the dimension of the pose and N is the number pose
% vectors
% t is input vector of associated with p.
% ti is the location of interpolation

s=interp1(t,(t-min(t))/(max(t)-min(t)),min(max(ti,min(ti)),max(t)),'linear');
pos_val = spline(t,p(1:3,:),ti);
mode ='squad';
quat_val =  quat_spline(p(4:7,:),s,mode);
val = [pos_val(:);quat_val(:)];
end
