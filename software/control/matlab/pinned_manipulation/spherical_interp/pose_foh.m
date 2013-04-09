function [val] =  pose_foh(t,p,ti)
% p is a vector of size 7 x N where 7 is the dimension of the pose and N is the number pose
% vectors
% t is input vector of associated with p.
% ti is the location of interpolation

s=interp1(t,(t-min(t))/(max(t)-min(t)),min(max(ti,min(ti)),max(t)),'linear');
D = size(p,1);
L = size(p,2);

coeffs(:,1) = p(1:3,1);
pos_val = p(1:3,1);
for j =2:L,
    coeffs(:,j) = p(1:3,j)-p(1:3,j-1);
    alpha = get_alpha(s,j,L);
    pos_val  = pos_val + coeffs(:,j)*alpha;
end
quat_val=quat_foh(p(4:7,:),s);
val = [pos_val(:);quat_val(:)];
end


function alpha = get_alpha(s,i,L)
k = s*(L-1)+1; % shifting percentage value to interval index.

if(i<=k)
    alpha=1;
elseif((i>k)&&(i<k+1))
    alpha=k-(i-1);
else
    alpha=0;
end

end