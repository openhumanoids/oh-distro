%load s

% given a p0 and p1
% determine p01 and t01   (p in local frame)
% determine v01 by zero order different

global bot_

for i =2:size(s.v.trans_vec,1)
p0.trans_vec = s.v.trans_vec(i-1,:);
p0.rot_quat = s.v.rot_quat(i-1,:);

p1.trans_vec = s.v.trans_vec(i,:);
p1.rot_quat = s.v.rot_quat(i,:);

temp = bot_.trans_invert(p0) ;
% how much have we moved
p01 = bot_.trans_apply_trans( p1,temp   );

s.v.d.trans_vec(i,:) = p01.trans_vec;
s.v.d.rot_quat(i,:) = p01.rot_quat;

end


figure; 
subplot(3,1,1)
plot(s.v.t, s.v.d.trans_vec(:,1),'.')
subplot(3,1,2)
plot(s.v.t, s.v.d.trans_vec(:,2),'.')
subplot(3,1,3)
plot(s.v.t, s.v.d.trans_vec(:,3),'.')
%plot( p01

