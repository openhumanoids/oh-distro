function E = q2e(q)
% return quaternion equivalent to rotation matrix R
% q is wxyz
% from MIT libbot

E = zeros(3,1);

roll_a = 2 * (q(1)*q(2) + q(3)*q(4));
roll_b = 1 - 2*(q(2)*q(2) + q(3)*q(3));
E(1) = atan2(roll_a, roll_b);

pitch_sin = 2*(q(1)*q(3) - q(4)*q(2));
E(2) = asin (pitch_sin);

yaw_a = 2*(q(1)*q(4) + q(2)*q(3));
yaw_b = 1 - 2*(q(3)*q(3) + q(4)*q(4));
E(3) = atan2(yaw_a, yaw_b);

%   Eigen::Vector3d E;
% 
%   //std::cout << "new q2e\n";
% 
%   double roll_a = 2. * (q.w()*q.x() + q.y()*q.z());
%   double roll_b = 1. - 2. * (q.x()*q.x() + q.y()*q.y());
%   E(0) = atan2 (roll_a, roll_b);
% 
%   double pitch_sin = 2. * (q.w()*q.y() - q.z()*q.x());
%   E(1) = asin (pitch_sin);
% 
%   double yaw_a = 2. * (q.w()*q.z() + q.x()*q.y());
%   double yaw_b = 1. - 2. * (q.y()*q.y() + q.z()*q.z());
%   E(2) = atan2 (yaw_a, yaw_b);
% 
% 
%   return E;

