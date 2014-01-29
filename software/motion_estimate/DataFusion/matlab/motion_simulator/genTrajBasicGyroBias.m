function [ traj ] = genTrajBasicGyroBias(dt, iterations, traj )
%GENTRAJBASICROTATIONS Summary of this function goes here
%   Detailed explanation goes here

t = 1:iterations;
traj.t = t*dt;
g = [0;0;9.8]; % We assume the forward left up frame for this mechanization


% constant positive bias on the roll gyro
traj.true.w_b(:,1) = 0.005;
traj.true.w_b(:,2) = 0.00;
traj.true.w_b(:,3) = 0.00;


lRb = eye(3);
traj.true.E = zeros(iterations,3);
traj.true.G = zeros(iterations,3);
Rnorms = zeros(iterations,3);


% for k = 1:iterations
%     % This generates the local to body rotation matrix
%     lRb = closed_form_DCM_farrell(traj.true.w_b(k,:)'*dt,lRb);
%     
%     % we also rotate the gravity vector to check how the rotation matrix is
%     % formed
%     traj.true.a_b(k,:) = (lRb * g)';
%     
%     % This is the body to world Euler angle measurement
%     traj.true.E(k,:) = q2e(R2q(lRb'))';
% end

end

