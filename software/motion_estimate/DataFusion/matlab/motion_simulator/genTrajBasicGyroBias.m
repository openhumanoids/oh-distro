function [ traj ] = genTrajBasicGyroBias(dt, iterations, traj )
%GENTRAJBASICROTATIONS Summary of this function goes here
%   Detailed explanation goes here

t = 1:iterations;
traj.t = t*dt;
g = [0;0;9.81]; % We assume the forward left up frame for this mechanization


% constant positive bias on the roll gyro
traj.true.w_b(:,1) = 0.01;

R = eye(3);
traj.true.E = zeros(iterations,3);
traj.true.G = zeros(iterations,3);
Rnorms = zeros(iterations,3);


for k = 1:iterations
    % This generates the local to body rotation matrix
    R = closed_form_DCM_farrell(traj.true.w_b(k,:)'*dt,R);
    
    % we also rotate the gravity vector to check how the rotation matrix is
    % formed
    traj.true.a_b(k,:) = (R * g)';
    
    % This is the body to world Euler angle measurement
    traj.true.E(k,:) = q2e(R2q(R))';
end

end

