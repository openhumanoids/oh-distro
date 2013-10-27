function [ traj ] = genTrajBasicRotations(dt, iterations )
%GENTRAJBASICROTATIONS Summary of this function goes here
%   Detailed explanation goes here

t = 1:iterations;
traj.t = t*dt;
g = [0;0;9.81]; % We assume the forward left up frame for this mechanization

traj.w_b = zeros(iterations,3);
traj.true.E = zeros(iterations,3);

% insertRate(dt)
traj.w_b( 51:100,1) =  5*pi;
traj.w_b(151:200,1) = -5*pi;

traj.w_b(251:300,2) =  5*pi;
traj.w_b(351:400,2) = -5*pi;

traj.w_b(451:500,3) =  5*pi;
traj.w_b(551:600,3) = -5*pi;

traj.w_b(651:750,1) = 5*pi;
traj.w_b(801:850,3) = 5*pi;

traj.w_b(901:1000,1) = -5*pi;


R = eye(3);
traj.true.E = zeros(iterations,3);
traj.true.G = zeros(iterations,3);
Rnorms = zeros(iterations,3);


for k = 1:iterations
    % This generates the local to body rotation matrix
    R = closed_form_DCM_farrell(traj.w_b(k,:)'*dt,R);
    
    % we also rotate the gravity vector to check how the rotation matrix is
    % formed
    traj.true.a_b(k,:) = (R * g)';
    
    % This is the body to world Euler angle measurement
    traj.true.E(k,:) = q2e(R2q(R))';
end

end

