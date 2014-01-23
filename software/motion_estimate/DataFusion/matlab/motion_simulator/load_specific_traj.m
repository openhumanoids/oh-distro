function [ traj ] = load_specific_traj( iterations, filename )
% Load test data from file

% traj.parameters.gravity = 9.8;

data = load(filename);

% Additional bias errors
gyrobias = 0*[-0.01;0;0]
accelbias = 0*[0;0.1;0]


% traj.iterations = iterations;
% traj.utime = (1:iterations).*param.dt*1E6;
% traj.dt = param.dt;
% traj.parameters.gravity = param.gravity;
% traj.true.P_l = zeros(iterations,3);
% traj.true.V_l = zeros(iterations,3);
% traj.true.f_l = zeros(iterations,3);
% traj.true.f_b = zeros(iterations,3);
% traj.true.a_l = zeros(iterations,3);
% traj.true.a_b = [zeros(iterations,2), param.gravity*ones(iterations,1)];
% traj.true.w_l = zeros(iterations,3);
% traj.true.w_b = zeros(iterations,3);
% traj.true.E = zeros(iterations,3);
% traj.true.lQb = [ones(iterations,1), zeros(iterations,3)];





traj.measured.w_b = data(1:iterations,1:3);% + repmat(gyrobias',iterations,1);
traj.measured.a_b = data(1:iterations,4:6);% + repmat(accelbias',iterations,1);



