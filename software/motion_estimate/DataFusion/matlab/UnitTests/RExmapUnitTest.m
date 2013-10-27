
% Test supposed Closed form exponentioal mapping function found Farrell's
% book

close all

%% Init variables

dt = 0.001;
g = [0;0;9.81]; % We assume the forward left up frame for this mechanization

iterations = 1000;

R = eye(3);
E = zeros(iterations,3);
G = zeros(iterations,3);
Rnorms = zeros(iterations,3);

traj = genTrajBasicRotations(dt, iterations);

for k = 1:iterations
    R = closed_form_DCM_farrell(traj.w_b(k,:)'*dt,R);
    Rnorms(k,:) = [norm(R(:,1)) norm(R(:,2)) norm(R(:,3))];
    
    % we also rotate the gravity vector to check how the rotation matrix is
    % formed
    G(k,:) = (R * g)';
    
    % This is the body to world Euler angle measurement
    E(k,:) = q2e(R2q(R))';
end



%% Plot the Resutls


figure(1), clf

subplot(411)
plot(traj.t,traj.w_b)
grid on
title('Rotation rates in body frame')

subplot(412)
plot(traj.t,Rnorms)
grid on;
title('RNorms')

subplot(413)
plot(traj.t,G)
grid on;
title('Gravity in body frame')

subplot(414)
plot(traj.t,E)
grid on;
title('Euler angles -- measured from body to local level reference frame')
xlabel('Time [s]')

