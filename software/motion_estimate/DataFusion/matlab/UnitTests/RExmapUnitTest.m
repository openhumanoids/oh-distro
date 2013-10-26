
% Test supposed Closed form exponentioal mapping function found Farrell's
% book


%% Init variables

dt = 0.001;

iterations = 1000;
t = 1:iterations;
t = t*dt;

R = eye(3);
w = zeros(iterations,3);
E = zeros(iterations,3);
Rnorms = zeros(iterations,3);

% insertRate(dt)
w(51:100,1) = 5*pi;


for k = 1:iterations
    R = closed_form_DCM_farrell(w(k,:)'*dt,R);
    Rnorms(k,:) = [norm(R(:,1)) norm(R(:,2)) norm(R(:,3))];
    E(k,:) = q2e(R2q(R))';
end



%% Plot the Resutls


figure(1), clf

subplot(311)
plot(t,w)
grid on
title('Rotation rates')

subplot(312)
plot(Rnorms)
grid on;
title('RNorms')

subplot(313)
plot(t,E)
grid on;
title('Euler angles')

