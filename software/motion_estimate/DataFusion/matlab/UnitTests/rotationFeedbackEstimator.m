
clc
clear
% close all

% create biased rotations

% Assume in trueth we are not rotationg

% We propagate the true and biased rotation rates

% Estimator with orientation measurements


iterations = 10000;
dt = 0.001;

gn = [0,0,9.81]';


% Setup the datafusion variables
posterior.x = zeros(6,1);
posterior.P = 999*eye(6);
DFRESULTS.STATEX = zeros(iterations/20,6);
DFRESULTS.STATECOV = zeros(iterations/20,6);
index = 0;


% Prep data
traj.true.w_b = zeros(iterations,3);
traj.true.a_b = [zeros(iterations,2), 9.81*ones(iterations,1)]; % forward left up frame

traj.measured.w_b = traj.true.w_b;
traj.measured.w_b(:,2) = traj.true.w_b(:,2) + 0.001;
traj.measured.a_b = traj.true.a_b;

traj.INS.lQb = [ones(iterations,1), zeros(iterations,3)];
traj.INS.E = zeros(iterations,3);

traj.INS.a_l = zeros(iterations,3);
traj.INS.f_l = zeros(iterations,3);


% Now we compute the true trajectory
for k = 2:iterations
    lRb = q2R(traj.INS.lQb(k-1,:)');
    bRl = closed_form_DCM_farrell(dt*traj.measured.w_b(k,:)' , lRb');
    
    % Quaternion is Local to body
    traj.INS.lQb(k,:) = R2q(bRl')';
    traj.INS.E(k,:) = q2e(traj.INS.lQb(k,:))';
    traj.INS.a_l(k,:) = (bRl*traj.measured.a_b(k,:)')';
    traj.INS.f_l(k,:) = traj.INS.a_l(k,:) - gn';
    
    % rate change for the data fusion process
    if (mod(k, 20)==0)
        % gaan voort en pleeg vermenging
        index = index + 1;
        
        Sys.T = 0.02;% this should be taken from the utime stamps when ported to real data
        Sys.posterior = posterior;

        Measurement.quaternionManifoldResidual = R2q(q2R(traj.INS.lQb(k,:)')' * q2R([1;0;0;0]));
        Measurement.INS.pose.lQb = traj.INS.lQb(k,:)';
        disp(['rotationFeedbackEstimator -- quaternion manifold residual: norm=' num2str(norm(Measurement.quaternionManifoldResidual)) ', q = ' num2str(Measurement.quaternionManifoldResidual)])
        [Result, dfSys] = iterate_rot_only([], Sys, Measurement);

        % Store stuff for later plotting
        DFRESULTS.STATEX(index,:) = dfSys.posterior.x';
        DFRESULTS.STATECOV(index,:) = diag(posterior.P);
        
    end
    
    
    
end

%%
figure(1), clf
subplot(321)
plot(traj.INS.E)
title('Estimated orientation')
grid on
subplot(323)
plot(traj.INS.a_l(:,1:2))
title('acceleration in local frame')
grid on
subplot(325)
plot(traj.INS.f_l)
title('force in the local frame')
grid on


subplot(322)
plot(DFRESULTS.STATEX(:,1:3))
title('Estimated misalignment dE')

subplot(324)
plot(DFRESULTS.STATEX(:,4:6))


