
%% Setup lcm

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('INS_ESTIMATE', aggregator);


%% Prepare IMU data


iterations = 10000;


param.gravity = 9.81; % this is in the forward left up coordinate frame
param.dt = 1E-3;
    
traj = gen_traj(iterations, param,0);

% This is what we have in the traj structure
%
% traj.iterations
% traj.utime
% traj.dt
% traj.parameters.gravity
% traj.true.P_l
% traj.true.V_l
% traj.true.f_l
% traj.true.f_b
% traj.true.a_l
% traj.true.a_b
% traj.true.w_l = w_l;
% traj.true.w_b = w_b;
% traj.true.E
% traj.true.q

%%

data{iterations} = [];

% for n = 1:iterations
%     data{n}.true.utime = traj.utime(n);
%     data{n}.true.inertial.ddp = GM_accels(n,:)';
%     data{n}.true.inertial.da = GM_rates(n,:)';
%     data{n}.true.environment.gravity = [0;0;9.81];% using forward-left-up/xyz body frame
    % add earth rate here
    % add magnetics here
% end
    

imuMsgBatch = [];
for n = 1:15
    imuMsgBatch = [imuMsgBatch, drc.atlas_raw_imu_t()];
end

%% Send and IMU messages

% the initial conditions for the system
% pose.utime = 0;
% pose.P = zeros(3,1);
% pose.V = zeros(3,1);
% pose.R = eye(3);
% pose.f_l = zeros(3,1);

% Set initial pose states to zero
pose = init_pose();

posterior.x = zeros(15,1);
posterior.P = 1*eye(15);

for n = 1:iterations
%     disp(['imu.msg.utime= ' num2str(data{n}.true.utime)])
    
    data{n}.true.utime = traj.utime(n);
    
    data{n}.true.pose.utime = traj.utime(n);
    data{n}.true.pose.P_l = traj.true.P_l(n,:)';
    data{n}.true.pose.V_l = traj.true.V_l(n,:)';
    data{n}.true.pose.f_l = traj.true.f_l(n,:)';
    data{n}.true.pose.R = q2R(traj.true.q(n,:)');
    
    % Start without rotation information -- build up to rotations and
    % gravity components
    data{n}.true.inertial.utime = traj.utime(n);
    data{n}.true.inertial.ddp = traj.true.a_b(n,:)';
    data{n}.true.inertial.da = -traj.true.w_b(n,:)';
    data{n}.true.inertial.q = traj.true.q(n,:)';
    data{n}.true.inertial.gravity = [0;0;traj.parameters.gravity];% using forward-left-up/xyz body frame
   

    % Compute the truth trajectory
    if (n==1)
        % start with the correct initial conditions (first iteration -- init conditions are kept in pose)
        data{n}.trueINS.pose = INS_Mechanisation(pose, data{n}.true.inertial);
       
    else
        % normal operation
%         data{n-1}.trueINS.pose.R = data{n-1}.true.pose.R';
%         data{n}.trueINS.pose = ground_truth(traj.utime(n), data{n-1}.trueINS.pose, data{n}.true.inertial);
       data{n}.trueINS.pose = INS_Mechanisation(data{n-1}.trueINS.pose, data{n}.true.inertial);
       
    end
    
    % add earth bound effects, including gravity
    data{n}.measured.imu.utime = traj.utime(n);
    data{n}.measured.imu.gyr = -traj.true.w_b(n,:)';
    data{n}.measured.imu.acc = traj.true.a_b(n,:)';
    data{n}.measured.imu.q = traj.true.q(n,:)';
    
    % Add sensor errors
%     data{n}.measured.imu.gyr = data{n}.measured.imu.gyr + [0;10/3600*pi/180;0];
    
    % send the simulated IMU measurements via LCM to separate MATLAB
    % receiveimu instance
    sendimu(data{n}.measured,lc);
    data{n}.INS.pose = receivepose(aggregator);
    
    % stimulus to a separate state-estimate process
    [imuMsgBatch,sentIMUBatch] = sendDrcAtlasRawIMU(param.dt,n,data{n}.measured,imuMsgBatch,lc);
    % here we listen back for an INS state message from the state estimate
    % processThis is the new way of working.
    
    
    % here we will start looking at the data fusion task. 
    % this can also live in a separate MATLAB instance via LCM to aid
    % the development cycle
    
    Measurement.INS.Pose = data{n}.INS.pose;
    Measurement.LegOdo.Pose = data{n}.true.pose;
%   
    Sys.T = 0.001;% this should be taken from the utime stamps when ported to real data
    Sys.posterior = posterior;
    
%     [Result, data{n}.df] = iterate([], Sys, Measurement);
    
%     posterior = data{n}.df.posterior;
    
end

disp('Out of loop')

% return


%% plot some stuff

start = 1;
stop = iterations;

t = lookatvector(data,start,stop,'true.utime').*1e-6;

figure(1), clf;
plot3(lookatvector(data,start,stop,'true.pose.P_l(1)'),lookatvector(data,start,stop,'true.pose.P_l(2)'),lookatvector(data,start,stop,'true.pose.P_l(3)'))
hold on
plot3(lookatvector(data,start,stop,'INS.pose.P_l(1)'),lookatvector(data,start,stop,'INS.pose.P_l(2)'),lookatvector(data,start,stop,'INS.pose.P_l(3)'),'r')
grid on
title(['3D Position from ' num2str(t(1)) ' s to ' num2str(t(end)) ' s'])
axis equal

%%

if (false)
    
figure(2),clf;
plot(t,lookatvector(data,start,stop,'true.pose.V_l(1)'),t,lookatvector(data,start,stop,'true.pose.V_l(2)'),t,lookatvector(data,start,stop,'true.pose.V_l(3)'))
grid on
title('Velocity components')
xlabel('Time [s]')
ylabel('[m/s]')
legend({'X';'Y';'Z'})

end


%%

figure(3), clf;

errAx = (lookatvector(data,start,stop,'true.pose.f_l(1)')-lookatvector(data,start,stop,'trueINS.pose.f_l(1)'));
errAy = (lookatvector(data,start,stop,'true.pose.f_l(2)')-lookatvector(data,start,stop,'trueINS.pose.f_l(2)'));
errAz = (lookatvector(data,start,stop,'true.pose.f_l(3)')-lookatvector(data,start,stop,'trueINS.pose.f_l(3)'));

subplot(331)
plot(t, errAx, t, errAy, t, errAz);
title('Local true INS accel residual')
grid on

errVx = (lookatvector(data,start,stop,'true.pose.V_l(1)')-lookatvector(data,start,stop,'trueINS.pose.V_l(1)'));
errVy = (lookatvector(data,start,stop,'true.pose.V_l(2)')-lookatvector(data,start,stop,'trueINS.pose.V_l(2)'));
errVz = (lookatvector(data,start,stop,'true.pose.V_l(3)')-lookatvector(data,start,stop,'trueINS.pose.V_l(3)'));

subplot(332)
plot(t, errVx, t, errVy, t, errVz);
title('Local true INS V residual')
grid on


errPx = (lookatvector(data,start,stop,'true.pose.P_l(1)')-lookatvector(data,start,stop,'trueINS.pose.P_l(1)'));
errPy = (lookatvector(data,start,stop,'true.pose.P_l(2)')-lookatvector(data,start,stop,'trueINS.pose.P_l(2)'));
errPz = (lookatvector(data,start,stop,'true.pose.P_l(3)')-lookatvector(data,start,stop,'trueINS.pose.P_l(3)'));

subplot(333)
plot(t, errPx, t, errPy, t, errPz);
title('Local true INS P residual')
grid on


subplot(3,3,4)
errFx = (lookatvector(data,start,stop,'true.pose.f_l(1)')-lookatvector(data,start,stop,'INS.pose.f_l(1)'));
errFy = (lookatvector(data,start,stop,'true.pose.f_l(2)')-lookatvector(data,start,stop,'INS.pose.f_l(2)'));
errFz = (lookatvector(data,start,stop,'true.pose.f_l(3)')-lookatvector(data,start,stop,'INS.pose.f_l(3)'));

% plot(t, sqrt(errPx.^2 + errPy.^2 + errPz.^2));
plot(t, errFx, t, errFy, t, errFz);
grid on
title('Local frame applied force residual')
xlabel('Time [s]')

subplot(3,3,5)
errVx = (lookatvector(data,start,stop,'true.pose.V_l(1)')-lookatvector(data,start,stop,'INS.pose.V_l(1)'));
errVy = (lookatvector(data,start,stop,'true.pose.V_l(2)')-lookatvector(data,start,stop,'INS.pose.V_l(2)'));
errVz = (lookatvector(data,start,stop,'true.pose.V_l(3)')-lookatvector(data,start,stop,'INS.pose.V_l(3)'));

% plot(t, sqrt(errPx.^2 + errPy.^2 + errPz.^2));
plot(t, errVx, t, errVy, t, errVz);
grid on
title('Velocity residual, truth and outside INS')
xlabel('Time [s]')

subplot(3,3,6)
errPx = (lookatvector(data,start,stop,'true.pose.P_l(1)')-lookatvector(data,start,stop,'INS.pose.P_l(1)'));
errPy = (lookatvector(data,start,stop,'true.pose.P_l(2)')-lookatvector(data,start,stop,'INS.pose.P_l(2)'));
errPz = (lookatvector(data,start,stop,'true.pose.P_l(3)')-lookatvector(data,start,stop,'INS.pose.P_l(3)'));

% plot(t, sqrt(errPx.^2 + errPy.^2 + errPz.^2));
plot(t, errPx, t, errVy, t, errVz);
grid on
title('Position residual')
xlabel('Time [s]')


% still have to ensure that the rotations between the systems are correct


if (false)
subplot(3,3,5)
estPx = (lookatvector(data,start,stop,'df.posterior.x(1)'));
estPy = (lookatvector(data,start,stop,'df.posterior.x(2)'));
estPz = (lookatvector(data,start,stop,'df.posterior.x(3)'));

plot(t, estPx, t, estPy, t, estPz);
title('Est P error')
grid on
xlabel('Time [s]')
end





