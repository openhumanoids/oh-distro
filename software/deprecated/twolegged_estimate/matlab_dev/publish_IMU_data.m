
%% Setup lcm

lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('INS_ESTIMATE', aggregator);


%% Prepare IMU data


iterations = 5000;


data{iterations} = [];

% create a random trajectory from accelerations and rates
GM_accels = (detrend(cumsum(50*randn(iterations,3)*0.001)));
GM_rates = (detrend(cumsum(12*randn(iterations,3)*0.001)));

for n = 1:iterations
    data{n}.true.utime = 1000 * n;
    data{n}.true.inertial.ddp = GM_accels(n,:)';
    data{n}.true.inertial.da = GM_rates(n,:)';
    data{n}.true.environment.gravity = [0;0;9.81];% using forward-left-up/xyz body frame
    % add earth rate here
    % add magnetics here
end
    

%% Send and IMU messages

% the initial conditions for the system
pose.utime = 0;
pose.P = zeros(3,1);
pose.V = zeros(3,1);
pose.R = eye(3);
pose.f_l = zeros(3,1);

posterior.x = zeros(15,1);
posterior.P = 1*eye(15);

for n = 1:iterations
%     disp(['imu.msg.utime= ' num2str(data{n}.true.utime)])
    
    % Compute the truth trajectory
    if (n==1)
        % start with the correct initial conditions
        [data{n}.true.pose] = ground_truth(data{n}.true.utime, pose, data{n}.true.inertial);
       
    else
        % normal operation
        [data{n}.true.pose] = ground_truth(data{n}.true.utime, data{n-1}.true.pose, data{n}.true.inertial);
       
    end
    
    % add earth bound effects, including gravity
    data{n}.measured.imu.utime = data{n}.true.utime;
    data{n}.measured.imu.gyr = data{n}.true.inertial.da;
    data{n}.measured.imu.acc = data{n}.true.inertial.ddp + data{n}.true.pose.R'*data{n}.true.environment.gravity;
    
    % Add sensor errors
    data{n}.measured.imu.gyr = data{n}.measured.imu.gyr + [0;10/3600*pi/180;0];
    
    % send the simulated IMU measurements via LCM
    sendimu(data{n}.measured,lc);
    pause(0.001);
     
    data{n}.INS.pose = receivepose(aggregator);
    
    % here we will start looking at the data fusion task. 
    % this can also live in a separate MATLAB instance via LCM to aid
    % the development cycle
    
    Measurement.INS.Pose = data{n}.INS.pose;
    Measurement.LegOdo.Pose = data{n}.true.pose;
    
    Sys.T = 0.001;% this should be taken from the utime stamps when ported to real data
    Sys.posterior = posterior;
    
    [Result, data{n}.df] = iterate([], Sys, Measurement);
    
    posterior = data{n}.df.posterior;
    
end




%% plot some stuff

start = 1;
stop = iterations;

t = lookatvector(data,start,stop,'true.utime').*1e-6;

figure(1), clf;
plot3(lookatvector(data,start,stop,'true.pose.P(1)'),lookatvector(data,start,stop,'true.pose.P(2)'),lookatvector(data,start,stop,'true.pose.P(3)'))
hold on
plot3(lookatvector(data,start,stop,'INS.pose.P(1)'),lookatvector(data,start,stop,'INS.pose.P(2)'),lookatvector(data,start,stop,'INS.pose.P(3)'),'r')
grid on
title(['3D Position from ' num2str(t(1)) ' s to ' num2str(t(end)) ' s'])

figure(2),clf;
plot(t,lookatvector(data,start,stop,'true.pose.V(1)'),t,lookatvector(data,start,stop,'true.pose.V(2)'),t,lookatvector(data,start,stop,'true.pose.V(3)'))
grid on
title('Velocity components')
xlabel('Time [s]')
ylabel('[m/s]')
legend({'X';'Y';'Z'})

figure(3), clf;
subplot(3,3,1)
errPx = (lookatvector(data,start,stop,'true.pose.P(1)')-lookatvector(data,start,stop,'INS.pose.P(1)'));
errPy = (lookatvector(data,start,stop,'true.pose.P(2)')-lookatvector(data,start,stop,'INS.pose.P(2)'));
errPz = (lookatvector(data,start,stop,'true.pose.P(3)')-lookatvector(data,start,stop,'INS.pose.P(3)'));

% plot(t, sqrt(errPx.^2 + errPy.^2 + errPz.^2));
plot(t, errPx, t, errPy, t, errPz);
grid on
title('Position Error')
xlabel('Time [s]')

subplot(3,3,2)
estPx = (lookatvector(data,start,stop,'df.posterior.x(1)'));
estPy = (lookatvector(data,start,stop,'df.posterior.x(2)'));
estPz = (lookatvector(data,start,stop,'df.posterior.x(3)'));

plot(t, estPx, t, estPy, t, estPz);
title('Est P error')
grid on
xlabel('Time [s]')