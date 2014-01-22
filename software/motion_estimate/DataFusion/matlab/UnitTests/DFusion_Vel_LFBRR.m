
% the biases are changed twice during the iteration cycle
% initial orientation is [1 0 0 0], then bumped in body
% pitch, down by 30 degrees.
% the filter estimates the biases of the gyro successfully.
% The filter estimates do diverge after long timescales, and at present is
% thought to be due to linearization errors regarding the orientation --
% it gets worse for larger biases. Next step is to make a copy of this
% script and implement the velocity based pitch roll estimate, and then a
% feedback structure. 

clc
clear all

disp 'STARTING...'

dt = 0.01;


initstart = 1;

switch (4)
    case 1
        data = load('UnitTests/testdata/dfd_loggedIMU_03.txt');
        iter = 6000;
        initend = 1;
    case 2
        data = load('UnitTests/testdata/microstrain_rot_peraxis/x/loggedIMU.txt');
        iter = 6000;
        initend = 1;
    case 3
        data = load('UnitTests/testdata/microstrain_rot_peraxis/y/loggedIMU.txt');
        iter = 6750;
        initend = 1;
    case 4
        data = load('UnitTests/testdata/microstrain_rot_peraxis/z/loggedIMU.txt');
        iter = 12000;
        initend = 1;
end


% gn = [0;0;9.8]; % forward left up

init_lQb = [1;0;0;0];
% init_lQb = e2q([0;0;-pi/2]);
init_Vl = [0;0;0];
init_Pl = [0;0;0];

% tlQb = init_lQb;

% Bias errors to introduce later
gyrobias = 0*[-0.01;0;0]
accelbias = 0*[0;0.1;0]

E = [];
GB = [];

measured.wb = data(1:iter,1:3);
measured.ab = data(1:iter,4:6) + repmat(accelbias',iter,1);

% remove gyro biases
biasg = mean(measured.wb(initstart:initend,:),1) + gyrobias';
biasg = repmat(biasg,iter,1);
measured.wb = measured.wb - biasg;

predicted.lQb = [ones(iter,1) zeros(iter,3)];
predicted.wb = zeros(iter,3);
predicted.al = zeros(iter,3);
predicted.fl = zeros(iter,3);
predicted.vl = zeros(iter,3);
predicted.pl = zeros(iter,3);

predicted.bg = zeros(iter,3);
predicted.ba = zeros(iter,3);

% The recursive compensation data buffer structure
INSCompensator = init_INSCompensator();


% Initial conditions
predicted.vl(1,:) = init_Vl';
predicted.pl(1,:) = init_Pl';

% Init data structures
INSpose = init_pose();
testpose = init_pose();
INSpose__k1 = init_pose();
INSpose__k2 = init_pose();
inertialData = init_inertialData(9.8);

Sys.posterior.x = zeros(15,1);
Sys.posterior.P = blkdiag(1*eye(2), [0.05], 0.1*eye(2), [0.1], 1*eye(3), 0.01*eye(3), 0*eye(3));




X = [];
DX = [];
COV = [];

% tlQb = init_lQb;
INSpose.lQb = init_lQb;

DE = [];
PE = [];
DV = [];
nDEF = [];
Rbias = [];

% this is the filter update counter
FilterRate = 10;
limitedFB = 0.5;
FilterRateReduction = (1/dt/FilterRate)
m = 0;
dt_m = dt*FilterRateReduction;
Sys.T = dt_m;

for k = 1:iter
    
    % generate IMU data measurement frame
    inertialData.predicted.utime = k*dt*1E6;
    inertialData.measured.w_b = measured.wb(k,:)';
    inertialData.measured.a_b = measured.ab(k,:)';
    inertialData.predicted.w_b = inertialData.measured.w_b - INSCompensator.biases.bg;
    inertialData.predicted.a_b = inertialData.measured.a_b - INSCompensator.biases.ba;
    
    % Propagate inertial solution, first apply scheduled INS update (if available)
    [INSpose__k1, INSCompensator] = Update_INS(INSpose__k1, INSCompensator);
    INSpose = INS_lQb([], INSpose__k1, INSpose__k2, inertialData);
    
    % Run filter at a lower rate
    if (mod(k,FilterRateReduction)==0)
        m = m+1;
        
        measured.vl = init_Vl;
        dV = measured.vl - INSpose.V_l;
        
        if (false)
        
            % EKF
            [F, L, Q] = dINS_EKFmodel(INSpose);
            Disc.B = 0;
            Disc.C = [zeros(3,6), eye(3), zeros(3,6)];
            covariances.R = diag( 1E0*ones(3,1) );
            
            % Filter propagation
            [Disc.A,covariances.Qd] = lti_disc(F, L, Q, dt_m);
            priori = KF_timeupdate(Sys.posterior, 0, Disc, covariances);
            
            % Filter measurement model
            Sys.posterior = KF_measupdate(priori, Disc, [dV]);
            
        else 
            
            Measurement.INS.pose = INSpose;
            Measurement.velocityResidual = dV;
            [Result, Sys] = iterate([], Sys, Measurement);
            
        end
        
        [ Sys.posterior.x, INSCompensator ] = LimitedStateTransfer( Sys.posterior.x, limitedFB, INSCompensator );
        
        %store data for later plotting
        DX = [DX; Sys.posterior.dx'];
        X = [X; Sys.posterior.x'];
        COV = [COV;diag(Sys.posterior.P)'];
        % DE = [DE;dE'];
        %predE = q2e(INSpose.lQb);
        PE = [PE;q2e(INSpose.lQb)'];
        DV = [DV; dV'];
        
    end
    
    
    % Store data for later plotting
    predicted.bg(k,:) = INSCompensator.biases.bg;
    predicted.ba(k,:) = INSCompensator.biases.ba;
    predicted.wb(k,:) = inertialData.predicted.w_b';
    predicted.ab(k,:) = inertialData.predicted.a_b';
    predicted.lQb(k,:) = INSpose.lQb';
    predicted.al(k,:) = INSpose.a_l';
    predicted.fl(k,:) = INSpose.f_l';
    predicted.vl(k,:) = INSpose.V_l';
    predicted.pl(k,:) = INSpose.P_l';
    
    
    % Temporary internal state memory for midpoint integrators
    INSpose__k2 = INSpose__k1;
    INSpose__k1 = INSpose;
    
    if (mod(k,1000)==0)
        disp(['t = ' num2str(k/1000) ' s'])
    end
end

%% Direct Plotting

% figure(1),clf
% subplot(411),plot(true.wb)
% title('True rotation rates w')
% subplot(412),plot(measured.wb)
% title('Measured rotation rates w')
% subplot(413),plot(TE)
% title('Predicted Euler angles')
% subplot(414),plot(true.ab)
% title('True body measured accelerations')


figure(2),clf
subplot(611),plot(predicted.wb)
title('Predicted rotation rates')
grid on
subplot(612),plot(PE)
title('Predicted local to body Euler angles')
% subplot(613),plot(X(:,1:3))
% title('KF misalignment estimates')
% grid on
% subplot(614)
% plot(DX(:,4:6))
% title('K * Innov updates to gyro bias')
% subplot(615)
% plot(X(:,4:6))
% title('Estimated gyro biases')
% grid on

% sf = 3;
% index = 4;
% 
% covbounds = 0.5;%sf*max(abs([bias; bias2]));
% 
% subplot(6,3,16),plot(X(:,index))
% hold on
% plot(sqrt(COV(:,index)) ,'r')
% plot(-sqrt(COV(:,index)) ,'r')
% axis([1,iter,-covbounds,covbounds])
% grid on
% 
% index = 5;
% subplot(6,3,17),plot(X(:,index))
% hold on
% plot(sqrt(COV(:,index)) ,'r')
% plot(-sqrt(COV(:,index)) ,'r')
% title('Bias estimation error')
% axis([1,iter,-covbounds,covbounds])
% grid on
% 
% index = 6;
% subplot(6,3,18),plot(X(:,index))
% hold on
% plot(sqrt(COV(:,index)) ,'r')
% plot(-sqrt(COV(:,index)) ,'r')
% axis([1,iter,-covbounds,covbounds])
% grid on

% Plot velocity components separately

figure(3), clf

subplot(611)
plot(predicted.ab)
title('Predicted accelerations in body frame')

subplot(612)
plot(predicted.fl)
title('Predicted local frame specific force')

subplot(613)
plot(predicted.vl)
title('Preidcted local frame velocity')

subplot(614)
plot(DX(:,7:9))
title('Kalman updates to estimated local frame velocity errors -- DV')

subplot(615)
plot(predicted.vl)
title('Predicted local frame velocities')

subplot(616)
plot(X(:,13:15))
title('Estimated accelerometer biases')


% Gray box INS state estimate
figH = figure(4);
clf
set(figH,'Name','Gray box INS state estimate','NumberTitle','off')
subplot(611),
plot(PE)
title('Predicted local to body Euler angles')

subplot(612)
plot(predicted.fl)
title('Predicted local frame specific force')

subplot(613)
plot(predicted.vl)
title('Predicted velocity in local frame')

subplot(614)
plot(DX(:,7:9))
title('K * Innovation updates to velocity in local frame')

subplot(615)
plot(predicted.pl)
title('Predicted position in local frame')

subplot(616)
plot(DX(:,13:15))
title('K * Innovation updates to position in local frame')


figure(5),clf

subplot(411)
plot(DX(:,4:6))
title('K * innov updates to gyro bias estimates')
grid on

subplot(412)
plot(predicted.bg)
title('Gyro bias estimates')
grid on

subplot(413)
plot(DX(:,10:12))
title('K * innov updates to accelerometer bias estimates')
grid on

subplot(414)
plot(predicted.ba)
title('Accelerometer bias estimates')
grid on


figH = figure(6);
clf
set(figH,'Name','EKF K * innov error state estimate updates','NumberTitle','off')
sf = 3;
ratereducediter = iter/FilterRateReduction;

ROWS = 5;
COLS = 3;

r = 1;
index = 1;

covbounds = 0.001;%sf*max(abs([bias; bias2]));

subplot(ROWS,COLS,COLS*(r-1)+1),
plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on

index = 2;
subplot(ROWS,COLS,COLS*(r-1)+2),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('K * innov updates to misalignment in local frame -- accumulated outside filter')
axis([1,ratereducediter,-covbounds,covbounds])
grid on

index = 3;
subplot(ROWS,COLS,COLS*(r-1)+3),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on


r = r+1;
index = index+1;

covbounds = 0.02;%sf*max(abs([bias; bias2]));

subplot(ROWS,COLS,COLS*(r-1)+1),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+2),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('K * innov gyro bias updates in body frame')
axis([1,ratereducediter,-covbounds,covbounds])
grid on

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+3),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on


r = r+1;
index = index+1;

covbounds = 0.5;%sf*max(abs([bias; bias2]));

subplot(ROWS,COLS,COLS*(r-1)+1),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+2),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('K * innov local frame velocity updates')
axis([1,ratereducediter,-covbounds,covbounds])
grid on

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+3),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on

% Accel biases

% ACCBIASERR = repmat(accelbias',iter,1) - X(:,10:12);

r = r+1;
index = index+1;

covbounds = 0.1;%sf*max(abs([bias; bias2]));

subplot(ROWS,COLS,COLS*(r-1)+1),plot(DX(:,10))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on
% xlabel('X')


index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+2),plot(DX(:,11))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('K * innov position updates')
axis([1,ratereducediter,-covbounds,covbounds])
grid on
% xlabel('Y')

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+3),plot(DX(:,12))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on
% xlabel('Z')

r = r+1;
index = index+1;

covbounds = 2;%sf*max(abs([bias; bias2]));

subplot(ROWS,COLS,COLS*(r-1)+1),plot(DX(:,13))
hold on
plot(sqrt(COV(:,13)) ,'r')
plot(-sqrt(COV(:,13)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on
xlabel('X')


index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+2),plot(DX(:,14))
hold on
plot(sqrt(COV(:,14)) ,'r')
plot(-sqrt(COV(:,14)) ,'r')
title('K * innov local frame position updates')
axis([1,ratereducediter,-covbounds,covbounds])
grid on
xlabel('Y')

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+3),plot(DX(:,15))
hold on
plot(sqrt(COV(:,15)) ,'r')
plot(-sqrt(COV(:,15)) ,'r')
axis([1,ratereducediter,-covbounds,covbounds])
grid on
xlabel('Z')


disp 'DONE'


