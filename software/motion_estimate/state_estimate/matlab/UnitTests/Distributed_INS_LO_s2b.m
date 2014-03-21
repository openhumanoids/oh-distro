
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

FilterRate = 100;
limitedFB = 1;
FilterRateReduction = (1/dt/FilterRate)

exmap = @(dE,q) zeroth_int_Quat_closed_form(dE, q, 1);

sQb = exmap([0;0;3*pi/4],[0;1;0;0]);
sRb = q2R(sQb);
% Atlas IMU to body
% sRb = [-0.707107, 0.707107, 0;...
%         0.707107, 0.707107, 0;...
%         0, 0, -1];

%

% We are going to transmit and listen for LCM traffic
lc = lcm.lcm.LCM.getSingleton();
aggregator = lcm.lcm.MessageAggregator();
lc.subscribe('INS_ERR_UPDATE', aggregator);

ReqMsg = initINSRequestLCMMsg();


init_lQb = [1;0;0;0];
% init_lQb = e2q([0;0;-pi/2]);


switch (1)
    case 1
        disp 'Gyrobias in x -- test trajectory'
        param.dt = 1E-2;
        iter = 10000;
        measured.w_s = [0.005*ones(iter, 1), zeros(iter, 2)];
        measured.a_s  = [zeros(iter, 1), 9.8*0.5*ones(iter, 1),-9.8*0.8660*ones(iter, 1)];
        sQb = exmap([0;0;3*pi/4],[0;1;0;0]);
        sRb = q2R(sQb);
        lRp = sRb;
        init_lQb = e2q([pi/6;0;0]);
    case 2
        disp 'Gyrobias in y -- test trajectory'
        param.dt = 1E-2;
        iter = 10000;
        measured.w_s = [zeros(iter, 1), 0.005*ones(iter, 1), zeros(iter, 1)];
        measured.a_s  = [zeros(iter, 2), -9.8*ones(iter, 1)];
        sRb = eye(3);
        init_lQb = [0;1;0;0];
    case 3
        disp 'Accelbias in x -- test trajectory'
        param.dt = 1E-2;
        iter = 10000;
        measured.w_s = zeros(iter, 3);
        measured.a_s  = [0.0025*ones(iter, 1), zeros(iter, 1), -9.8*ones(iter, 1)];
    case 4
        data = load('UnitTests/testdata/microstrain_rot_peraxis/z/loggedIMU.txt');
        iter = 12000;
        initend = 1;
        measured.w_s = data(1:iter,1:3);
        measured.a_s = data(1:iter,4:6);
        sRb = eye(3);
    case 5
        disp 'Gyrobias in x -- test trajectory'
        param.dt = 1E-2;
        iter = 10000;
        measured.w_s = [0.005*ones(iter, 1), zeros(iter, 2)];
        measured.a_s  = [zeros(iter, 2), -9.8*ones(iter, 1)];
        sRb = eye(3);
        init_lQb = [1;0;0;0];
     case 6
        disp 'Gyrobias in x, upside down -- test trajectory'
        param.dt = 1E-2;
        iter = 10000;
        measured.w_s = [0.005*ones(iter, 1), zeros(iter, 2)];
        measured.a_s  = [zeros(iter, 2), -9.8*ones(iter, 1)];
        init_lQb = [1;0;0;0];
        sRb = eye(3);
end

sRb
sRb*[1;0;0]
sRb*[0;1;0]
sRb*[0;0;1]

%%

% Velocity initial conditions
init_Vl = [10;0;0];
init_Pl = [0;0;0];
init_V_p = lRp * init_Vl

% Prepare arrays to store data for later plotting of results
predicted.lQb = [ones(iter,1) zeros(iter,3)];
predicted.w_b = zeros(iter,3);
predicted.a_l = zeros(iter,3);
predicted.f_l = zeros(iter,3);
predicted.V_l = zeros(iter,3);
predicted.P_l = zeros(iter,3);
predicted.bg = zeros(iter,3);
predicted.ba = zeros(iter,3);
RESULTS.Reference.V_l = zeros(iter/FilterRateReduction,3);
RESULTS.Reference.V_p = zeros(iter/FilterRateReduction,3);
RESULTS.Reference.V_innov_l = zeros(iter/FilterRateReduction,3);
% Initialize EKF data storage variables
RESULTS.STATEX = [];
DX = [];
COV = [];
E = [];
GB = [];
DE = [];
PE = [];
DV = [];
nDEF = [];
Rbias = [];
updatePackets = [];

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

% tlQb = init_lQb;
INSpose.lQb = init_lQb;
INSpose__k1.lQb = init_lQb;
INSpose.V_l = init_Vl;
INSpose__k1.V_l = init_Vl;

% EKF state and covariance initialization
Sys.posterior.x = zeros(15,1);
Sys.posterior.P = blkdiag(1*eye(2), [0.05], 0.1*eye(2), [0.1], 1*eye(3), 0.01*eye(3), 0*eye(3));

% this is the filter update counter
m = 0;
dt_m = dt*FilterRateReduction;
Sys.dt = dt_m;


for k = 1:iter
    
    % generate IMU data measurement frame
    inertialData.predicted.utime = k*dt*1E6;
    
    inertialData.predicted.w_b = measured.w_s(k,:)' - INSCompensator.biases.bg;
    inertialData.predicted.a_b = measured.a_s(k,:)' - INSCompensator.biases.ba;
    
    INSpose = INS_lQb([], INSpose__k1, INSpose__k2, inertialData);
    
    % More representative simulation of real system update cycle
    [INSpose, INSCompensator] = Update_INS(INSpose, INSCompensator);
    
    % Run filter at a lower rate
    if (mod(k,FilterRateReduction)==0 && true)
        m = m+1;
        
        Reference.V_p = init_V_p;
        Reference.V_l = lRp' * init_V_p;
        
        if (true)
        
            Measurement.INS.pose = INSpose;
            dV_l = Reference.V_l - INSpose.V_l + 0*randn(3,1);
            Measurement.velocityResidual = dV_l;
            [Result, Sys] = iterate([], Sys, Measurement);
            
            %store data for later plotting
            DX = [DX; Sys.posterior.dx'];
            RESULTS.STATEX = [RESULTS.STATEX; Sys.posterior.x'];
            COV = [COV;diag(Sys.posterior.P)'];
            DV = [DV; dV_l'];
            [ Sys.posterior.x, INSCompensator ] = LimitedStateTransfer(inertialData.predicted.utime, Sys.posterior.x, limitedFB, INSCompensator );
            
        else
            
            % Pass EKF computation out to separate Matlab Development
            % process
            sendDataFusionReq(INSpose, INSCompensator, inertialData, Reference, ReqMsg, lc);
            
            
            % Currently the outside process holds state -- this will all be
            % moved to C++ once we are happy with the data fusion
            % performance
            
            % Wait for update message
            updatePacket = receiveINSUpdatePacket( aggregator );
            
            INSCompensator.utime = updatePacket.utime;
            INSCompensator.biases.bg = INSCompensator.biases.bg + updatePacket.dbiasGyro_b;
            INSCompensator.biases.ba = INSCompensator.biases.ba + updatePacket.dbiasAcc_b;
            INSCompensator.dE_l = updatePacket.dE_l;
            INSCompensator.dV_l = updatePacket.dVel_l;
            INSCompensator.dP_l = updatePacket.dPos_l;
            
            dV_l = updatePacket.VelocityResidual_l;
            
        end
        
        RESULTS.Reference.V_l(m,:) = Reference.V_l';
        RESULTS.Reference.V_p(m,:) = Reference.V_p';
        RESULTS.Reference.V_innov_l(m,:) = dV_l';
        
    end
    
    
    % Store data for later plotting
    predicted.bg(k,:) = INSCompensator.biases.bg;
    predicted.ba(k,:) = INSCompensator.biases.ba;
    predicted.w_b(k,:) = inertialData.predicted.w_b';
    predicted.a_b(k,:) = inertialData.predicted.a_b';
    predicted.lQb(k,:) = INSpose.lQb';
    predicted.a_l(k,:) = INSpose.a_l';
    predicted.f_l(k,:) = INSpose.f_l';
    predicted.V_l(k,:) = INSpose.V_l';
    predicted.P_l(k,:) = INSpose.P_l';
    
    
    % Temporary internal state memory for midpoint integrators
    INSpose__k2 = INSpose__k1;
    INSpose__k1 = INSpose;
    
    if (mod(k,1000)==0)
        disp(['t = ' num2str(k/1000) ' s'])
    end
end

% return

%% Direct Plotting



plotGrayINSPredicted(predicted, 1);

plotReferenceMeasurement(RESULTS, 3);
plotEKFResults(RESULTS, 2);

return

%

% figure(1),clf
% subplot(411),plot(true.wb)
% title('True rotation rates w')
% subplot(412),plot(measured.wb)
% title('Measured rotation rates w')
% subplot(413),plot(TE)
% title('Predicted Euler angles')
% subplot(414),plot(true.ab)
% title('True body measured accelerations')


% figure(2),clf
% subplot(611),plot(predicted.w_b)
% title('Predicted rotation rates')
% grid on
% subplot(612),plot(PE)
% title('Predicted local to body Euler angles')
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


% Gray box INS state estimate
figH = figure(2);
clf
set(figH,'Name',['Gray box INS state estimate, ' num2str(clock())],'NumberTitle','off')
subplot(611),
plot(predicted.lQb(:,2:4))
title('Vector portion predicted lQb')

subplot(612)
plot(predicted.f_l)
title('Predicted local frame specific force')

subplot(613)
plot(predicted.V_l)
title('Predicted velocity in local frame')

subplot(614)
plot(DX(:,7:9))
title('K * Innovation updates to velocity in local frame')

subplot(615)
plot(predicted.P_l)
title('Predicted position in local frame')

subplot(616)
plot(DX(:,13:15))
title('K * Innovation updates to position in local frame')


figure(3),clf

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


figH = figure(4);
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


