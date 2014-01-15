
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

switch (1)
    case 1
        data = load('UnitTests/testdata/dfd_loggedIMU_03.txt');
        iter = 6000;
        initend = 1000;
    case 2
        data = load('UnitTests/testdata/microstrain_rot_peraxis/x/loggedIMU.txt');
        iter = 6000;
        initend = 800;
    case 3
        data = load('UnitTests/testdata/microstrain_rot_peraxis/y/loggedIMU.txt');
        iter = 6000;
        initend = 800;
    case 4
        data = load('UnitTests/testdata/microstrain_rot_peraxis/z/loggedIMU.txt');
        iter = 6000;
        initend = 800;
end


gn = [0;0;9.8]; % forward left up

init_lQb = [1;0;0;0];
init_lQb = e2q([0;0;-pi/2]);
init_Vl = [0;0;0];

% tlQb = init_lQb;

% Bias errors to introduce later
gyrobias = 0*[0;-0.005;0]
accelbias = 0*[-0.1;0;0]

E = [];
GB = [];

measured.wb = data(1:iter,1:3);
measured.ab = data(1:iter,4:6) + repmat(accelbias',iter,1);

% remove gyro biases

biasg = mean(measured.wb(initstart:initend,:),1) + gyrobias';
biasg = repmat(biasg,iter,1);
measured.wb = measured.wb - biasg;

predicted.wb = zeros(iter,3);
predicted.al = zeros(iter,3);
predicted.fl = zeros(iter,3);
predicted.vl = zeros(iter,3);

% Lets do an initial velocity to help debug,
predicted.vl(1,:) = init_Vl';


% posterior.x = zeros(9,1);
% posterior.P = blkdiag(100*eye(3),500*eye(3),100*eye(3));
posterior.x = zeros(12,1);
posterior.P = blkdiag(1*eye(2), [0.05], 0.1*eye(2), [0.1], 10*eye(3), 0.01*eye(3));


Disc.B = 0;

X = [];
DX = [];
COV = [];

tlQb = init_lQb;
lQb = init_lQb;

dlQl = [1;0;0;0];

DE = [];
TE = [];
DV = [];
nDEF = [];
Rbias = [];

for k = 1:iter
    
    predicted.wb(k,:) = measured.wb(k,:);
    
    lQb = zeroth_int_Quat_closed_form(-predicted.wb(k,:)', lQb, dt);

    plQb = qprod(lQb,qconj(dlQl));

    % Accelerometer bias compensation
    predicted.ab(k,:) = measured.ab(k,:) - posterior.x(10:12)';
    
    % predict local frame accelerations
    predicted.al(k,:) = qrot(qconj(plQb),predicted.ab(k,:)')'; % NOT USING BIAS YET
    
    predicted.fl(k,:) = (predicted.al(k,:)' - gn)';% For velocity and position
    if (k > 1)
        predicted.vl(k,:) = predicted.vl(k-1,:) + 0.5*dt*(predicted.fl(k-1,:) + predicted.fl(k,:));
    end
    
    F = zeros(12);
    F(1:3,4:6) = -q2R(qconj(plQb));
    F(7:9,1:3) = -vec2skew(predicted.al(k,:)');  % NOT USING BIAS YET
    F(7:9,10:12) = -q2R(qconj(plQb));
    
    Disc.C = [zeros(3,6), eye(3), zeros(3)];
    
    covariances.R = diag([1E1*ones(3,1)]);
    
    Q = 1*diag([0*1E-16*ones(1,3), 1E-5*ones(1,3), 0*1E-15*ones(1,3), 1E-5*ones(1,3)]);
    
    L = blkdiag(eye(3), -eye(3), -q2R(qconj(plQb)), eye(3));
    
    [Disc.A,covariances.Qd] = lti_disc(F, L, Q, dt);
    
    priori = KF_timeupdate(posterior, 0, Disc, covariances);
    
    predE = q2e(plQb);
    d_lQb = qprod(qconj(lQb), tlQb);
    dE_Q = q2e(d_lQb);
    dE = dE_Q;
    
    measured.vl = init_Vl;
    
    dV = measured.vl - predicted.vl(k,:)';
    
    posterior = KF_measupdate(priori, Disc, [dV]);
    
    % we move misalignment information out of the filter to achieve better
    % linearization
    dlQl = qprod(e2q(posterior.x(1:3)),dlQl);
    posterior.x(1:3) = [0;0;0];
    
    %Apply velocity updates to the system also, and remove information from
    %the filter state
    predicted.vl(k,:) = predicted.vl(k,:) + posterior.x(7:9)';
    posterior.x(7:9) = [0;0;0];
    
    DX = [DX; posterior.dx'];
    
    X = [X; posterior.x'];
%     Rbias = [Rbias; qrot(e2q(posterior.x(1:3)),posterior.x(4:6))'];
    COV = [COV;diag(posterior.P)'];
    
    clear bRl
    
    %store data for later plotting
    DE = [DE;dE'];
    TE = [TE;predE'];
    
    DV = [DV; dV'];
    
    if (mod(k,1000)==0)
        disp(['t = ' num2str(k/1000) ' s'])
%         disp 'Predicted bRl'
%         q2R(qconj(lQb))
%         disp 'Estimated misalignment'
%         q2R(qconj(tlQb))*( eye(3) + vec2skew(posterior.x(1:3)) )
    end
end

%% Plotting

figure(1),clf
% subplot(411),plot(true.wb)
% title('True rotation rates w')
subplot(411),plot(measured.wb)
title('Measured rotation rates w')
subplot(412),plot(TE)
title('Predicted Euler angles')
% subplot(414),plot(true.ab)
title('True body measured accelerations')


figure(2),clf
subplot(611),plot(DE)
title('Measured Misalignment -- SOMETHING IS WRONG WITH THIS MEASUREMENT PROCESS, ignore for now')
grid on
subplot(612),plot(DX(:,1:3))
title('K * Innov updates to misalignment')
subplot(613),plot(X(:,1:3))
title('KF misalignment estimates')
grid on
subplot(614)
plot(DX(:,4:6))
title('K * Innov updates to gyro bias')
subplot(615)
plot(X(:,4:6))
title('Estimated gyro biases')
grid on

sf = 3;
index = 4;

covbounds = 0.5;%sf*max(abs([bias; bias2]));

subplot(6,3,16),plot(X(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-covbounds,covbounds])
grid on

index = 5;
subplot(6,3,17),plot(X(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('Bias estimation error')
axis([1,iter,-covbounds,covbounds])
grid on

index = 6;
subplot(6,3,18),plot(X(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-covbounds,covbounds])
grid on

% Plot velocity components separately

figure(3), clf

subplot(611)
plot(measured.ab)
title('Measured accelerations in body frame')

subplot(612)
plot(X(:,10:12))
title('Estimated accelerometer biases')

subplot(613)
plot(predicted.fl)
title('Predicted local frame specific force')

subplot(614)
plot(predicted.vl)
title('Preidcted local frame velocity')

subplot(615)
plot(DX(:,7:9))
title('Kalman updates to estimated local frame velocity errors -- DV')

subplot(616)
plot(predicted.vl + X(:,7:9))
title('Predicted local frame velocities')


% Tracing gravity feedback error
figure(4), clf

subplot(611),
plot(TE)
title('Predicted euler angles')

subplot(612)
plot(predicted.fl)
title('Predicted local frame specific force')

subplot(613)
plot(predicted.vl)
title('Predicted velocity in local frame')

subplot(614)
plot(DV)
title('Local frame velocity innovation -- DV = measured - estimated')

subplot(615)
plot(DX(:,7:9))
title('K * Innovation updates to dV')

subplot(616)
plot(X(:,7:9))
title('Estimated dVl')


figure(5),clf

subplot(411)
plot(DX(:,10:12))
title('K * innov updates to accelerometer bias estimates')

subplot(412)
plot(X(:,10:12))
title('Accelerometer bias estimates')



figure(6), clf

sf = 3;


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
axis([1,iter,-covbounds,covbounds])
grid on

index = 2;
subplot(ROWS,COLS,COLS*(r-1)+2),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('Updates to misalignment estimates in local frame -- accumulated outside filter')
axis([1,iter,-covbounds,covbounds])
grid on

index = 3;
subplot(ROWS,COLS,COLS*(r-1)+3),plot(DX(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-covbounds,covbounds])
grid on


r = r+1;
index = index+1;

covbounds = 0.02;%sf*max(abs([bias; bias2]));

subplot(ROWS,COLS,COLS*(r-1)+1),plot(X(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-covbounds,covbounds])
grid on

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+2),plot(X(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('Gyro bias estimates in body frame')
axis([1,iter,-covbounds,covbounds])
grid on

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+3),plot(X(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-covbounds,covbounds])
grid on


r = r+1;
index = index+1;

covbounds = 0.5;%sf*max(abs([bias; bias2]));

subplot(ROWS,COLS,COLS*(r-1)+1),plot(predicted.vl(:,1) + X(:,index)-init_Vl(1))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-covbounds,covbounds])
grid on
title('subtracting init\_Vl')

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+2),plot(predicted.vl(:,2) + X(:,index)-init_Vl(2))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('Local frame predicted + estimated velocity')
axis([1,iter,-covbounds,covbounds])
grid on

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+3),plot(predicted.vl(:,3) + X(:,index)-init_Vl(3))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-covbounds,covbounds])
grid on

% Accel biases

ACCBIASERR = repmat(accelbias',iter,1) - X(:,10:12);

r = r+1;
index = index+1;

covbounds = 0.1;%sf*max(abs([bias; bias2]));

subplot(ROWS,COLS,COLS*(r-1)+1),plot(ACCBIASERR(:,1))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-covbounds,covbounds])
grid on
xlabel('X')


index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+2),plot(ACCBIASERR(:,2))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('Estimated accelerometer bias error in body frame')
axis([1,iter,-covbounds,covbounds])
grid on
xlabel('Y')

index = index+1;
subplot(ROWS,COLS,COLS*(r-1)+3),plot(ACCBIASERR(:,3))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-covbounds,covbounds])
grid on
xlabel('Z')


disp 'DONE'


