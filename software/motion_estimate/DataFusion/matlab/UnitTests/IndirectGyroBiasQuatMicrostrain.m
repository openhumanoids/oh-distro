
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

data = load('UnitTests/testdata/dfd_loggedIMU_03.txt');
iter = 6000;

initstart = 1;
initend = 1000;

gn = [0;0;9.81]; % forward left up

init_lQb = [1;0;0;0];
% tlQb = init_lQb;

% Bias errors to introduce later
bias = 0*[0;0.002;0];


E = [];
GB = [];

measured.wb = data(1:iter,1:3);
measured.ab = data(1:iter,4:6);

% remove gyro biases

biasg = mean(measured.wb(initstart:initend,:),1) + [0.01,0,0];
biasg = repmat(biasg,iter,1);
measured.wb = measured.wb - biasg;

predicted.al = zeros(iter,3);
predicted.fl = zeros(iter,3);
predicted.vl = zeros(iter,3);


posterior.x = zeros(9,1);
posterior.P = blkdiag(100*eye(3),500*eye(3),100*eye(3));

Disc.B = 0;

X = [];
DX = [];
COV = [];

tlQb = init_lQb;
lQb = init_lQb;

DE = [];
TE = [];
DV = [];
nDEF = [];
Rbias = [];

for k = 1:iter
    
    lQb = zeroth_int_Quat_closed_form(-measured.wb(k,:)', lQb, dt);

    plQb = lQb;

    % Accelerometer bias compensation
    predicted.ab(k,:) = measured.ab(k,:);
    
    % predict local frame accelerations
    predicted.al(k,:) = qrot(qconj(plQb),predicted.ab(k,:)')';
    
    predicted.fl(k,:) = (predicted.al(k,:)' - gn)';% For velocity and position
    if (k > 1)
        predicted.vl(k,:) = predicted.vl(k-1,:) + 0.5*dt*(predicted.fl(k-1,:) + predicted.fl(k,:));
    end
    
    F = zeros(9);
    F(1:3,4:6) = -eye(3);
    F(7:9,1:3) = -q2R(qconj(plQb))*vec2skew(predicted.ab(k,:)');
    
    Disc.C = [zeros(3,6), eye(3)];
    
    covariances.R = diag([1E-2*ones(3,1)]);
    
    Q = diag([1E-9*ones(1,3), 1E-4*ones(1,3), 1E-9*ones(1,3)]);
    
    L = blkdiag(eye(3), -eye(3), eye(3));
    
    [Disc.A,covariances.Qd] = lti_disc(F, L, Q, dt);
    
    priori = KF_timeupdate(posterior, 0, Disc, covariances);
    
    predE = q2e(plQb);
    d_lQb = qprod(qconj(lQb), tlQb);
    dE_Q = q2e(d_lQb);
    dE = dE_Q;
    
    measured.vl = [0;0;0];
    
    dV = measured.vl - predicted.vl(k,:)';
    
    posterior = KF_measupdate(priori, Disc, [dV]);
    
    
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
title('True rotation rates w')
subplot(412),plot(measured.wb)
title('Measured rotation rates w')
subplot(413),plot(E)
title('True Euler angles')
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

subplot(411)
% plot(true.ab)
title('True body accelerations')

subplot(412)
plot(measured.vl)
title('Measured local velocities')

subplot(413)
plot(predicted.fl)
title('Predicted local frame specific force')

subplot(414)
% plot(DV)
% title('Local frame velocity innovation -- DV')


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



% subplot(414)
% plot(cumsum(nDEF))
% title('cumsum(nDEF)')

disp 'DONE'


