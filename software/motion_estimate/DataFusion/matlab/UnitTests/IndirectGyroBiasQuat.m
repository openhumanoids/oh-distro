
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
clear

disp 'STARTING...'


iter = 10000;
dt = 0.001;

gn = [0;0;1]; % forward left up

initE = [0;0;0];
bias = [0.;0.01;0];
bias2 = 0*[-0.005;0;0];

% Translational components
true.vl = zeros(iter,3);
true.fb = zeros(iter,3); % there are no forces on the body, so body frame forces are zero for the time being
measured.fb = true.fb + 1E-2*randn(iter,3);
true.ab = zeros(iter,3);
% true and measured velocities

true.wb = zeros(iter, 3);
true.wb(3500:3600,2) = +pi/6/100*1000;

% Add more random motion tot he true signal
if (1==0)
    WRT = [];
    repsteps = 100;
    for k = 1:3
        WRa = randn(iter/repsteps,1);
        WRa = repmat(WRa,1,repsteps);
        WRa = reshape(WRa',[],1);
        WRT = [WRT, WRa];
    end
    true.wb = true.wb + WRT;
end


% tbRl = eye(3);
% tbQl = e2q(initE);
% tbRl = q2R(tbQl);
tlQb = [1;0;0;0];

E = [];
GB = [];
for k = 1:iter
    %     tbRl = closed_form_DCM_farrell(-dt*true.w(k,:)', tbRl); % True rotation matrix
    tlQb = zeroth_int_Quat_closed_form(-true.wb(k,:)', tlQb, dt); % true orientation quaternion

    % E = [E; q2e(R2q(tbRl))'];
    E = [E; q2e(tlQb)'];
    
    %     true.ab(k,:) = (tbRl'*gn)' + true.fb(k,:); % we are in the forward left up frame
    true.ab(k,:) = qrot(tlQb,gn)' + true.fb(k,:); % we are in the forward left up frame
    
    %     GB = [GB;true.ab(k,:)];
    gb = qrot(tlQb,gn);
    GB = [GB; gb'];
    
    if (k > 1)
        %true.vl(k,:) = true.vl(k-1,:) + 0.5*dt*(true.ab(k-1,:) + true.ab(k,:)) - dt*(tbRl'*gn)';
        % true.vl(k,:) = true.vl(k-1,:) + 0.5*dt*(true.ab(k-1,:) + true.ab(k,:)) - dt*qrot(tlQb,gn)';
        %or directly from fl
        true.vl(k,:) = true.vl(k-1,:) + qrot(  qconj(tlQb), 0.5*dt*(true.fb(k-1,:)' + true.fb(k,:)')  )';
    end
end

% Measured body frame accelerations, with measurement noise
measured.ab = true.ab + 1E-3*randn(iter,3);
% Measured local frame velocities, with added measurement noise
measured.vl = true.vl + 3E-4*randn(iter,3);


measured.wb = true.wb;
% add the biases to the measured values
true.bias.wb = zeros(iter,3);
for k = 500:(iter-5000)
    true.bias.wb(k,:) = true.bias.wb(k,:) + bias';
end
for k = (iter-6000):iter
    true.bias.wb(k,:) = true.bias.wb(k,:) + bias2';
end

% Add the biases
measured.wb = measured.wb + true.bias.wb;
% Add measurement noise
measured.wb = measured.wb + 0.001*randn(size(measured.wb));

predicted.al = zeros(iter,3);
predicted.fl = zeros(iter,3);
predicted.vl = zeros(iter,3);

% estimate the misalignment and gyro bias

posterior.x = zeros(9,1);
posterior.P = blkdiag(1*eye(3),1*eye(3),1*eye(3));


Disc.B = 0;

X = [];
COV = [];

% tbQl = e2q(initE);
% tbRl = q2R(tbQl);
% bQl = e2q(initE);
% bRl = q2R(bQl);

tlQb = [1;0;0;0];
lQb = [1;0;0;0];

DE = [];
TE = [];
DV = [];
nDEF = [];

for k = 1:iter
    
    %     tbRl = closed_form_DCM_farrell(-dt*true.w(k,:)', tbRl); % True rotation matrix
    %     % Orientation estimate
    %     bRl = q2R(bQl);
    %     bRl = closed_form_DCM_farrell(-dt*measured.w(k,:)', bRl); % use negative rotations because here we are looking at the body to world rotations
    %     bQl = R2q(bRl);
    
    tlQb = zeroth_int_Quat_closed_form(-true.wb(k,:)', tlQb, dt);
    lQb = zeroth_int_Quat_closed_form(-measured.wb(k,:)', lQb, dt);

    predicted.al(k,:) = qrot(qconj(lQb),measured.ab(k,:)')';
    
    predicted.fl(k,:) = (qrot(qconj(lQb),measured.ab(k,:)') - gn)';
    if (k > 1)
        predicted.vl(k,:) = predicted.vl(k-1,:) + 0.5*dt*(predicted.fl(k-1,:) + predicted.fl(k,:));
    end
    
    F = zeros(9);
    F(1:3,4:6) = q2R(qconj(lQb));
%     F(7:9,1:3) = vec2skew(-predicted.fl(k,:)');
    
    %Disc.A = eye(6) + F.*dt; % Basic first order approximate
    Disc.A = expm(F.*dt); % w Pade approximations
    Disc.C = [eye(3), zeros(3,6); zeros(3,6), eye(3)];
    %     Disc.C = [eye(3), zeros(3)];
    
    covariances.R = blkdiag(1E-3*eye(3),1E-3*eye(3));
    
    Q = diag([1E-10*ones(1,3), 1E-5*ones(1,3), 1E-5*ones(1,3)]);
    
    L = blkdiag(q2R(qconj(lQb)), eye(3), zeros(3));
    covariances.Qd = L*Q*L'*dt;
    
    nDEF = [nDEF; (vec2skew(-predicted.fl(k,:)')*posterior.x(1:3))' ];
    
    priori = KF_timeupdate(posterior, 0, Disc, covariances);
    
    %     testE = E(k,:)' - dE_R
    %     dE = zeros(3,1) - E(k,:)';
    
    %     predE = q2e(R2q(bRl));
    %     d_bQl = R2q(q2R(bQl)' * tbRl);
    %     dE_Q = q2e(d_bQl);
    
    predE = q2e(lQb);
    d_lQb = qprod(qconj(lQb), tlQb);
    dE_Q = q2e(d_lQb);
    dE = dE_Q;
    
    % predictedV = [0;0;0];
    
    dV = measured.vl(k,:)' - predicted.vl(k,:)';
    % dV = 1E-3*randn(3,1);
    
    posterior = KF_measupdate(priori, Disc, [dE; dV]);
    
    X = [X; posterior.x'];
    COV = [COV;diag(posterior.P)'];
    
    clear bRl
    
    %store data for later plotting
    DE = [DE;dE'];
    TE = [TE;predE'];
    
    DV = [DV; dV'];
    
    if (mod(k,1000)==0)
        disp(['t = ' num2str(k/1000) ' s'])
    end
end

%% Plotting

figure(1),clf
subplot(411),plot(true.wb)
title('True rotation rates w')
subplot(412),plot(measured.wb)
title('Measured rotation rates w')
subplot(413),plot(E)
title('True Euler angles')
subplot(414),plot(true.ab)
title('True body measured accelerations')


figure(2),clf
subplot(411),plot(DE)
title('Measured Misalignment')
grid on
subplot(412),plot(DE - X(:,1:3))
title('KF misalignment estimate errors')
grid on
subplot(413),plot(X(:,4:6))
title('Estimated gyro biases')
grid on

sf = 2;
index = 4;
subplot(4,3,10),plot(true.bias.wb(:,1) - X(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-sf*max(bias),sf*max(bias)])
grid on

index = 5;
subplot(4,3,11),plot(true.bias.wb(:,2) - X(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
title('Bias estimation error')
axis([1,iter,-sf*max(bias),sf*max(bias)])
grid on

index = 6;
subplot(4,3,12),plot(true.bias.wb(:,3) - X(:,index))
hold on
plot(sqrt(COV(:,index)) ,'r')
plot(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-sf*max(bias),sf*max(bias)])
grid on

% Plot velocity components separately

figure(3), clf

subplot(411)
plot(true.ab)
title('True body accelerations')

subplot(412)
plot(measured.vl)
title('Measured local velocities')

subplot(413)
plot(predicted.fl)
title('Predicted local frame specific force')

subplot(414)
plot(DV)
title('Local frame velocity innovation -- DV')


% Tracing gravity feedback error
figure(4), clf

subplot(411),
plot(predicted.al)
title('Predicted local frame acceleration')

subplot(412)
plot(predicted.fl)
title('Predicted local frame specific force')

subplot(413)
plot(cumsum(nDEF))
title('cumsum(nDEF)')

disp 'DONE'


