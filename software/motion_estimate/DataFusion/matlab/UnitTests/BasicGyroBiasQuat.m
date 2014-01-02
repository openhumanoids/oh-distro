
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

figure(1),clf

iter = 20000;
dt = 0.001;

gn = [0;0;1]; % forward left up

initE = [0;0;0];

bias = [0.;0.01;0];
bias2 = [-0.005;0;0];


% something with bias.

true.wb = zeros(iter, 3);

true.wb(3500:3600,2) = +pi/6/100*1000;



% tbQl = e2q(initE);
tlQb = [1;0;0;0];
% tbRl = q2R(tbQl);

% E = cumsum(true.w.*dt);
E = [];
GB = [];
for k = 1:iter
    %     tbRl = closed_form_DCM_farrell(-dt*true.w(k,:)', tbRl); % True rotation matrix
    tlQb = zeroth_int_Quat_closed_form(-true.wb(k,:)', tlQb, dt); % true orientation quaternion
%     E = [E; q2e(R2q(tbRl))'];
    E = [E; q2e(tlQb)'];
%     GB = [GB;(tbRl'*gn)'];
    gb = qrot(tlQb,gn);
    GB = [GB; gb'];
end


measured.wb = true.wb;
% add the biases to the measured values
for k = 500:(5000)
    measured.wb(k,:) = measured.wb(k,:) + bias';
end
for k = (4000):iter
    measured.wb(k,:) = measured.wb(k,:) + bias2';
end
measured.wb = measured.wb + 0.001*randn(size(measured.wb));

subplot(411),plot(true.wb)
title('True rotation rates w')
subplot(412),plot(measured.wb)
title('Measured rotation rates w')
subplot(413),plot(E)
title('True Euler angles')
subplot(414),plot(GB)
title('True body measured accelerations')

%%

% estimate the misalignment and gyro bias

posterior.x = zeros(6,1);
posterior.P = blkdiag(1*eye(3),1*eye(3));


Disc.B = 0;

X = [];
COV = [];

% tbQl = e2q(initE);
tlQb = [1;0;0;0];
% tbRl = q2R(tbQl);
% bQl = e2q(initE);
lQb = [1;0;0;0];
% bRl = q2R(bQl);

DE = [];
TE = [];

for k = 1:iter
    %tbRl = closed_form_DCM_farrell(-dt*true.w(k,:)', tbRl); % True rotation matrix
    %bRl = q2R(bQl);
    %bRl = closed_form_DCM_farrell(-dt*measured.w(k,:)', bRl); % use negative rotations because here we are looking at the body to world rotations
    %bQl = R2q(bRl);
    
    tlQb = zeroth_int_Quat_closed_form(-true.wb(k,:)', tlQb, dt);
    lQb = zeroth_int_Quat_closed_form(-measured.wb(k,:)', lQb, dt);
    
    F = zeros(6);
%     F(1:3,4:6) = bRl';
    F(1:3,4:6) = q2R(qconj(lQb));
    
    Disc.A = eye(6) + F.*dt; % Basic first order approximate, but will be converted expm
    % use expm with Pade approximation
    Disc.C = [eye(3), zeros(3)];
    
    covariances.R = 1E-3*eye(3);
    
    Q = diag([1E-10*ones(1,3), 1E-5*ones(1,3)]);
    
    %     L = [bRl', zeros(3); zeros(3), eye(3)]; % this line may be wrong
    
    L = [q2R(qconj(lQb)), zeros(3); zeros(3), eye(3)];
    covariances.Qd = L*Q*L'*dt;
    
    priori = KF_timeupdate(posterior, 0, Disc, covariances);
    
    %     testE = E(k,:)' - dE_R
    %     dE = zeros(3,1) - E(k,:)';

    predE = q2e(lQb);
    %     d_bQl = R2q(q2R(bQl)' * tbRl);
    %     dE_Q = q2e(d_bQl);
        

    d_lQb = qprod(qconj(lQb), tlQb);
    dE_Q = q2e(d_lQb);
    
    dE = dE_Q;
    
    DE = [DE;dE'];
    TE = [TE;predE'];
    
    posterior = KF_measupdate(priori, Disc, [dE]);
    
    X = [X; posterior.x'];
    COV = [COV;diag(posterior.P)'];
    
    clear bRl
end

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
subplot(4,3,10),plot([bias(1)*ones(iter,1)] + X(:,index))
hold on
semilogx(sqrt(COV(:,index)) ,'r')
semilogx(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-sf*max(bias),sf*max(bias)])
grid on

index = 5;
subplot(4,3,11),plot([bias(2)*ones(iter,1)] + X(:,index))
hold on
semilogx(sqrt(COV(:,index)) ,'r')
semilogx(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-sf*max(bias),sf*max(bias)])
grid on

index = 6;
subplot(4,3,12),plot([bias(3)*ones(iter,1)] + X(:,index))
hold on
semilogx(sqrt(COV(:,index)) ,'r')
semilogx(-sqrt(COV(:,index)) ,'r')
axis([1,iter,-sf*max(bias),sf*max(bias)])
grid on


disp 'DONE'


