
% the biases are changed twice during the iteration cycle
% initial orientation is rolled positive 45 degrees, then bumped in body
% pitch, down by 30 degrees.
% the filter estimates the biases of the gyro successfully.
% The filter estimates do diverge after long timescales, and at present is
% thought to be due to linearization errors regarding the orientation --
% it gets worse for larger biases. Next step is to make a copy of this
% script and implement the velocity based pitch roll estimate, and then a
% feedback structure. We may also upgrade to passing orientations by
% quaternions rather than the rotation matrices

clc
clear

dt = 0.01;

disp 'Loading data from file...'

data = load('UnitTests/testdata/GH_imudata01.txt');

keep = 2500;

measured.wb = data(1:keep,1:3);
measured.ab = data(1:keep,4:6);

% Try estimate the error parameters of the sensors
initstart = 1;
initend = 500;
biasg = mean(measured.wb(initstart:initend,:));
biasg = repmat(biasg,keep,1);

measured.wb = measured.wb - biasg;

%% Own data sequence

measured.wb = zeros(keep,3);
measured.wb(201:300,2) = measured.wb(201:300,2) - pi/1/100/dt;
measured.wb(501:600,1) = measured.wb(501:600,1) + pi/2/100/dt;
measured.wb(801:900,2) = measured.wb(801:900,2) - pi/2/100/dt;
measured.wb(1101:1200,1) = measured.wb(1101:1200,1) - pi/2/100/dt;
measured.wb(1401:1500,3) = measured.wb(1401:1500,3) + pi/2/100/dt;
measured.wb(1701:1800,1) = measured.wb(1701:1800,1) - pi/2/100/dt;
measured.wb(2001:2100,2) = measured.wb(2001:2100,2) - pi/2/100/dt;
measured.wb(2301:2400,3) = measured.wb(2301:2400,3) + pi/1/100/dt;

%%
disp 'STARTING...'

iter = keep;

gn = [0;0;9.81]; % forward left up

% initE = [0;0;0];
% bRl = q2R(e2q(initE));

% E = [];
% for k = 1:iter
%     tbRl = closed_form_DCM_farrell(-dt*true.w(k,:)', tbRl); % True rotation matrix
%     E = [E; q2e(R2q(tbRl))'];
%     GB = [GB;(tbRl'*gn)'];
% end


% estimate the misalignment and gyro bias

posterior.x = zeros(6,1);
posterior.P = blkdiag(1*eye(3),1*eye(3));


Disc.B = 0;

X = [];
COV = [];

% bRl = q2R(e2q(initE));
lQb = qprod([1;0;0;0],[0;1;0;0]); % Microstrain data is known to start with Z pointing down

DE = [];
TE = [];
CE = [];
GB = [];

for k = 1:iter
    %     bRl = closed_form_DCM_farrell(-dt*measured.wb(k,:)', bRl); % use negative rotations because here we are looking at the body to world rotations
    lQb = zeroth_int_Quat_closed_form(-measured.wb(k,:)', lQb, dt);
    
    % predict the measured gravity vector
    gb = qrot(lQb,gn);
    GB = [GB; gb'];
    
    %     bRl = q2R(e2q(E(k,:)'));

%     F = zeros(6);
%     F(1:3,4:6) = bRl';
%     
%     Disc.A = eye(6) + F.*dt; % Basic first order approximate
%     Disc.C = [eye(3), zeros(3)];
%     
%     covariances.R = 1E-3*eye(3);
%     
%     Q = diag([1E-10*ones(1,3), 1E-5*ones(1,3)]);
%     
%     L = [bRl', zeros(3); zeros(3), eye(3)];
%     covariances.Qd = L*Q*L'*dt;
%     
%     priori = KF_timeupdate(posterior, 0, Disc, covariances);
    
%     predE = q2e(R2q(bRl));
    predE = q2e(lQb);
    CE = [CE; predE'];

%     d_bRl = bRl' * tbRl;
%     dE_R = q2e(R2q(d_bRl));
%     
%     dE = dE_R;
%     
%     DE = [DE;dE'];
%     TE = [TE;predE'];
%     
%     posterior = KF_measupdate(priori, Disc, [dE]);
%     
%     X = [X; posterior.x'];
%     COV = [COV;diag(posterior.P)'];
end



%% Plotting

figure(1),clf
subplot(411),plot(measured.wb)
title('Measured rotation rates w')
subplot(412),plot(measured.ab)
title('Body measured accelerations')
subplot(413),plot(CE)
title('Computed Euler angles')
subplot(414),plot(GB)
title('predicted body measured gravity')



% figure(2),clf
% 
% subplot(411),plot(DE)
% title('Measured Misalignment')
% grid on
% 
% subplot(412),plot(DE - X(:,1:3))
% title('KF misalignment estimate errors')
% grid on
% 
% subplot(413),plot(X(:,4:6))
% title('Estimated gyro biases')
% grid on
% 
% sf = 2;
% index = 4;
% subplot(4,3,10),plot(0*[bias(1)*ones(iter,1)] + X(:,index))
% hold on
% semilogx(sqrt(COV(:,index)) ,'r')
% semilogx(-sqrt(COV(:,index)) ,'r')
% axis([1,iter,-sf*max(bias),sf*max(bias)])
% grid on
% 
% index = 5;
% subplot(4,3,11),plot(0*[bias(2)*ones(iter,1)] + X(:,index))
% hold on
% semilogx(sqrt(COV(:,index)) ,'r')
% semilogx(-sqrt(COV(:,index)) ,'r')
% axis([1,iter,-sf*max(bias),sf*max(bias)])
% grid on
% 
% index = 6;
% subplot(4,3,12),plot(0*[bias(3)*ones(iter,1)] + X(:,index))
% hold on
% semilogx(sqrt(COV(:,index)) ,'r')
% semilogx(-sqrt(COV(:,index)) ,'r')
% axis([1,iter,-sf*max(bias),sf*max(bias)])
% grid on


disp 'DONE'


