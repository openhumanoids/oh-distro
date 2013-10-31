function traj = gen_traj(iterations, param, showplots)
% How many iteration samples should be generated
% Hd is a subduing filter used to enhance quality of the generated data

if (nargin<3)
    showplots=0;
end

d = fdesign.lowpass('Fp,Fst,Ap,Ast',3,5,0.5,40,100);
Hd = design(d,'equiripple');

dt = param.dt;


%% Prepare true trajectory

knobs_hori.alpha = 1;
knobs_hori.beta = 0.8;
knobs_hori.eta = 1E-2;
knobs_hori.step = 1500;

V_l(:,1) = basic_traj(iterations+1, knobs_hori, Hd);
V_l(:,2) = basic_traj(iterations+1, knobs_hori, Hd);

knobs_vert = knobs_hori;
knobs_hori.beta = 0.2;
knobs_vert.eta = 1E-3;

V_l(:,3) = basic_traj(iterations+1, knobs_vert, Hd) + filter(Hd,0.015*randn(size(V_l,1),1));


P_l = cumsum(V_l*dt);

 % generate true orientation from random data
knobs_ori.alpha = 1;
knobs_ori.beta = 1;
knobs_ori.eta = 1E-3;
knobs_ori.step = 200;

w_l(:,1) = basic_traj(iterations+1, knobs_ori, Hd);
w_l(:,2) = basic_traj(iterations+1, knobs_ori, Hd);

knobs_ori.alpha = 1;
knobs_ori.beta = 0.1;
knobs_ori.eta = 5E-3;
knobs_ori.step = 1000;

w_l(:,3) = basic_traj(iterations+1, knobs_ori, Hd);



%% Compute velocities, accelerations, rotation rates and orientation quaternion

% Local frame accelerations
f_l = diff(V_l)./dt;

% here we add gravity
a_l = f_l + [zeros(size(f_l,1),2) param.gravity*ones(size(f_l,1),1)];

% local rotation angles
R = eye(3);
w_l_ = zeros(1,3);
for n = 1:size(w_l,1)
    R = closed_form_DCM_farrell( 0.5*(w_l_ + w_l(n,:))'*dt , R);
    w_l_ = w_l(n,:);
    E_interm(n,:) = q2e(R2q(R))';
end

E = mod(E_interm + pi , 2*pi) - pi;

% Body frame accels and rates
w_b = zeros(size(w_l));
f_b = zeros(size(f_l));
a_b = zeros(size(a_l));
q = zeros(size(E,1),4);

for n = 1:length(a_l)
    q(n,:) = e2q(E(n,:)')';
    lRb = q2R(q(n,:)' );
    w_b(n,:) = (  w_l(n,:) );
    f_b(n,:) = ( lRb * f_l(n,:)' )';
    a_b(n,:) = ( lRb * a_l(n,:)' )';
end

t = (1:iterations).*dt;

% trim vectors to orginal size
P_l((iterations+1):end,:) = [];
V_l((iterations+1):end,:) = [];
w_l((iterations+1):end,:) = [];
w_b((iterations+1):end,:) = [];
E((iterations+1):end,:) = [];



traj.iterations = iterations;
traj.utime = 1E6*t;
traj.dt = dt;
traj.parameters.gravity = param.gravity;
traj.true.P_l = P_l;
traj.true.V_l = V_l;
traj.true.f_l = f_l;
traj.true.f_b = f_b;
traj.true.a_l = a_l;
traj.true.a_b = a_b;
traj.true.w_l = w_l;
traj.true.w_b = w_b;
traj.true.E = E;
traj.true.q = q;



% we can suppress the plotting process
if (showplots==0)
    return
end
%% Plot computations

figure(1),clf
plot3(P_l(:,1),P_l(:,2),P_l(:,3));
grid on
axis equal
dist = iterations/1000;
axis([-dist dist -dist dist -3 3])

figure(2), clf

% Plot the positions
subplot(431)
plot(t,P_l)
grid on
title('Local postions')

subplot(434)
plot(t,V_l)
grid on
title('Local velocities')

subplot(437)
plot(t,a_l)
grid on
title(['Local accels, stdevs ' num2str(std(a_l)) ' m/s^2 @ 1kHz'])

subplot(4,3,10)
plot(t,f_b)
grid on
title('Body accelerations')


subplot(4,3,11)
plot(t,a_b)
grid on
title('Body accelerations + g')

% plot the orientations
subplot(432)
plot(E)
grid on
title('Local to body orientation')


subplot(435)
plot(w_l)
grid on
title('Local frame rates')

subplot(438)
plot(w_b)
grid on
title('Body frame rates')


%% test the generated trajectory with our standard NII INS Mechanization

% initialize at identity
pose = init_pose();

insE = zeros(iterations,3);
insV = zeros(iterations,3);
insP = zeros(iterations,3);

imudata.gravity = [0;0;9.81];

% iterate through all the the data
for n = 1:iterations
    imudata.utime = traj.utime(n);
    imudata.ddp = traj.true.a_b(n,:)';
    imudata.da = -traj.true.w_b(n,:)';
    imudata.q = traj.true.q(n,:)';
    
    
    insE(n,:) = q2e(R2q(pose.R))';
    insV(n,:) = pose.V_l';
    insP(n,:) = pose.P_l';
    
    [pose] = INS_Mechanisation(pose, imudata);
end

subplot(433)
plot(insE)
grid on

subplot(436)
plot(traj.true.E - insE)
grid on

subplot(439)
plot(traj.true.V_l - insV)
grid on

subplot(4,3,12)
plot(traj.true.P_l - insP)
grid on


