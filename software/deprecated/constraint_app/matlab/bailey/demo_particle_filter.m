function demo_particle_filter(N)
%function demo_particle_filter(N)
% N is number of particles.
%
% This demo compares a basic particle filter with a Kalman filter on a linear problem.
% In this case, the KF estimate is optimal. The particle filter can, at best, approximate
% the KF result. Compare the results for small and large numbers of particles.
%
% The state is a point-target moving according to a constant-velocity model. The state
% parameters are [x; y; vx; vy]. The observation is the target's position [x; y], as given 
% by a GPS.
%
% Notice how the particle filter tends to perform badly if the process noise is small.
% This is greatly improved by employing the "optimal importance function" as the proposal
% distribution. See Doucet 98 for more details.
%
% Tim Bailey 2004.

dt = 0.2; % time-step (seconds)

intro;
h = setup_plots;

% Properties of target
xtrue = [0;0;1;0]; % true state: [x; y; vx; vy]
F = [1 0 dt 0; % state transition model
     0 1 0 dt;
     0 0 1  0;
     0 0 0  1];
Q = zeros(4) + diag([eps,eps,eps,eps]); % process noise
Q(3,3) = 0.2^2; % x velocity noise (st. dev. in m\s)
Q(4,4) = 0.5^2; % y velocity noise (st. dev. in m\s)

% Properties of GPS sensor
H = [1 0 0 0;   % observation model
     0 1 0 0];
R = [5^2 0;     % observe noise (st. dev. in m)
      0  5^2]; 

% Initialise filters (so particles and KF are the same)
Pest = [20 0 0 0; 
        0 20 0 0; 
        0  0 20 0;
        0 0 0 20];
xpart = multivariate_gauss(xtrue, Pest, N); % initial particles
[xest, Pest] = sample_mean(xpart);          % initial mean and covariance (for KF)

% main loop 
NLOOPS = 500;
kpath = zeros(2,NLOOPS); % storage for KF path estimate
ppath = zeros(2,NLOOPS); % storage for PF path estimate
for i=1:NLOOPS
    
    % simulate motion 
    xtrue = F * xtrue;
    
    % predict Kalman
    xest = F * xest;
    Pest = F*Pest*F' + Q;
    
    % predict particle
    s = multivariate_gauss([0;0;0;0], Q, N);
    xpart = F*xpart + s; 
    
    % simulate observation
    GPS = multivariate_gauss(xtrue(1:2), R, 1);
    
    % observe Kalman
    zpred = H * xest;
    v = GPS - zpred;
    [xest,Pest]= KF_update(xest,Pest, v,R,H);
    
    % observe particle
    w = gauss_likelihood(repvec(GPS,N) - H*xpart, R); 
    [keep, Neff] = stratified_resample(w);
    xpart = xpart(:,keep);
    
    % plotting
    [kpath, ppath] = do_plots(h, xtrue, GPS, xest, Pest, xpart, kpath, ppath, i);
end

%
%

function h = setup_plots()
figure, hold on, axis equal, axis([-10 110 -50 50]), grid
h.xt = plot(0,0, 's', 'erasemode', 'xor');     % true position
h.GPS = plot(0,0, 'k*', 'erasemode', 'xor');   % GPS observe
h.xk = plot(0,0, 'r+', 'erasemode', 'xor');    % Kalman estimate
h.Pk = plot(0,0, 'r', 'erasemode', 'xor');     % Kalman covariance
h.xpart = plot(0,0, 'g.', 'erasemode', 'xor'); % Particles
h.xp = plot(0,0, 'g+', 'erasemode', 'xor');    % Particle estimate
h.Pp = plot(0,0, 'g', 'erasemode', 'xor');     % Particle covariance
h.kpath = plot(0,0, 'r', 'erasemode', 'xor');  % Kalman estimate path
h.ppath = plot(0,0, 'g', 'erasemode', 'xor');  % Particle estimate path
xlabel('x - metres')
ylabel('y - metres')

%
%

function [kpath, ppath] = do_plots(h, xtrue, GPS, xest, Pest, xpart, kpath, ppath, i)
set(h.xt, 'xdata', xtrue(1), 'ydata', xtrue(2))
set(h.GPS, 'xdata', GPS(1), 'ydata', GPS(2))
set(h.xk, 'xdata', xest(1), 'ydata', xest(2))

ecov= sigma_ellipse(xest(1:2), Pest(1:2,1:2), 2);
set(h.Pk, 'xdata', ecov(1,:), 'ydata', ecov(2,:))

set(h.xpart, 'xdata', xpart(1,:), 'ydata', xpart(2,:))
[xp, Pp] = sample_mean(xpart);
set(h.xp, 'xdata', xp(1), 'ydata', xp(2))

pcov= sigma_ellipse(xp(1:2), Pp(1:2,1:2), 2);
set(h.Pp, 'xdata', pcov(1,:), 'ydata', pcov(2,:))

kpath(:,i) = xest(1:2);
ppath(:,i) = xp(1:2);
set(h.kpath, 'xdata', kpath(1,1:i), 'ydata', kpath(2,1:i))    
set(h.ppath, 'xdata', ppath(1,1:i), 'ydata', ppath(2,1:i))    

drawnow

%
%

function intro()
disp(' ')
disp('This demo compares a basic particle filter with a Kalman filter on a linear problem.')
disp('In this case, the KF estimate is optimal. The particle filter can, at best, approximate')
disp('the KF result. Compare the results for small and large numbers of particles.')
disp(' ')
disp('The state is a point-target moving according to a constant-velocity model. The state')
disp('parameters are [x; y; vx; vy]. The observation is the target position [x; y], as given')
disp('by a GPS.')
disp(' ')
disp('The true position is shown by a square, the GPS measurements by a star, the KF estimate')
disp('in red and the particle estimate in green.')
disp(' ')
