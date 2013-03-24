function demo_bearing_only()
%function demo_bearing_only
%
% Demonstrate the behaviour of an EKF, UKF, and naive particle filter for
% the bearing-only target-tracking problem as described in: 
%
%   N. Gordon, D. Salmond, and A. F. M. Smith, Novel approach to nonlinear
%   and non-Gaussian Bayesian state estimation, Proc. Inst. Elect. Eng., F,
%   vol. 140, pp. 107-113, 1993.
%
% The plots show variances of EKF (red), UKF(cyan) and particle(blue), as well
% as the prior particle samples (green) and resampled posterior particle samples (blue).
% The true states and observations are shown in black.
%
% Tim Bailey 2005.

intro;
h = setup_plot;

% true states and observations
xtrue = [-0.05; 0.001; 0.7; -0.055];
[xactual, zactual] = generate_truestates(xtrue);

% linear models and noises
[F,G,Q] = predict_model;
R = observe_model_noise;

% initial filter estimates with Gordon's prior (EKF - {x,P}, UKF - {xu,Pu}, particles {s})
P = diag([0.5^2, 0.005^2, 0.3^2, 0.01^2]); 
x = [0.0; 0.0; 0.4; -0.05];                
Pu = P;
xu = x;
N = 20000;
s = gauss_samples(x,P,N);

% filter loop
for i = 1:length(zactual)
    
    % predict step (Kalman)
    x = F*x;
    P = F*P*F' + G*Q*G';
    xu = F*xu;
    Pu = F*Pu*F' + G*Q*G';
    
    % predict step (particle)
    s = F*s + G*gauss_samples([0;0],Q,N);
    spri = s; % prior sample set (ie, predict set)
    
    % update step (EKF and UKF)
    z = zactual(i);
    [x,P]   = EKF_update      (@observe_model, @observe_model_diff, x, P,  z,R);
    [xu,Pu] = unscented_update(@observe_model, @observe_model_diff, xu,Pu, z,R);
    
    % update step (particle)
    v = observe_model_diff(repmat(z,1,N), observe_model(s));
    w = gauss_likelihood(v, R);    
    if sum(w) ~= 0
        [keep, Neff] = stratified_resample(w);
        s = s(:,keep);
    else
        warning('All likelihoods are zero at step:'), disp(i)
    end
    
    % plots
    do_plot(h, xactual(:,1:i+1), z, spri,s, x,P, xu,Pu);
    disp(['Step ' num2str(i) '. Press any key to continue.'])
    pause
end

%
%

function [xactual, zactual] = generate_truestates(xtrue)
xactual = xtrue;
[F,G,Q] = predict_model;
R = observe_model_noise;

for i=1:24
    xactual(:,i+1) = F * xactual(:,i) + G * gauss_samples([0;0],Q,1);
    zactual(i) = observe_model(xactual(:,i+1)) + gauss_samples(0,R,1);
end

%
%

function [F,G,Q] = predict_model()
F = [1 1 0 0; % state transition model
     0 1 0 0;
     0 0 1 1;
     0 0 0 1];

G = [0.5 0; % model relating process noise to state
     1  0;
     0 0.5;
     0  1];
 
Q = eye(2) * 0.001^2; % process noise

%
%

function z = observe_model(x)
z = atan2(x(3,:), x(1,:));

%
%

function v = observe_model_diff(z1,z2)
v = pi_to_pi(z1 - z2);

%
%

function R = observe_model_noise()
R = 0.005^2; 

%
%

function h = setup_plot()

figure, hold on, % axis equal

h.prior = plot(0,0,'g+', 'markersize', 2); % prior samples
h.post = plot(0,0,'bx', 'markersize', 3);  % posterior samples
h.cov = plot(0,0,'b');                     % particle variance

h.kfx = plot(0,0,'r.');                    % kalman mean
h.kfP = plot(0,0,'r');                     % kalman covariance

h.ux = plot(0,0,'c.');                     % unscented mean
h.uP = plot(0,0,'c');                      % unscented covariance

h.xt = plot(0,0,'k*');                     % actual state
h.z = plot(0,0,'k');                       % actual measurement

%
%

function do_plot(h, xtrue, z, spri, spost, x,P, xu,Pu)

set(h.prior, 'xdata', spri(1,:), 'ydata', spri(3,:))
set(h.post, 'xdata', spost(1,:), 'ydata', spost(3,:))
set(h.z, 'xdata', [0 cos(z)], 'ydata', [0 sin(z)])
set(h.xt, 'xdata', xtrue(1,:), 'ydata', xtrue(3,:))

[xm,Pm] = sample_mean(spost([1 3],:));
e = sigma_ellipse(xm, Pm, 2);
set(h.cov, 'xdata', e(1,:), 'ydata', e(2,:))

e = sigma_ellipse(x([1 3]), P([1 3],[1 3]), 2);
set(h.kfP, 'xdata', e(1,:), 'ydata', e(2,:))
set(h.kfx, 'xdata', x(1), 'ydata', x(3))

e = sigma_ellipse(xu([1 3]), Pu([1 3],[1 3]), 2);
set(h.uP, 'xdata', e(1,:), 'ydata', e(2,:))
set(h.ux, 'xdata', xu(1), 'ydata', xu(3))

drawnow

%
%

function intro()

disp(' ')
disp('This demo shows the behaviour of an EKF, UKF, and basic particle filter for')
disp('the bearing-only target-tracking problem as described in: ')
disp(' ')
disp('  N. Gordon, D. Salmond, and A. F. M. Smith, Novel approach to nonlinear')
disp('  and non-Gaussian Bayesian state estimation, Proc. Inst. Elect. Eng., F,')
disp('  vol. 140, pp. 107-113, 1993.')
disp(' ')
disp('The plots show variances of EKF (red), UKF(cyan) and particle(blue), as well')
disp('as the prior particle samples (green) and resampled posterior particle samples')
disp('(blue). The measurements and trajectory of true states are shown in black.')
disp(' ')