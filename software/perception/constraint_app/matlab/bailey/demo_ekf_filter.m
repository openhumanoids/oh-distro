function demo_ekf_filter()
%function demo_ekf_filter()
% Simple demo of the various Kalman filter update equations and the numerical
% Jacobian function. The numerical_Jacobian function computes the Jacobian directly
% from the non-linear function, so there is no need to derive an analytical Jacobian.
%
% The state is a vehicle pose [x;y;phi]. The predict model is a simple bicycle
% model with controls of velocity and steer-angle. The observe model is range-
% bearing from a fixed observer at the origin.
%
% Notice how this demo deals with discontinuous models (ie, in this case the jump 
% between plus-minus pi in the observation model).
%
% Tim Bailey 2004.

% Setup plots
h = setup_plots;

% Initialise state
x = [0; 0; pi];
P = eye(3)*eps; % note: for stability, P should never be quite zero
xtrue = x;

% Specify controls
V = 1;          % velocity (m/s)
G = 0*pi/180;   % steer angle (rad)
dt = 0.1;       % time increment (s)

% Specify uncertainties
SigmaV = 0.5;       % velocity (m/s)
SigmaG = 5*pi/180;  % steer angle (rad)
Q = [SigmaV^2 0;    % prediction uncertainty
    0 SigmaG^2];    

SigmaR = 0.5;       % range (m)
SigmaB = 2*pi/180;  % bearing (rad)
R = [SigmaR^2 0;    % observation uncertainty
    0 SigmaB^2];

% Filter loop
NN = 1000;
xpath = zeros(2, NN);
for i=1:NN
    
    % true states
    xtrue = predict_model(xtrue, [V;G], dt);
    ztrue = observe_model(xtrue);
    
    % simulate our measurements by adding noise to true values
    u = multivariate_gauss([V;G], Q, 1);
    z = multivariate_gauss(ztrue, R, 1);
    
    % predict
    [x,P] = predict(x, P, u, Q, dt);
    
    % update
    [x,P] = update(x, P, z, R, 3); % the end parameter specifies which KF function to use.
    
    % plots
    xpath(:,i) = x(1:2);
    do_plots(h, x, P, xtrue, z, xpath(:,1:i));
end

%
%

function [x,P] = predict(x, P, u, Q, dt)
% We use the numerical_Jacobian function to compute an approximate Jacobian of the non-linear
% predict model. Thus, we can avoid deriving explicit analytical Jacobians. Note, to compute
% the Jacobian of a model h() wrt x, the parameter 'x' must be first in h()'s parameter list.
% This is simple to arrange, as shown for the computation of G.

F = numerical_Jacobian(x, @predict_model, [], [], u, dt);
G = numerical_Jacobian(u, @predict_modelu, [], [], x, dt);

x = predict_model(x, u, dt);
P = F*P*F' + G*Q*G';

%
%

function [x,P] = update(x, P, z, R, type)
% This update demonstrates a variety of KF update implementations. For most purposes, the 
% KF_cholesky_update is best.

zpred = observe_model(x);
v = observe_residual(z, zpred);
H = numerical_Jacobian(x, @observe_model, @observe_residual, []);

switch type
case 1
    [x,P] = KF_update_simple(x,P,v,R,H);
case 2
    [x,P]= KF_update_joseph(x,P,v,R,H);
case 3
    [x,P]= KF_update_cholesky(x,P,v,R,H);
case 4
    [x,P] = KF_update_IEKF(x,P, z,R, @iekf_observe_model, @iekf_jacobian_model, 10);
otherwise
    error('Invalid choice of KF update')
end

%
%

function x = predict_model(x, u, dt)
% Vehicle model is a simple bicycle model
WB = 2; % wheelbase

v = u(1); 
g = u(2);

x = [x(1) + v*dt*cos(g+x(3)); 
     x(2) + v*dt*sin(g+x(3));
     x(3) + v*dt*sin(g)/WB]; 
% NOTE: do NOT use pi_to_pi on x(3) -- it will cause problems with numerical Jacobian calculations.
% (Unless, of course, we pass a 'residual model' to the numerical_Jacobian function.)

%
%

function x = predict_modelu(u, x, dt)
% Second predict model, that has 'u' as first argument; this allows us to compute Jacobian wrt u
x = predict_model(x, u, dt);

%
%

function z = observe_model(x)
% We have a range-bearing measurement of the vehicle from a fixed observer at origin.
z = [sqrt(x(1)^2 + x(2)^2);
     atan2(x(2), x(1))];

%
%

function v = observe_residual(z1, z2)
% Given two observation values, compute their residual.
v = z1-z2;
v(2) = pi_to_pi(v(2)); % normalise angle to +/- pi

%
%

function v = iekf_observe_model(x,z)
% IEKF update expects an innovation
zpred = observe_model(x);
v = observe_residual(z, zpred);

%
%

function H = iekf_jacobian_model(x)
% IEKF update requires a function to compute H. This may be an analytical Jacobian, but
% we use a numerical approximation here just for convenience.
H = numerical_Jacobian(x, @observe_model, @observe_residual, []);

%
%

function h = setup_plots()
figure, hold on, axis equal, axis([-105 0 -20 40]), grid
h.xt = plot(0,0, 'sk', 'erasemode', 'xor'); % true vehicle position
h.x = plot(0,0, '*', 'erasemode', 'xor'); % vehicle position
h.P = plot(0,0, 'r', 'erasemode', 'xor'); % vehicle uncertainty
h.obs = plot(0,0, 'g', 'erasemode', 'xor'); % observation
h.path = plot(0,0, 'c', 'erasemode', 'xor'); % estimated path

%
%

function do_plots(h, x, P, xtrue, z, xpath)
set(h.xt, 'xdata', xtrue(1), 'ydata', xtrue(2))
set(h.x, 'xdata', x(1), 'ydata', x(2))
set(h.path, 'xdata', xpath(1,:), 'ydata', xpath(2,:))
set(h.obs, 'xdata', [0 z(1)*cos(z(2))], 'ydata', [0 z(1)*sin(z(2))])

pc = sigma_ellipse(x(1:2), P(1:2,1:2), 2);
set(h.P, 'xdata', pc(1,:), 'ydata', pc(2,:))

drawnow
