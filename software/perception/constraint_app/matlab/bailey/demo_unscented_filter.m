function demo_unscented_filter()
%function demo_unscented_filter()
% Simple demo of unscented transform and unscented update modules.
%
% The state is a vehicle pose [x;y;phi]. The predict model is a simple bicycle
% model with controls of velocity and steer-angle. The observe model is range-
% bearing from a fixed observer at the origin.
%
% Notice how this demo deals with discontinuous models (ie, the jump between 
% plus-minus pi in the observation model).
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
    xtrue = predict_model([xtrue;V;G], dt);
    ztrue = observe_model(xtrue);
    
    % simulate our measurements by adding noise to true values
    u = multivariate_gauss([V;G], Q, 1);
    z = multivariate_gauss(ztrue, R, 1);
    
    % predict
    [x,P] = predict(x, P, u, Q, dt);
    
    % update
    [x,P] = update(x, P, z, R);
    
    % plots
    xpath(:,i) = x(1:2);    
    do_plots(h, x, P, xtrue, z, xpath(:,1:i));
end

%
%

function [x,P] = predict(x, P, u, Q, dt)
% Passing the control parameters (u,Q) to the unscented transform requires first
% augmenting the state with these parameters.

% Augment state with control parameters
x = [x; u];
P = blkdiag(P,Q);

% Perform unscented transform
[x,P] = unscented_transform(@predict_model, [], x, P, dt); % notice how the additional parameter 'dt' is passed to the model
x(3) = pi_to_pi(x(3));

%
%

function [x,P] = update(x, P, z, R)
[x,P] = unscented_update(@observe_model, @observe_residual, x, P, z, R);

%
%

function x = predict_model(x, dt)
% Vehicle model is a simple bicycle model.
% Notice that this model is 'vectorised' in terms of x (ie, it can work for a matrix of x's).
WB = 2; % wheelbase

v = x(end-1,:); 
g = x(end,:);
b = g + x(3,:);

x = [x(1,:) + v.*dt.*cos(b); 
     x(2,:) + v.*dt.*sin(b);
     x(3,:) + v.*dt.*sin(g)/WB]; 
% NOTE: do NOT use pi_to_pi on x(3) -- it will cause averaging problems with unscented transform.
% Of course, we could pass the unscented transform a 'residual model', but by avoiding the
% normalisation here, we don't need to.

%
%

function z = observe_model(x)
% We have a range-bearing measurement of the vehicle from a fixed observer at origin.
% Notice, once again, that the model is vectorised in terms of x.
z = [sqrt(x(1,:).^2 + x(2,:).^2);
     atan2(x(2,:), x(1,:))];

%
%

function v = observe_residual(z1, z2)
% Given two observation values, compute their normalised residual.
% Notice, once again, that the model is vectorised in terms of z1 and z2.
v = z1-z2;
v(2,:) = pi_to_pi(v(2,:));

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

pc = sigma_ellipse(x(1:2), P(1:2,1:2), 2);
set(h.P, 'xdata', pc(1,:), 'ydata', pc(2,:))

set(h.obs, 'xdata', [0 z(1)*cos(z(2))], 'ydata', [0 z(1)*sin(z(2))])

drawnow
