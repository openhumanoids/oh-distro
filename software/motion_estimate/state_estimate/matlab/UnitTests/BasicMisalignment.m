

clc
clear

% Basic platform misalignment Test

% Start at 100Hz to test with Microstrain
dt = 0.01;

% Gravity in the terrestrial navigation frame
gn = [0;0;1]; % forward-left-up frame

% True rotation matrix
true.lRb = eye(3);

% Error angles
true.dPsi = [0.001;0;0];


true.fb = [0;0;0];

% Now lets try estimate the misalignment angles
predicted.Vl = [0;0;0];
predicted.dVl = [0;0;0];
predicted.dPsi = [0.001;0;0];

measured.Vl = [0;0;0];


% this will end up in a recursive loop

% misaligned rotation matrix
predicted.lRb = true.lRb*(eye(3) + vec2skew(true.dPsi));

% Specific force and acceleration in body frame
true.ab = true.fb + true.lRb*gn;

% Body measured accelerations
measured.ab = true.ab; % we are not adding any noise yet

% predicted local frame acceleration
predicted.al = predicted.lRb'*measured.ab;
predicted.fl = predicted.al - gn;

disp 'Predicted local frame specific force, fl, is: '
disp(num2str(predicted.fl))
disp ' '

% predict local frame velocity
predicted.Vl = predicted.Vl + dt*predicted.fl;

% propagate velocity error model
predicted.dVl = predicted.dVl - vec2skew(predicted.fl) * predicted.dPsi;

% Compute the local frame velocity innovation
disp 'Innovation'
innov.Vl = measured.Vl - (predicted.Vl + predicted.dVl);

innov.Vl

K = eye(3);

predicted.dVl = predicted.dVl + K*innov.Vl;
predicted.dVl


