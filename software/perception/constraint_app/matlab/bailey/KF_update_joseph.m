function [x,P] = KF_update_joseph(x,P,v,R,H)
%function [x,P] = KF_update_joseph(x,P,v,R,H)
%
% Calculate the KF (or EKF) update given the prior state [x,P], the innovation v, the 
% observe uncertainty R, and the (linearised) observation model H. This module uses the 
% Joseph-form covariance update. 
%
%   Reference: Bar-Shalom "Estimation with Applications...", 2001, p302. 
%
% Tim Bailey 2003, modified 2005.

% Innovation covariance
PHt = P*H';
S = H*PHt + R;
Si = inv_posdef(S);

% Kalman gain
W = PHt*Si;

% State update
x = x + W*v; 

% Joseph-form covariance update
C = eye(size(P)) - W*H;
P = C*P*C' + W*R*W';
PSD_check = chol(P);

%
%

function Ai = inv_posdef(A)
Ac = chol(A);
Aci = inv(Ac);
Ai = Aci*Aci';
