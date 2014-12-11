function observe_heading(phi, useheading)
%function observe_heading(phi, useheading)
%
% Perform state update for a given heading measurement, phi,
% with fixed measurement noise: sigmaPhi
global XX PX

if useheading==0, return, end
sigmaPhi= 1*pi/180; % radians, heading uncertainty

H= zeros(1,length(XX));
H(3)= 1;
v= pi_to_pi(phi - XX(3));

[XX,PX] = KF_update_cholesky(XX,PX, v, sigmaPhi^2, H);
