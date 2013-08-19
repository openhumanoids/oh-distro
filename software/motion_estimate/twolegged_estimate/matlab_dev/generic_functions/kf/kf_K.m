function [K] = kf_K(M,H,R)
% Determine the Kalman gain matrix
%
% Versions,
%	1.0: 11/10/2011	- Basic structure
%

MHT = M*H';
K = MHT*inv(H*MHT + R);

