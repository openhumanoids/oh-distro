function [x_hat, P] = kf_measupdate_innov(x, innov, K, H, M)
% Function to perform a generic measurement update on state vector x 
% and error covariance matrix P.
%
% Versions:
%	1.0: 11/10/2011	- Basic structure, implemented to allow nonlinear measurement models
%

x_hat = x + K*innov;
P = kf_P_MU(K,H,M);

