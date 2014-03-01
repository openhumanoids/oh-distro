function [P] = kf_P_MU(K,H,M)
% Function to propagate the error covariance matrix P for the Measurement Update 
% step.
%
% Versions:
%	1.0: 14/1/2011 - Basic structure
%
% D Fourie


P = (eye(size(M,1)) - K*H)*M;
