function [innov] = kf_innov(y,H,x)
% Function to compute the innovation. Standard KF equation
%
% Versions:
%	1.0: 17/1/2011 - Basic structure
%

% this should allow for non-linear cases in the future- via fnction handles
innov = y-H*x;
