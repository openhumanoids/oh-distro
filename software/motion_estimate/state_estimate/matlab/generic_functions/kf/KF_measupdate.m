function [posterior] = KF_measupdate(priori, sys_d, y)
% Function to perform a generic measurement update on state vector x 
% and error covariance matrix P.
%
% RELIES ON:
%  priori.x
%        .K
%        .M
%  sys_d.c        (or H matrix)
%  y              as measurement column vector
%
% RETURNS:
%  posterior.innov
%           .x
%           .P
%
% Versions:
%	1.0: 06/11/2011	- Basic structure
%


posterior.innov = kf_innov(y,sys_d.C,priori.x);
posterior.dx =  priori.K * posterior.innov;
posterior.x = priori.x + posterior.dx;
posterior.P = kf_P_MU(priori.K,sys_d.C,priori.M);

