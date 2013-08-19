function [M] = kf_P_TU(Phi,Qd,P)
% Function to propagate the error covariance matrix for the time update cycle.
%
% Versions:
%	1.0: 14/1/2011 - Basic Structure
%

M = Phi*P*(Phi')+Qd;
