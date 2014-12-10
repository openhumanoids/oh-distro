function [priori] = KF_timeupdate(previous,u,sys_d,covariances)
% Function to propagate the state estimate and error covariance 
% with generic Time Update.
%
% RELIES ON:
%  previous.x           - Previous best state estimate
%          .P           -  Best state error covariance estimate
%  u                    - Input vector
%  sys_d.a              - Discrete state transition matrix
%       .b              -  Input matrix
%       .c              -  Measurement matrix
%  covariances.Qd       - Process noise error covariance
%             .Rd       - Measurment noise error covariance
%
% RETURNS:
%  priori.x             - priori state estimate, linear SS propagation
%        .M             -  state error covariance, Ricatti equations
%        .K             -  Kalman gain
%
% Versions:
%	1.0: 06/11/2011		- Basic Structure
%



priori.M = kf_P_TU(sys_d.A,covariances.Qd,previous.P);
priori.K = kf_K(priori.M,sys_d.C,covariances.R);
priori.x = sys_d.A*previous.x + sys_d.B*u;

