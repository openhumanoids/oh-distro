function [Result, Sys] = iterate(Param, Sys, Measurement)
% This function must be able to be called as part of a separate process, given Param, Sys and a Measurement
%
% Requires:
%   Sys.T
%   Sys.priori
%   Sys.posterior
%   Measurement.INS.Pose
%
% Measurement update step requires:
%   Measurement.LegOdo.Pose.P
%
% Versions,
%	0.5: 8/9/2013	- Basic Structure for MIT DRC pelvis navigation
%

% This is a Kalman Filter implementation

% state vector
% x = [dPsi dbg dV dba dP]

% storing measurement data for later plotting with eef
% Sys.measured = Measurement.measured;

Tm = Sys.T;


% EKF
[F, L, Q] = dINS_EKFmodel(Measurement.INS);
Disc.C = [zeros(3,6), eye(3), zeros(3,6)];
covariances.R = diag( 1E0*ones(3,1) );


% TIME UPDATE, PRIORI STATE=========================================================================
[Disc.A,covariances.Qd] = lti_disc(F, L, Q, Tm);
Sys.priori = KF_timeupdate(Sys.posterior, 0, Disc, covariances);
Sys.priori.utime = Measurement.INS.pose.utime;


% MEASUREMENT UPDATE, POSTERIOR STATE===============================================================
posterior = KF_measupdate(Sys.priori, Disc, [Measurement.velocityResidual]);
Sys.posterior.utime = Sys.priori.utime;


% Some results to look at later=====================================================================

Result = [];

