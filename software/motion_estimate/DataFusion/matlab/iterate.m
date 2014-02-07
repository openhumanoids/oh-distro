function [Result, Sys] = iterate(Param, Sys, Measurement, sRb)
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


% Tm = Sys.T; -- to be depreciated

if (nargin<4)
    sRb = eye(3);
end

% Measurement.INS.pose.utime
% Measurement.INS.pose.lQb
% Measurement.INS.pose.a_l
% Measurement.velocityResidual

% EKF
[F, L, Q] = dINS_EKFmodel_s2b(Measurement.INS.pose, sRb);
covariances.R = diag( 5E0*ones(3,1) );

Disc.B = 0;
Disc.C = [zeros(3,6), eye(3), zeros(3,6)];

% TIME UPDATE, PRIORI STATE=========================================================================
[Disc.A,covariances.Qd] = lti_disc(F, L, Q, Sys.dt);
Sys.priori = KF_timeupdate(Sys.posterior, 0, Disc, covariances);
Sys.priori.utime = Measurement.INS.pose.utime;


% MEASUREMENT UPDATE, POSTERIOR STATE===============================================================
Sys.posterior = KF_measupdate(Sys.priori, Disc, [Measurement.velocityResidual]);
Sys.posterior.utime = Sys.priori.utime;

% Sys.posterior.x

% Some results to look at later=====================================================================

Result = [];

