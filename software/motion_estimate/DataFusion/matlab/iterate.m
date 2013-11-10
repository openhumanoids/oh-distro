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
% x = [dPsi dbg dP dV dba]

% storing measurement data for later plotting with eef
% Sys.measured = Measurement.measured;

T = Sys.T;

Sys.Cont.F = zeros(15);
Sys.Cont.F(1:3,4:6) = q2R(Measurement.INS.pose.q); % here we need the body to local rotation matrix
Sys.Cont.F(7:9,10:12) = eye(3);
Sys.Cont.F(10:12,13:15) = q2R(Measurement.INS.pose.q); % here again we need to keep track of biases in the body frame

% Sys.Cont.F(4:6,7:9) = vec2skew(-Measurement.INS.Pose.f_l);

% Sys.Cont.F(4:6,13:15) = -q2R(Measurement.INS.Pose.q);
% Sys.Cont.F(7:9,10:12) = q2R(Measurement.INS.Pose.q);

Sys.Cont.Q = diag([1E-8*ones(1,6), 1E-8*ones(1,3), 1E-8*ones(1,6)]);

[Sys.Disc.A,Sys.covariances.Qd] = lti_disc(Sys.Cont.F, [], Sys.Cont.Q, T);

Sys.Disc.B = zeros(15,1);

Sys.Disc.C = [eye(3), zeros(3,12);...
              zeros(3,6), eye(3), zeros(3,6)];

Sys.covariances.R = diag([0.03*ones(3,1); 0.03*ones(3,1)]); % Assume LegOdo position measurement update      
      
% TIME UPDATE, PRIORI STATE=========================================================================

Sys.priori = KF_timeupdate(Sys.posterior, 0, Sys.Disc, Sys.covariances);
Sys.priori.utime = Measurement.INS.pose.utime;

% MEASUREMENT UPDATE, POSTERIOR STATE===============================================================

dq = Measurement.quaternionManifoldResidual;
dE = q2e(dq);
dP = Measurement.positionResidual;


disp(['iterate dE ' num2str(dE')]);

Sys.posterior = KF_measupdate(Sys.priori, Sys.Disc, [dE; dP]);
Sys.posterior.utime = Sys.priori.utime;

disp(['iterate gyro bias est ' num2str(Sys.posterior.x(4:6)')])
disp(['iterate acc bias est ' num2str(Sys.posterior.x(13:15)')])

disp(['iterate pos err est ' num2str(Sys.posterior.x(7:9)')])

% Some results to look at later=====================================================================


Result = [];

