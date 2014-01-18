function [pose] = INS_lQb(PARAM, pose__k1, pose__k2, inertialData)
% Ouput
%   -- pose (includes all states of the classical INS mechanization)
% Required Input
% Inertial measurements
%   -- inertialData.predicted.utime
%   -- inertialData.predicted.wb
%   -- inertialData.predicted.ab
%   -- inertialData.predicted.lQb
% Previous poses
%   -- pose__k1
%   -- pose__k2
%
% To be added in the future
%   -- inertialData.gw
%
%   -- PARAM.w_ie_w
%   -- PARAM.Re_maj
%   -- PARAM.Re_min
%
% Next development steps for this function include
%   -- convert to trapezoidal integrations
%   -- Support pre-integrals
%   -- Convert to dual quaternion skrew representation
%

% time first
if (inertialData.predicted.utime<pose__.utime)
    disp('INS_lQb.m: ERROR, you cannot integrate backwards in time.');
end

pose.utime = inertialData.predicted.utime;
dt = (pose.utime - pose__k1.utime)*1e-6; % convert units to seconds

% predict local frame accelerations
pose.a_l = qrot(qconj(pose__k1.lQb),inertialData.predicted.ab);
pose.f_l = (inertialData.predicted.al - inertialData.gw)';
pose.P_l = pose__k1.P_l + 0.5*dt*(pose__k1.V_l + pose__k2.V_l);
pose.V_l = pose__k1.V_l + 0.5*dt*(pose__k1.f_l + pose__k2.f_l);

pose.lQb = zeroth_int_Quat_closed_form(-inertialData.predicted.wb, pose__k1.lQb, dt);

return % Previous implementation is temporarily kept below




%% Previous DCM based mechanization -- to be depreciated, in favor of full quaternion solution


pose.utime = imudata.utime;
dt = (pose.utime - pose__.utime)*1e-6;

% Newton 2 mechanisation
pose.P_l = pose__.P_l + (pose__.V_l) * dt; 

% this propagation gives us the local to body quaternion
pose.R = closed_form_DCM_farrell( 0.5*(pose__.da+imudata.da)*dt , pose__.R); % trapezoidal integration, before application through the exponential map

pose.a_l = pose.R' * (imudata.ddp);
pose.f_l = pose.a_l - imudata.gravity; % we break this step up specifically for testing purposes

% pose.f_l = q2R(imudata.q)' * (imudata.ddp);

pose.V_l = pose__.V_l + 0.5*(pose.f_l + pose__.f_l) * dt;

pose.da = imudata.da;
pose.q = R2q(pose.R');
pose.E = q2e(pose.q);

