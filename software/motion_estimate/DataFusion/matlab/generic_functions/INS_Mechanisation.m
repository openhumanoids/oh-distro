function [pose] = INS_Mechanisation(pose__, imudata)
% requires
%
% imudata.utime
% imudata.da
% imudata.ddp
% imudata.q (under special testing conditions)
%
% must still gravity as a parameter, which is passed to this function

% time first
if (imudata.utime<pose__.utime)
    disp('INS_Mechanisation.m: ERROR, you cannot integrate backwards in time here.');
end

pose.utime = imudata.utime;
dt = (pose.utime - pose__.utime)*1e-6;

% Newton 2 mechanisation
pose.P_l = pose__.P_l + (pose__.V_l) * dt; 

pose.R = closed_form_DCM_farrell( 0.5*(pose__.da+imudata.da)*dt , pose__.R); % trapezoidal integration, before application through the exponential map
pose.f_l = pose.R' * (imudata.ddp) - imudata.gravity;

% pose.f_l = q2R(imudata.q)' * (imudata.ddp);

pose.V_l = pose__.V_l + 0.5*(pose.f_l + pose__.f_l) * dt;

pose.da = imudata.da;

