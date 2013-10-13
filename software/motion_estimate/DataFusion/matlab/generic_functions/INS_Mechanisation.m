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
    disp('ground_truth.m: ERROR, you cannot integrate backwards in time here.');
end

pose.utime = imudata.utime;
dt = (pose.utime - pose__.utime)*1e-6;


pose.R = closed_form_DCM_farrell( 0.5*(pose__.da+imudata.da)*dt , pose__.R); % trapezoidal integration, before application through the exponential map

% Newton 2 mechanisation
pose.P = pose__.P + (pose__.V) * dt; 

% pose.f_l = q2R(imudata.q)' * (imudata.ddp);
pose.f_l = pose.R * (imudata.ddp);

pose.V = pose__.V + 0.5*(pose.f_l + pose__.f_l) * dt;

pose.da = imudata.da;

