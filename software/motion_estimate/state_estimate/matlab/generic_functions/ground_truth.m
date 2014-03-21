function [pose] = ground_truth(utime, pose__, imudata)

% time first
if (imudata.utime<pose__.utime)
    disp('ground_truth.m: ERROR, you cannot integrate backwards in time here.');
end

pose.utime = imudata.utime;
dt = (pose.utime - pose__.utime)*1e-6;

% Newton 2 mechanisation
pose.P = pose__.P + (pose__.V) * dt; 
pose.f_l = pose__.R * (imudata.ddp);
pose.V = pose__.V + 0.5*(pose.f_l + pose__.f_l) * dt;
pose.R = closed_form_DCM_farrell(imudata.da,pose__.R,dt);
