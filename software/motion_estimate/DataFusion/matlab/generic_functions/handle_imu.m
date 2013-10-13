function [pose] = handle_imu(pose__, imu)

pose.utime = imu.utime;
dt = (pose.utime - pose__.utime)*1e-6;

% Newton 2 mechanisation
% pose.P = pose__.P + (pose__.V) * dt; 
% pose.f_l = pose__.R * (imudata.linear_acceleration) - [0;0;9.81];
% pose.V = pose__.V + 0.5*(pose__.f_l + pose.f_l) * dt;
% pose.R = closed_form_DCM_farrell(imudata.angular_velocity,pose__.R,dt);

imudata.utime = imu.utime;
imudata.ddp = imu.linear_acceleration;
imudata.da = imu.angular_velocity;
 

pose = INS_Mechanisation(pose__, imudata);

% platform rates in the estimated reference frame
pose.w_l = pose__.R*imudata.da;

% convert R to q -- quaternion attitude computer to be used as primary in
% the future
pose.q = R2q(pose.R);
