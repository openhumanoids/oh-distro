function [pose] = handle_imu(pose__, imudata)

pose.utime = imudata.utime;
dt = (pose.utime - pose__.utime)*1e-6;

% Newton 2 mechanisation
pose.P = pose__.P + (pose__.V) * dt; 
pose.f_l = pose__.R * (imudata.linear_acceleration) - [0;0;9.81];
pose.V = pose__.V + pose.f_l * dt;
pose.R = closed_form_DCM_farrell(-imudata.angular_velocity,pose__.R,dt);

% platform rates in the estimated reference frame
pose.w_l = pose__.R*imudata.angular_velocity;

% convert R to q -- quaternion attitude computer to be used as primary in
% the future
pose.q = R2q(pose.R);
