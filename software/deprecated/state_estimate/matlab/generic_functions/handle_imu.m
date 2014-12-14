function [RecursiveData] = handle_imu(RecursiveData, imu)

RecursiveData.pose.utime = imu.utime;
dt = (RecursiveData.pose.utime - RecursiveData.pose__k1.utime)*1e-6;

inertialData = init_inertialData(9.8);

inertialData.predicted.utime = imu.utime;
inertialData.measured.w_b = imu.angular_velocity;
inertialData.measured.a_b = imu.linear_acceleration;
inertialData.predicted.w_b = inertialData.measured.w_b - RecursiveData.INSCompensator.biases.bg;
inertialData.predicted.a_b = inertialData.measured.a_b - RecursiveData.INSCompensator.biases.ba;

[RecursiveData.pose__k1, RecursiveData.INSCompensator] = Update_INS(pose__k1, RecursiveData.INSCompensator);
RecursiveData.pose = INS_lQb([], RecursiveData.pose__k1, RecursiveData.pose__k2, inertialData);

    
% imudata.utime = imu.utime;
% imudata.ddp = imu.linear_acceleration;
% imudata.da = imu.angular_velocity;
% imudata.q = imu.orientation;
% imudata.gravity = [0;0;9.8];
% 
% pose = INS_Mechanisation(pose__, imudata);


% platform rates in the estimated reference frame
RecursiveData.pose.w_l = qrot(qconj(RecursiveData.pose.lQb),imudata.angular_velocity);

% convert R to q -- quaternion attitude computer to be used as primary in
% the future
% pose.q = R2q(pose.R);
