function [] = sendimu(data,lc)

% lc = lcm.lcm.LCM.getSingleton();

msg = drc.imu_t();

msg.utime = data.imu.utime;
msg.frame_id = 'Test';
msg.angular_velocity = data.imu.gyr;
msg.angular_velocity_covariance = zeros(1,9);
msg.orientation = data.imu.q;
msg.orientation_covariance = zeros(1,9);
msg.linear_acceleration = data.imu.acc;
msg.linear_acceleration_covariance = zeros(1,9);


lc.publish('IMU_DATA', msg);

