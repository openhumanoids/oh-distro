function [] = sendpose_lcm(lc, msg, pose)


msg.utime = pose.utime;
msg.pose.translation.x = pose.P_l(1);
msg.pose.translation.y = pose.P_l(2);
msg.pose.translation.z = pose.P_l(3);
msg.pose.rotation.w = pose.q(1);
msg.pose.rotation.x = pose.q(2);
msg.pose.rotation.y = pose.q(3);
msg.pose.rotation.z = pose.q(4);
msg.twist.linear_velocity.x = pose.V_l(1);
msg.twist.linear_velocity.y = pose.V_l(2);
msg.twist.linear_velocity.z = pose.V_l(3);

msg.twist.angular_velocity.x = pose.w_l(1);
msg.twist.angular_velocity.y = pose.w_l(2);
msg.twist.angular_velocity.z = pose.w_l(3);

msg.local_linear_acceleration.x = pose.f_l(1);
msg.local_linear_acceleration.y = pose.f_l(2);
msg.local_linear_acceleration.z = pose.f_l(3);

lc.publish('INS_ESTIMATE', msg);
