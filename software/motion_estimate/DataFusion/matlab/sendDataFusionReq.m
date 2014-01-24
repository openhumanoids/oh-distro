function sendDataFusionReq(INSpose, msg, lc)


msg.utime = INSpose.utime;
msg.pose.translation.x = INSpose.P_l(1);
msg.pose.translation.y = INSpose.P_l(2);
msg.pose.translation.z = INSpose.P_l(3);

msg.twist.linear_velocity.x = INSpose.V_l(1);
msg.twist.linear_velocity.y = INSpose.V_l(2);
msg.twist.linear_velocity.z = INSpose.V_l(3);

msg.pose.rotation.w = INSpose.lQb(1);
msg.pose.rotation.x = INSpose.lQb(2);
msg.pose.rotation.y = INSpose.lQb(3);
msg.pose.rotation.z = INSpose.lQb(4);

msg.twist.angular_velocity.x = INSpose.w_l(1);
msg.twist.angular_velocity.y = INSpose.w_l(2);
msg.twist.angular_velocity.z = INSpose.w_l(3);

msg.local_linear_force.x = INSpose.f_l(1);
msg.local_linear_force.y = INSpose.f_l(2);
msg.local_linear_force.z = INSpose.f_l(3);

msg.local_linear_acceleration.x = INSpose.a_l(1);
msg.local_linear_acceleration.y = INSpose.a_l(2);
msg.local_linear_acceleration.z = INSpose.a_l(3);


% reference measurements
msg.referencePos_local.x = 0;
msg.referencePos_local.y = 0;
msg.referencePos_local.z = 0;

% Should add noise to this in the near future
msg.referenceVel_local.x = 0;
msg.referenceVel_local.y = 0;
msg.referenceVel_local.z = 0;

msg.referenceVel_body.x = 0;
msg.referenceVel_body.y = 0;
msg.referenceVel_body.z = 0;

msg.referenceQ_local.w = 1;
msg.referenceQ_local.x = 0;
msg.referenceQ_local.y = 0;
msg.referenceQ_local.z = 0;

msg.updateType = 3;

lc.publish('SE_MATLAB_DATAFUSION_REQ', msg);
