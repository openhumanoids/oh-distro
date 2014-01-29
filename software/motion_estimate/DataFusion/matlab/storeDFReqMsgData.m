function REQM = storeDFReqMsgData(k, REQM, msg)


REQM.utime(k) = msg.utime;
toc
REQM.ba(k,1:3) = [msg.accBiasEst.x, msg.accBiasEst.y, msg.accBiasEst.z];
REQM.bg(k,1:3) = [msg.gyroBiasEst.x, msg.gyroBiasEst.y, msg.gyroBiasEst.z];
REQM.a_b(k,1:3) = [msg.predicted_a_b.x, msg.predicted_a_b.y, msg.predicted_a_b.z];
REQM.w_b(k,1:3) = [msg.predicted_w_b.x, msg.predicted_w_b.y, msg.predicted_w_b.z];
REQM.a_l(k,1:3) = [msg.local_linear_acceleration.x, msg.local_linear_acceleration.y, msg.local_linear_acceleration.z];
REQM.f_l(k,1:3) = [msg.local_linear_force.x, msg.local_linear_force.y, msg.local_linear_force.z];
REQM.P_l(k,1:3) = [msg.pose.translation.x, msg.pose.translation.y, msg.pose.translation.z];
REQM.V_l(k,1:3) = [msg.twist.linear_velocity.x, msg.twist.linear_velocity.y, msg.twist.linear_velocity.z];
REQM.lQb(k,1:4) = [msg.pose.rotation.w, msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z];

