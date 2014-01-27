function REQM = storeDFReqMsgData(k, REQM, msg)


REQM.utime(k) = msg.utime;
REQM.P_l(k,1:3) = [msg.pose.translation.x, msg.pose.translation.y, msg.pose.translation.z];
REQM.V_l(k,1:3) = [msg.twist.linear_velocity.x, msg.twist.linear_velocity.y, msg.twist.linear_velocity.z];
REQM.lQb(k,1:4) = [msg.pose.rotation.w, msg.pose.rotation.x, msg.pose.rotation.y, msg.pose.rotation.z];
REQM.a_l(k,1:3) = [msg.local_linear_force.x, msg.local_linear_force.y, msg.local_linear_force.z];

