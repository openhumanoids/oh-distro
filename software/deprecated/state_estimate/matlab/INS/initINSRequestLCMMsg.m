function [ msg ] = initINSRequestLCMMsg( )
%PREPAREINSUPDATELCMMSG Summary of this function goes here
%   Detailed explanation goes here


msg = drc.ins_update_request_t();
msg.utime = 0;
msg.pose = drc.position_3d_t();
msg.pose.translation = drc.vector_3d_t();
msg.pose.rotation = drc.quaternion_t();
msg.twist = drc.twist_t();
msg.twist.linear_velocity = drc.vector_3d_t();
msg.twist.angular_velocity = drc.vector_3d_t();
msg.local_linear_acceleration = drc.vector_3d_t();
msg.local_linear_force = drc.vector_3d_t();
msg.predicted_a_b = drc.vector_3d_t();
msg.predicted_w_b = drc.vector_3d_t();
msg.gyroBiasEst = drc.vector_3d_t();
msg.accBiasEst = drc.vector_3d_t();
msg.referencePos_local = drc.vector_3d_t();
msg.referenceVel_local = drc.vector_3d_t();
msg.referenceVel_body = drc.vector_3d_t();
msg.referenceQ_local = drc.quaternion_t();
msg.updateType = 0;

end

