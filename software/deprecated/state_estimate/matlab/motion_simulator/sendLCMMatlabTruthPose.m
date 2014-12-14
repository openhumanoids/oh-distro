function sendLCMMatlabTruthPose( truePose, trueMatlabPoseMsg, lc )
%SENDLCMMATLABTRUTHPOSE Summary of this function goes here
%   Detailed explanation goes here


trueMatlabPoseMsg.utime = truePose.utime;

trueMatlabPoseMsg.pose.translation.x = truePose.P_l(1);
trueMatlabPoseMsg.pose.translation.y = truePose.P_l(2);
trueMatlabPoseMsg.pose.translation.z = truePose.P_l(3);

trueMatlabPoseMsg.pose.rotation.w = truePose.q(1);
trueMatlabPoseMsg.pose.rotation.x = truePose.q(2);
trueMatlabPoseMsg.pose.rotation.y = truePose.q(3);
trueMatlabPoseMsg.pose.rotation.z = truePose.q(4);
trueMatlabPoseMsg.twist.linear_velocity.x = truePose.V_l(1);
trueMatlabPoseMsg.twist.linear_velocity.y = truePose.V_l(2);
trueMatlabPoseMsg.twist.linear_velocity.z = truePose.V_l(3);

% These rotations are in the local frame, but this should not be our standard mode of operations, this is available for testing and sanity checking only
trueMatlabPoseMsg.twist.angular_velocity.x = truePose.w_l(1);
trueMatlabPoseMsg.twist.angular_velocity.y = truePose.w_l(2);
trueMatlabPoseMsg.twist.angular_velocity.z = truePose.w_l(3); 

trueMatlabPoseMsg.local_linear_acceleration.x = truePose.f_l(1);
trueMatlabPoseMsg.local_linear_acceleration.x = truePose.f_l(2);
trueMatlabPoseMsg.local_linear_acceleration.x = truePose.f_l(3);

lc.publish('TRUTH_TRAJ_MATLAB', trueMatlabPoseMsg);

end

