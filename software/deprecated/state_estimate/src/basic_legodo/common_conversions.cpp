// this file is not used --- remove from git
// when i figure out how!

Eigen::Isometry3d KDLToEigen(KDL::Frame tf){
  Eigen::Isometry3d tf_out;
  tf_out.setIdentity();
  tf_out.translation()  << tf.p[0], tf.p[1], tf.p[2];
  Eigen::Quaterniond q;
  tf.M.GetQuaternion( q.x() , q.y(), q.z(), q.w());
  tf_out.rotate(q);    
  return tf_out;
}

bot_core::pose_t getPoseAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();  
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();  
  pose_msg.orientation[1] =  r_x.x();  
  pose_msg.orientation[2] =  r_x.y();  
  pose_msg.orientation[3] =  r_x.z();  
  return pose_msg;
}

bot_core::pose_t getRobotStatePoseAsBotPose(drc::robot_state_t msg){
    bot_core::pose_t bdipose;
    bdipose.utime = msg->utime;
    bdipose.pos[0] = msg->pose.translation.x;
    bdipose.pos[1] = msg->pose.translation.y;
    bdipose.pos[2] = msg->pose.translation.z;
    bdipose.orientation[0] = msg->pose.rotation.w;
    bdipose.orientation[1] = msg->pose.rotation.x;
    bdipose.orientation[2] = msg->pose.rotation.y;
    bdipose.orientation[3] = msg->pose.rotation.z;
    bdipose.vel[0] = msg->twist.linear_velocity.x;
    bdipose.vel[1] = msg->twist.linear_velocity.y;
    bdipose.vel[2] = msg->twist.linear_velocity.z;
    bdipose.rotation_rate[0] = msg->twist.angular_velocity.x;
    bdipose.rotation_rate[1] = msg->twist.angular_velocity.y;
    bdipose.rotation_rate[2] = msg->twist.angular_velocity.z;  
  
    /*
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();  
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();  
  pose_msg.orientation[1] =  r_x.x();  
  pose_msg.orientation[2] =  r_x.y();  
  pose_msg.orientation[3] =  r_x.z();  */
  return bdipose;
}

void insertPoseInRobotState(drc::robot_state_t& msg, Eigen::Isometry3d pose){
  msg.pose.translation.x = pose.translation().x();
  msg.pose.translation.y = pose.translation().y();
  msg.pose.translation.z = pose.translation().z();
  Eigen::Quaterniond r_x(pose.rotation());
  msg.pose.rotation.w = r_x.w();  
  msg.pose.rotation.x = r_x.x();  
  msg.pose.rotation.y = r_x.y();  
  msg.pose.rotation.z = r_x.z();  
}