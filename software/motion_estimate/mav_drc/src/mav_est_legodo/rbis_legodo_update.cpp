#include "rbis_legodo_update.hpp"
#include <path_util/path_util.h>
#include <string>

using namespace std;

namespace MavStateEst {


LegOdoHandler::LegOdoHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model): lcm_pub(lcm_pub)
{
  std::cout << "LegOdo will compute directly, in thread\n";

  boost::shared_ptr<lcm::LCM> lcm_recv_boost = boost::shared_ptr<lcm::LCM>(lcm_recv);
  boost::shared_ptr<lcm::LCM> lcm_pub_boost = boost::shared_ptr<lcm::LCM>(lcm_pub);
  boost::shared_ptr<ModelClient> model_boost = boost::shared_ptr<ModelClient>(model);
  leg_odo_ = new leg_odometry(  lcm_recv_boost , lcm_pub_boost ,
                               param, model_boost );

  leg_odo_common_ = new LegOdoCommon(param);
}

RBISUpdateInterface * LegOdoHandler::processMessage(const drc::robot_state_t *msg)
{
  /// ... insert handling and special cases here.
  bool verbose = false;
  if (verbose) std::cout << "LegOdoHandler: received EST_ROBOT_STATE msg\n";

  Eigen::Isometry3d delta_odo;
  int64_t utime, prev_utime;
  
  // The changes below will eventually allow us
  // to read directly from ATLAS_STATE and to publish EST_ROBOT_STATE
  // This is unused if using the "basic mode":
  if (1==0){
    Eigen::Isometry3d world_to_body_bdi;
    world_to_body_bdi.setIdentity();
    world_to_body_bdi.translation()  << msg->pose.translation.x, msg->pose.translation.y, msg->pose.translation.z;
    Eigen::Quaterniond quat = Eigen::Quaterniond(msg->pose.rotation.w, msg->pose.rotation.x, 
                                               msg->pose.rotation.y, msg->pose.rotation.z);
    world_to_body_bdi.rotate(quat);   
    leg_odo_->setPoseBDI( world_to_body_bdi ); 
  }
  
  // Don't publish then working live:
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
  lcm_pub->publish("POSE_BDI", &bdipose);
  
  
  ///////////////////////////////////////////////
  
  
  leg_odo_->setFootForces(msg->force_torque.l_foot_force_z,msg->force_torque.r_foot_force_z);
    
  if (leg_odo_->updateOdometry(msg->joint_name, msg->joint_position,
                           msg->joint_velocity, msg->joint_effort, msg->utime)){
    leg_odo_->getDeltaLegOdometry(delta_odo, utime, prev_utime);
  } else {
    std::cout << "Leg Odometry is not valid ==============================\n";
    return NULL;
  }

  BotTrans msgT;
  memset(&msgT, 0, sizeof(msgT));
//  memcpy(msgT.trans_vec, msg->translation, 3 * sizeof(double));
//  memcpy(msgT.rot_quat,  msg->rotation   , 4 * sizeof(double));

  Eigen::Vector3d motion_T = delta_odo.translation();
  Eigen::Quaterniond motion_R = Eigen::Quaterniond(delta_odo.rotation());
  msgT.trans_vec[0] = motion_T(0);
  msgT.trans_vec[1] = motion_T(1);
  msgT.trans_vec[2] = motion_T(2);
  msgT.rot_quat[0] = motion_R.w();
  msgT.rot_quat[1] = motion_R.x();
  msgT.rot_quat[2] = motion_R.y();
  msgT.rot_quat[3] = motion_R.z();  

  return leg_odo_common_->createMeasurement(msgT, utime, prev_utime);

}
}
