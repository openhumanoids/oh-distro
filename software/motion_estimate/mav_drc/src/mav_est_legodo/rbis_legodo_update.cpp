#include "rbis_legodo_update.hpp"
#include <path_util/path_util.h>
#include <string>

using namespace std;

namespace MavStateEst {


LegOdoHandler::LegOdoHandler(lcm::LCM* lcm_recv,  lcm::LCM* lcm_pub, 
      BotParam * param, ModelClient* model)
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
  
  if (leg_odo_->Update(msg)){
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
