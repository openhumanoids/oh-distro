#include "rbis_legodo_external_update.hpp"
#include <path_util/path_util.h>

namespace MavStateEst {


LegOdoExternalHandler::LegOdoExternalHandler(lcm::LCM* lcm_recv, lcm::LCM* lcm_pub, BotParam * param):
  lcm_recv(lcm_recv), lcm_pub(lcm_pub){
  std::cout << "LegOdoExternalHandler will compute from incoming delta messages\n";  
  // NB: This module will subscribe to the channel specified in the 
  // state_estimator.legodo_external block of the cfg file
  // but read all perameters from state_estimator.legodo block
  leg_odo_common_ = new LegOdoCommon(lcm_recv, lcm_pub, param);
}


RBISUpdateInterface * LegOdoExternalHandler::processMessage(const drc::pose_transform_t * msg)
{
  /// ... insert handling and special cases here.
  bool verbose = false;
  if (verbose) 
    std::cout << "LegOdoExternalHandler: received LegOdo delta msg\n";


  BotTrans odo_positionT;
  memset(&odo_positionT, 0, sizeof(odo_positionT));
  
  
  BotTrans odo_deltaT;
  memset(&odo_deltaT, 0, sizeof(odo_deltaT));
  memcpy(odo_deltaT.trans_vec, msg->translation, 3 * sizeof(double));
  memcpy(odo_deltaT.rot_quat,  msg->rotation   , 4 * sizeof(double));
  int64_t utime = msg->utime;
  int64_t prev_utime = msg->prev_utime;

  // TODO: the pose_transform_t doesn't provide any information about
  // the confidence of the estimate, this field needs to be added
  float odo_delta_status =0.0; // by default assume accurate
  int odo_position_status = false; // unknown in this mode
  return leg_odo_common_->createMeasurement(odo_positionT, odo_deltaT, 
                                            utime, prev_utime, 
                                            odo_position_status, odo_delta_status);

}
}
