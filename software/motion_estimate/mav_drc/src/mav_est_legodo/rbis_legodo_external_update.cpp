#include "rbis_legodo_external_update.hpp"
#include <path_util/path_util.h>

namespace MavStateEst {


LegOdoExternalHandler::LegOdoExternalHandler(BotParam * param)
{
  std::cout << "LegOdoExternalHandler will compute from incoming delta messages\n";  
  // NB: This module will subscribe to the channel specified in the 
  // state_estimator.legodo_external block of the cfg file
  // but read all perameters from state_estimator.legodo block
  leg_odo_common_ = new LegOdoCommon(param);
}


RBISUpdateInterface * LegOdoExternalHandler::processMessage(const drc::pose_transform_t * msg)
{
  /// ... insert handling and special cases here.
  bool verbose = false;
  if (verbose) 
    std::cout << "LegOdoExternalHandler: received LegOdo delta msg\n";


  BotTrans msgT;
  memset(&msgT, 0, sizeof(msgT));
  memcpy(msgT.trans_vec, msg->translation, 3 * sizeof(double));
  memcpy(msgT.rot_quat,  msg->rotation   , 4 * sizeof(double));
  int64_t utime = msg->utime;
  int64_t prev_utime = msg->prev_utime;

  return leg_odo_common_->createMeasurement(msgT, utime, prev_utime);

}
}
