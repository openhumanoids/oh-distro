// This class implements a torque adjustment suggested
// by IHMC

#include <estimate_tools/torque_adjustment.hpp>
#include "atlas/AtlasJointNames.h"
#include "atlas/AtlasControlTypes.h"
#include "atlas/AtlasJointNames.h"

namespace EstimateTools {

TorqueAdjustment::TorqueAdjustment(){

}


double magnitudeLimit(double val_in){
  if (val_in > 0.1){
    return 0.1;
  }else if (val_in < -0.1){
    return -0.1;
  }
  return val_in;
}

void TorqueAdjustment::processSample(std::vector<float> &position, std::vector<float> &effort ){
  //return;

  // // tmp testing
  // int JOINT_L_LEG_HPZ   = 4;
  // int JOINT_L_LEG_HPX   = 5;
  // int JOINT_L_LEG_HPY   = 6;
  // int JOINT_L_LEG_KNY   = 7;
  // int JOINT_L_LEG_AKY   = 8;
  // int JOINT_L_LEG_AKX   = 9;
  // int JOINT_R_LEG_HPZ   = 10;
  // int JOINT_R_LEG_HPX   = 11;
  // int JOINT_R_LEG_HPY   = 12;
  // int JOINT_R_LEG_KNY   = 13;
  // int JOINT_R_LEG_AKY   = 14;
  // int JOINT_R_LEG_AKX   = 15;

  // Actual:
  double k_hpz = 7000;
  double k     = 10000;

  // Huge effect - for testing:
  //double k_hpz = 700;
  //double k     = 1000;

  position[Atlas::JOINT_R_LEG_HPZ] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_HPZ] / k_hpz );
  position[Atlas::JOINT_R_LEG_HPX] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_HPX] / k );
  position[Atlas::JOINT_R_LEG_HPY] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_HPY] / k );

  position[Atlas::JOINT_R_LEG_KNY] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_KNY] / k );
  position[Atlas::JOINT_R_LEG_AKY] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_AKY] / k );
  position[Atlas::JOINT_R_LEG_AKX] -= magnitudeLimit( effort[Atlas::JOINT_R_LEG_AKX] / k );

  position[Atlas::JOINT_L_LEG_HPZ] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_HPZ] / k_hpz );
  position[Atlas::JOINT_L_LEG_HPX] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_HPX] / k );
  position[Atlas::JOINT_L_LEG_HPY] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_HPY] / k );

  position[Atlas::JOINT_L_LEG_KNY] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_KNY] / k );
  position[Atlas::JOINT_L_LEG_AKY] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_AKY] / k );
  position[Atlas::JOINT_L_LEG_AKX] -= magnitudeLimit( effort[Atlas::JOINT_L_LEG_AKX] / k );

}

}
