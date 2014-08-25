#ifndef __TORQUE_ADJUSTMENT_HPP__
#define __TORQUE_ADJUSTMENT_HPP__

#include <iostream>
#include <inttypes.h>
#include <vector>
#include "atlas/AtlasControlTypes.h"
#include "atlas/AtlasJointNames.h"

namespace EstimateTools {

class TorqueAdjustment{
  public:
    TorqueAdjustment();

    ~TorqueAdjustment(){
    }

    void processSample(std::vector<float> &position, std::vector<float> &effort );

  private:


};


}

#endif

