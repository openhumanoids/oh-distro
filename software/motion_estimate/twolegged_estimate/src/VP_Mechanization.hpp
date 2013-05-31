#ifndef __VP_MECHANIZATION_H__
#define __VP_MECHANIZATION_H__

#include <iostream>
#include <Eigen/Dense>
#include "InertialOdometry_Types.hpp"
#include "SignalTap.hpp"

namespace InertialOdometry {

  class VP_Mechanization {
    private:
      InertialOdomOutput state;
      TrapezoidalInt a2v;
      TrapezoidalInt v2p;

      Parameters std_param;

      unsigned long long last_uts; // ?? micro_seconds?

      void SubtractGravity(IMU_dataframe* _imu);

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      VP_Mechanization();

      void PropagateTranslation(IMU_dataframe* _imu);

      // This function
      void updateOutput(InertialOdomOutput *_out);

      int Reset_all_states();

  };

}

#endif
