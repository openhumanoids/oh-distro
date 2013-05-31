#include "Odometry.hpp"
#include <cmath>

namespace InertialOdometry {

  Odometry::Odometry()
    {
      std::cout << "An InertialOdometry::Odometry object was created, using internal state structure." << std::endl;

    }

  InertialOdomOutput Odometry::PropagatePrediction_wo_IMUCompensation(IMU_dataframe* _imu, const Eigen::Quaterniond &orient)
  {
	InertialOdomOutput ret; // populated at end of this member

    orc.updateOrientation(_imu->uts,orient);

    _imu->accel_ = orc.ResolveBodyToRef( _imu->acc_b);//??
    avp.PropagateTranslation(_imu);

    // update output structure
    orc.updateOutput(&ret);
    avp.updateOutput(&ret);

    // return the data
    return ret;
  }
  
  DynamicState Odometry::PropagatePrediction(IMU_dataframe *_imu, const Eigen::Quaterniond &orient)
  {
	InertialOdomOutput out;

    imu_compensator.Full_Compensation(_imu);

    out = PropagatePrediction_wo_IMUCompensation(_imu,orient);

    //std::cout << "imu: " << _imu->force_.transpose() << " | " << out.first_pose_rel_pos.transpose() << std::endl;

    DynamicState ret;
    ret.P = out.first_pose_rel_pos;
    ret.V = out.first_pose_rel_vel;
    ret.E.setZero();
    ret.b_a.setZero();
    ret.b_g.setZero();
    ret.q = out.quat;
    
    return ret;
  }
  
  // This should be moved to the QuaternionLib library
  Eigen::Matrix<double, 3, 3> Odometry::Expmap(const Eigen::Vector3d &w)
  {
	  Eigen::Matrix<double, 3, 3> R;
	  Eigen::Matrix<double, 3, 3> temp;
	  
#ifdef USE_TRIGNOMETRIC_EXMAP
	  R.setIdentity(3,3);
	  double mag = w.norm();
	  Eigen::Vector3d direction = 1/mag * w;
	  skew(direction,temp);
	  R += sin(mag) * temp + (1- cos(mag))*(temp*temp);
	  
#endif
	  
	  /*
	  mag = norm(w);
      direction = w./mag;
      R = eye(3) + sin(mag)*skew(direction) + (1-cos(mag))*(skew(direction)^2);
      */
	  
	  return R;
  }

}

