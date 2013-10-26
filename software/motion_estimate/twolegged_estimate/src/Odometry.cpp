#include <inertial-odometry/Odometry.hpp>
#include <cmath>

namespace InertialOdometry {

  Odometry::Odometry()
    {
      std::cout << "An InertialOdometry::Odometry object was created, using internal state structure." << std::endl;

    }

  InertialOdomOutput Odometry::PropagatePrediction_wo_IMUCompensation(IMU_dataframe* _imu, const Eigen::Quaterniond &orient)
  {
	InertialOdomOutput ret; // populated at end of this member

	// We are going to now compute he quaternion ourselves
    //orc.updateOrientation(_imu->uts,orient);

    orc.updateOrientation(_imu->uts,_imu->gyr_b);

    _imu->accel_ = orc.ResolveBodyToRef( _imu->acc_comp);//??
    ret.first_pose_rel_acc = _imu->accel_;
    
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

    //    DynamicState ret;
    state.imu = *_imu;
    state.uts = _imu->uts;
    state.a_l = _imu->accel_;
    state.f_l = _imu->force_;
    state.w_l = C_bw()*_imu->gyro_; // TODO -- this may be a duplicated computation. Ensure this is done in only one place
    state.P = out.first_pose_rel_pos;
    state.V = out.first_pose_rel_vel;
    state.E.setZero();
    state.b_a.setZero();
    state.b_g.setZero();
    state.q = out.quat;
    
    return state;
  }
  
  DynamicState Odometry::getDynamicState() {
	  return state;
  }
  
  // This should be moved to the QuaternionLib library
  // TODO -- should be updated with trigonometric and near zero power expansion cases.
  Eigen::Matrix3d Odometry::Expmap(const Eigen::Vector3d &w)
  {
	  Eigen::Matrix3d R;
	  Eigen::Matrix3d temp;
	  
#ifdef USE_TRIGNOMETRIC_EXMAP
	  R.setIdentity();
	  double mag = w.norm();
	  Eigen::Vector3d direction = 1/mag * w;
	  skew(direction,temp);
	  R += sin(mag) * temp + (1- cos(mag))*(temp*temp);
	  //  TODO -- confirm that we do not have to divide by mag -- if then do the numerical fix to second order Taylor

#endif
	  
	  /*
	  mag = norm(w);
      direction = w./mag;
      R = eye(3) + sin(mag)*skew(direction) + (1-cos(mag))*(skew(direction)^2);
      */
	  
	  return R;
  }

	void Odometry::setPositionState(const Eigen::Vector3d &P_set) {

		avp.setPosStates(P_set);

		return;
	}

	void Odometry::setVelocityState(const Eigen::Vector3d &V_set) {

		avp.setVelStates(V_set);

		return;
	}

	Eigen::Matrix3d Odometry::C_bw() {
		return q2C(orc.q()); //Not sure about the transpose -- must test
	}

}
