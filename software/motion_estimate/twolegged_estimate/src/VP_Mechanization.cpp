#include "VP_Mechanization.hpp"

namespace InertialOdometry {

  VP_Mechanization::VP_Mechanization()
  {
	  a2v.setSize(3);
	  v2p.setSize(3);

	  last_uts = 0;
  }

  void VP_Mechanization::SubtractGravity(IMU_dataframe* _imu) {

	  _imu->force_ = _imu->accel_ - std_param.gravity;
	  _imu->gravity_subtracted = true;

  }


  void VP_Mechanization::PropagateTranslation(IMU_dataframe* _imu)
  {
	  SubtractGravity(_imu);

	  Eigen::VectorXd test;

	  test = a2v.integrate(_imu->uts,_imu->force_);

	  state.first_pose_rel_vel = test;

	  //std::cout << "state: " << _imu->force_.transpose() << " | " << test.transpose() << std::endl;

	  state.first_pose_rel_pos = v2p.integrate(_imu->uts, state.first_pose_rel_vel);

	  last_uts = _imu->uts;

	  return;
  }

   void VP_Mechanization::updateOutput(InertialOdomOutput* _out) {

	  _out->vp_uts = last_uts;

	  _out->first_pose_rel_pos = state.first_pose_rel_pos;
	  _out->first_pose_rel_vel = state.first_pose_rel_vel;

	  return;
  }


}
