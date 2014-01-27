#include <inertial-odometry/VP_Mechanization.hpp>

namespace InertialOdometry {

  VP_Mechanization::VP_Mechanization()
  {
	  //a2v.setSize(3);
	  //v2p.setSize(3);

	  last_uts = 0;
  }


  void VP_Mechanization::PropagateTranslation(IMU_dataframe &_imu)
  {
	  
	  _imu.f_l = _imu.a_l - std_param.gravity;
	  _imu.gravity_subtracted = true;

	  //std::cout << "VP_Mechanization::PropagateTranslation -- before subtracting gravity " << _imu.a_l.transpose() << std::endl;
	  //std::cout << "VP_Mechanization::PropagateTranslation -- f_l after subtracting gravity " << _imu.f_l.transpose() << std::endl;

	  double dt;
	  dt = (_imu.uts - last_uts)*1E-6;

	  // TODO -- Improve to better numerical integration
	  state.first_pose_rel_pos = state.first_pose_rel_pos + dt*state.first_pose_rel_vel; // Position first
	  state.first_pose_rel_vel = state.first_pose_rel_vel + dt*_imu.f_l; // Velocity second

	  //	  state.first_pose_rel_pos = v2p.integrate(_imu.uts, state.first_pose_rel_vel); // Position first
	  //	  state.first_pose_rel_vel = a2v.integrate(_imu.uts,_imu.f_l); // Velocity second


	  last_uts = _imu.uts;

	  return;
  }

   void VP_Mechanization::updateOutput(InertialOdomOutput &_out) {

	  _out.vp_uts = last_uts;

	  _out.first_pose_rel_pos = state.first_pose_rel_pos;
	  _out.first_pose_rel_vel = state.first_pose_rel_vel;

	  return;
  }

   void VP_Mechanization::setPosStates(const Eigen::Vector3d &P_set) {

	   //std::cout << "VP_Mechanization::setPosStates -- previous positions " << v2p.getVal().transpose() << std::endl;
	   //	   v2p.setStateTo(P_set);
	   state.first_pose_rel_pos = P_set;
	   //std::cout << "VP_Mechanization::setPosStates -- P_set " << P_set.transpose() << std::endl;
	   //std::cout << "VP_Mechanization::setPosStates -- new positions " << v2p.getVal().transpose() << std::endl;

	   return;
   }

   Eigen::Vector3d VP_Mechanization::getVelStates() {
	   return state.first_pose_rel_vel;
   }

   Eigen::Vector3d VP_Mechanization::getPosStates() {
	   return state.first_pose_rel_pos;
   }


   void VP_Mechanization::setVelStates(const Eigen::Vector3d &V_set) {

	   //   	   a2v.setStateTo(V_set);
   	   state.first_pose_rel_vel = V_set;

   	   return;
   }

}
