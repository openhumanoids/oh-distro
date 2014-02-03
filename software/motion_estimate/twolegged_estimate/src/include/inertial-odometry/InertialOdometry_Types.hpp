#ifndef __INERTIALODOMETRY_TYPES_H__
#define __INERTIALODOMETRY_TYPES_H__

#include <Eigen/Dense>

namespace InertialOdometry {

  struct IMU_dataframe
  {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	unsigned long long uts;

    Eigen::Vector3d w_b; // rate after bias compensation
	Eigen::Vector3d w_b_measured;
	Eigen::Vector3d dang_b;
    Eigen::Vector3d dang_s;

    Eigen::Vector3d f_l;
    Eigen::Vector3d a_l;
    Eigen::Vector3d a_b;
    Eigen::Vector3d a_b_measured;
	Eigen::Vector3d a_s_measured;

    bool use_dang;
    bool gyro_compensated_flag;
    bool accel_compensated_flag;
    bool gravity_subtracted;
  };

  struct SensorErrorParameters
  {
    double ARW[3];
    double w_bias_instability[3];
    double VRW[3];
    double a_bias_instability[3];
  };
  
  struct DynamicState
  {
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	  IMU_dataframe imu;
	  
	  Eigen::Vector3d P;
	  Eigen::Vector3d V;
	  Eigen::Vector3d f_l;
	  Eigen::Vector3d a_l;
	  Eigen::Vector3d w_l;
	  Eigen::Vector3d ba;
	  Eigen::Vector3d bg;
	  Eigen::Vector3d a_b;
	  Eigen::Vector3d w_b;

	  Eigen::Quaterniond lQb;

	  unsigned long long uts;
  };

  struct CalibrationState
  {
	 EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	 Eigen::Matrix<double, 3, 3> gyro_errors;
     Eigen::Matrix<double, 3, 3> accel_errors;
     Eigen::Vector3d gyro_biases;
     Eigen::Vector3d accel_biases; 
  };
  
  struct INSUpdatePacket {
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	  unsigned long long utime;

	  Eigen::Vector3d dbiasGyro_b;
	  Eigen::Vector3d dbiasAcc_b;

	  Eigen::Vector3d dE_l;
	  Eigen::Vector3d dVel_l;
	  Eigen::Vector3d dPos_l;
  };


  struct InertialOdomOutput
  {
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// Quaternion relative to the initial orientation conditions given
	Eigen::Quaterniond quat;

	Eigen::Vector3d first_pose_rel_acc;
	  
    //Accumulating Velocity Value, relative to provided global frame
    Eigen::Vector3d global_velocity;
    Eigen::Vector3d first_pose_rel_vel;
    
    //Accumulating Position Value
    Eigen::Vector3d global_position;
    Eigen::Vector3d first_pose_rel_pos;
    
    
    //Initial Velocity state, relative to user supplied global frame
    Eigen::Vector3d global_relative_initial_vel;//Assume zeros for now
    //Initial position state, relative to user supplied global frame
    Eigen::Vector3d global_relative_init_pos;//Assume zeros for now
    
    unsigned long long q_uts;
	unsigned long long vp_uts;
	//unsigned long long T_interm;//??
  };


 // enum is put in during initial writing, but is not implemented yet.
  enum OrientationPropagationApproach {
    FastNumericalMultirate = 0, // Quaternion, Savage 1998 implementation - Second order solution to Bortz Equation 1971 (Linear coning correction)
    ClosedFormAnalytical        // Analytical solution, Sine/Cosine restructured Taylor expansions
  };


  class Parameters
  {
    public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       std::string IMU_name;
       SensorErrorParameters imu_error_specs;
       enum OrientationPropagationApproach orientation_propagation_approach;
       double Ts;
       Eigen::Matrix<double, 6, 6> Qimu;
       Eigen::Vector3d gravity;
       
       // Earth rate is not used yet, as it is more sensitive to change in the platform position - this is a
       //TODO
       Eigen::Vector3d w_ie;

       Parameters() {
    	   Ts = 0;
    	   orientation_propagation_approach = FastNumericalMultirate;
    	   Qimu.setZero(6,6);
    	   Qimu.setIdentity(6,6);
    	   
    	   gravity << 0, 0, -9.8;
       }
  };


}

#endif
