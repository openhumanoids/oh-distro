#include <inertial-odometry/Odometry.hpp>
#include <cmath>

namespace InertialOdometry {

  Odometry::Odometry(double imu_sampletime)
  {
	Ts_imu = imu_sampletime;
	IMU_to_body.setIdentity();
  }

  InertialOdomOutput Odometry::PropagatePrediction_wo_IMUCompensation(IMU_dataframe &_imu)
  {
	InertialOdomOutput ret; // populated at end of this member

	// We are going to now compute he quaternion ourselves
    //orc.updateOrientation(_imu->uts,orient);

	// We use update WithRate, since we would like to cater for packet loss. Ideally though, we should use delta_angles directly
    orc.updateOrientationWithRate(_imu.uts, _imu.w_b);

    std::cout << "Odometry::PropagatePrediction_wo_IMUCompensation -- a_b " << std::endl << _imu.a_b.transpose() << std::endl;

    _imu.a_l = orc.ResolveBodyToRef( _imu.a_b);
    ret.first_pose_rel_acc = _imu.a_l;
    
    avp.PropagateTranslation(_imu);

    // update output structure
    //orc.updateOutput(ret);
    ret.quat = orc.q();
    avp.updateOutput(ret);

    // return the data
    return ret;
  }
  
  InertialOdomOutput Odometry::PropagatePrediction_wo_IMUCompensation(IMU_dataframe &_imu, const Eigen::Quaterniond &orient)
  {
  	  InertialOdomOutput ret; // populated at end of this member

  	sensedImuToBodyTransform(_imu);

  	  // We are going to now compute he quaternion ourselves
      orc.updateOrientation(_imu.uts,orient);

      _imu.a_l = orc.ResolveBodyToRef( _imu.a_b);
      ret.first_pose_rel_acc = _imu.a_l;

      avp.PropagateTranslation(_imu);

      // update output structure
      //orc.updateOutput(ret);
      ret.quat = orc.q();
      avp.updateOutput(ret);

      // return the data
      return ret;
  }

  DynamicState Odometry::PropagatePrediction(IMU_dataframe &_imu, const Eigen::Quaterniond &orient) {
	InertialOdomOutput out;

	sensedImuToBodyTransform(_imu);
    imu_compensator.Full_Compensation(_imu);
    out = PropagatePrediction_wo_IMUCompensation(_imu,orient);

    DynamicState state;
    state.imu = _imu;
    state.uts = _imu.uts;
    state.a_l = _imu.a_l;
    state.f_l = _imu.f_l;
    state.w_l = orc.ResolveBodyToRef(_imu.w_b); // TODO -- this may be a duplicated computation. Ensure this is done in only one place
    state.P = out.first_pose_rel_pos;
    state.V = out.first_pose_rel_vel;
    state.ba.setZero();
    state.bg.setZero();
    state.a_b = _imu.a_b;
    state.w_b = _imu.w_b;
    state.lQb = out.quat;
    
    return state;
  }
  
  IMU_dataframe PlatformSpecificIMUTransform(IMU_dataframe &_imu) {
	// Align IMU to body

	// Compute rotation rate and accelerations from delta quantities
	//_imu.w_b_measured = - 1/Ts_imu * Eigen::Vector3d(_imu.delta_rotation[0], _imu.delta_rotation[1], _imu.delta_rotation[2]);
	return _imu;
  }

  DynamicState Odometry::PropagatePrediction(IMU_dataframe &_imu)
  {
    InertialOdomOutput out;

    sensedImuToBodyTransform(_imu);

    if (&_imu.use_dang) {
    	_imu.w_b_measured = 1/Ts_imu * _imu.dang_b;
    }
    std::cout << "Odometry::PropagatePrediction -- imu update with utime " << _imu.uts << std::endl;
    std::cout << "Odometry::PropagatePrediction -- a_b_measured " << _imu.a_b_measured.transpose() << std::endl;
    std::cout << "Odometry::PropagatePrediction -- w_b_measured " << _imu.w_b_measured.transpose() << std::endl;
    std::cout << "Odometry::PropagatePrediction -- a_b " << _imu.a_b.transpose() << std::endl;
    std::cout << "Odometry::PropagatePrediction -- w_b " << _imu.w_b.transpose() << std::endl;

    imu_compensator.Full_Compensation(_imu);

    out = PropagatePrediction_wo_IMUCompensation(_imu);

    DynamicState state;
    state.imu = _imu;
    state.uts = _imu.uts;
    state.a_l = _imu.a_l;
    state.f_l = _imu.f_l;
    state.w_l = orc.ResolveBodyToRef(_imu.w_b); // TODO -- this may be a duplicated computation. Ensure this is done in only one place
    state.P = out.first_pose_rel_pos;
    state.V = out.first_pose_rel_vel;
    state.ba = imu_compensator.getAccelBiases();
    state.bg = imu_compensator.getGyroBiases();
    state.a_b = _imu.a_b;
    state.w_b = _imu.w_b;
    state.lQb = out.quat;

    return state;
  }

  Eigen::Quaterniond Odometry::lQb() {
	  return orc.q();
  }
  

  void Odometry::incorporateERRUpdate(const InertialOdometry::INSUpdatePacket &updateData) {

	//	  std::cout << "Odometry::incorporateERRUpdate -- received and INSUpdatePacket." << std::endl;
	//	  std::cout << "Odometry::incorporateERRUpdate -- utime " << std::endl << updateData.utime << std::endl;
	//	  std::cout << "Odometry::incorporateERRUpdate -- dbg_b " << std::endl << updateData.dbiasGyro_b << std::endl;
	//	  std::cout << "Odometry::incorporateERRUpdate -- dba_b " << std::endl << updateData.dbiasAcc_b << std::endl;
	//	  std::cout << "Odometry::incorporateERRUpdate -- dE_l " << std::endl << updateData.dE_l << std::endl;
	//	  std::cout << "Odometry::incorporateERRUpdate -- dV_l " << std::endl << updateData.dVel_l << std::endl;
	//	  std::cout << "Odometry::incorporateERRUpdate -- dP_l " << std::endl << updateData.dPos_l << std::endl;

	Eigen::Vector3d tmp;

	enterCritical();
	imu_compensator.AccumulateGyroBiases(updateData.dbiasGyro_b);
	imu_compensator.AccumulateAccelBiases(updateData.dbiasAcc_b);
	orc.rotateOrientationUpdate(updateData.dE_l);
	tmp = avp.getVelStates();
	avp.setVelStates(tmp - updateData.dVel_l);
	tmp = avp.getPosStates();
	avp.setPosStates(tmp - updateData.dPos_l);
	exitCritical();
  }

  void Odometry::setPositionState(const Eigen::Vector3d &P_set) {
	avp.setPosStates(P_set);
	return;
  }

  void Odometry::setVelocityState(const Eigen::Vector3d &V_set) {
	avp.setVelStates(V_set);
	return;
  }


  Eigen::Vector3d Odometry::ResolveBodyToRef(const Eigen::Vector3d &_va) {
	return orc.ResolveBodyToRef(_va);
  }

  void Odometry::enterCritical() {
	bool unlocked = false;
	while (!mINSUpdateAtomic.compare_exchange_weak(unlocked, true));
  }

  void Odometry::exitCritical() {
	mINSUpdateAtomic.store(false);
  }

  void Odometry::setIMU2Body(const Eigen::Isometry3d &imu2body) {
	IMU_to_body = imu2body;
	std::cout << "Odometry::setIMU2Body -- IMU_to_body: " << IMU_to_body.linear() << std::endl << IMU_to_body.translation() << std::endl;
  }

  void Odometry::sensedImuToBodyTransform(IMU_dataframe &_imu) {
	//_imu.a_b_measured = IMU_to_body.linear() * _imu.a_s_measured;
	//_imu.dang_b = IMU_to_body.linear() * _imu.dang_s;

	_imu.a_b_measured = _imu.a_s_measured;
	_imu.dang_b = _imu.dang_s;
  }

}

// Been moved to QuaternionLib
//  Eigen::Matrix3d Odometry::Expmap(const Eigen::Vector3d &w)
//  {
//	  Eigen::Matrix3d R;
//	  Eigen::Matrix3d temp;
//
//#ifdef USE_TRIGNOMETRIC_EXMAP
//	  R.setIdentity();
//	  double mag = w.norm();
//	  Eigen::Vector3d direction = 1/mag * w;
//	  skew(direction,temp);
//	  R += sin(mag) * temp + (1- cos(mag))*(temp*temp);
//	  //  TODO -- confirm that we do not have to divide by mag -- if then do the numerical fix to second order Taylor
//
//#endif
//
//	  /*
//	  mag = norm(w);
//      direction = w./mag;
//      R = eye(3) + sin(mag)*skew(direction) + (1-cos(mag))*(skew(direction)^2);
//      */
//
//	  return R;
//  }
