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

    //std::cout << "Odometry::PropagatePrediction_wo_IMUCompensation -- a_b " << std::endl << _imu.a_b.transpose() << std::endl;

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

    imu_compensator.Full_Compensation(_imu);
    sensedImuToBodyTransform(_imu);
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

    //std::cout << "Odometry::PropagatePrediction -- before lQb " << orc.q().w() << ", " << orc.q().x() << ", " << orc.q().y() << ", " << orc.q().z() << ", " << std::endl;

    //sensedImuToBodyTransform(_imu);
    if (&_imu.use_dang) {
    	_imu.w_b_measured = 1/Ts_imu * _imu.dang_b;
    }
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


//    std::cout << "Odometry::PropagatePrediction -- current lQb " << orc.q().w() << ", " << orc.q().x() << ", " << orc.q().y() << ", " << orc.q().z() << ", " << std::endl;
//    std::cout << "Odometry::PropagatePrediction -- imu update with utime " << _imu.uts << std::endl;
//    std::cout << "Odometry::PropagatePrediction -- a_s_measured " << _imu.a_s_measured.transpose() << std::endl;
//    std::cout << "Odometry::PropagatePrediction -- a_b_measured " << _imu.a_b_measured.transpose() << std::endl;
//    std::cout << "Odometry::PropagatePrediction -- w_b_measured " << _imu.w_b_measured.transpose() << std::endl;
//    std::cout << "Odometry::PropagatePrediction -- a_b " << _imu.a_b.transpose() << std::endl;
//    std::cout << "Odometry::PropagatePrediction -- a_l " << _imu.a_l.transpose() << std::endl;
//    std::cout << "Odometry::PropagatePrediction -- w_b " << _imu.w_b.transpose() << std::endl;

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
	//std::cout << "Odometry::sensedImuToBodyTransform -- IMU_to_body.linear()" << IMU_to_body.linear() << std::endl;

	_imu.a_b_measured = IMU_to_body.linear() * _imu.a_s_measured; // TODO -- Remove radial acceleration component from this signal.
	_imu.dang_b = IMU_to_body.linear() * _imu.dang_s;

	//_imu.a_b_measured = _imu.a_s_measured;
	//_imu.dang_b = _imu.dang_s;
  }

  void Odometry::setInitPitchRoll(const std::vector<Eigen::Vector3d> &initacceldata) {

	Eigen::Vector3d a_b;
	a_b.setZero();

	int length;
	length = initacceldata.size();

	for (int k=0;k<length;k++) {
	  //std::cout << "Odometry::setInitPitchRoll -- initacceldata[k] = " << initacceldata[k].transpose() << std::endl;
      a_b = a_b + initacceldata[k]/(length+0.) ;
	}
	std::cout << "Odometry::setInitPitchRoll -- a_b = " << a_b.transpose() << std::endl;

	double roll, pitch;

	roll = atan2(-a_b(1),-a_b(2));
	pitch = atan2( a_b(0), sqrt(a_b(1)*a_b(1) + a_b(2)*a_b(2) ) );

	std::cout<< "Odometry::setInitPitchRoll -- roll, pitch " << roll << ", " << pitch << std::endl;

	Eigen::Vector3d E(roll,pitch,0);
	Eigen::Matrix3d bRn, tmp;
	bRn = e2C(E);

	std::cout<< "Odometry::setInitPitchRoll -- bRn " << std::endl << bRn << std::endl;

	//	Eigen::Quaterniond q;
	//	q.w() = 0.;
	//	q.x() = 1.;
	//	q.y() = 0.;
	//	q.z() = 0.;
	//	tmp = q2C(q)*bRn;
	//	bRn = tmp;

	orc.updateOrientation(0, C2q(bRn.transpose()));
	std::cout << "Odometry::setInitPitchRoll -- Initial lQb has been set to " << orc.q().w() << ", " << orc.q().x() << ", " << orc.q().y() << ", " << orc.q().z() << std::endl;


	//    Eigen::Vector3d v1,v2,v3,v3m,tmp;
	//
	//    tmp = a_b/a_b.norm();
	//    a_b = tmp;
	//
	//    v1 << 0,0,1;
	//    v2 << 1,0,0;
	//    v3 = v1.cross(v2);
	//    v3m = a_b.cross(v2);
	//
	//    Eigen::Matrix3d A, B, nRb;
	//
	//    A(0,0) = v1(0);
	//    A(1,0) = v1(1);
	//    A(2,0) = v1(2);
	//
	//    A(0,1) = v2(0);
	//	A(1,1) = v2(1);
	//	A(2,1) = v2(2);
	//
	//	A(0,2) = v3(0);
	//	A(1,2) = v3(1);
	//	A(2,2) = v3(2);
	//
	//	B(0,0) = a_b(0);
	//	B(1,0) = a_b(1);
	//	B(2,0) = a_b(2);
	//
	//	B(0,1) = v2(0);
	//	B(1,1) = v2(1);
	//	B(2,1) = v2(2);
	//
	//	B(0,2) = v3m(0);
	//	B(1,2) = v3m(1);
	//	B(2,2) = v3m(2);
	//
	//	nRb = B * A.inverse();



	//	ab = mean_acc(:)/norm(mean_acc);
	//
	//	v1 = [0;0;1];
	//	v2 = [1;0;0];
	//	v3 = cross(v1,v2);
	//	v3m = cross(ab,v2);
	//
	//	if (true)
	//	    A = [v1,v2,v3];
	//	    B = [ab(:)/norm(ab),v2(:)/norm(v2),v3m(:)/norm(v3m)];
	//	    R_nav_to_body = B*inv(A); % synthetic heading alignment is not working
	//	else
	//	    roll = atan2(-ab(2),-ab(3));
	//		pitch = atan2(ab(1), norm(ab(2:3)));
	//		R_body_to_nav = q2R(e2q([roll;pitch;0]));
	//
	//		% purposefully flip about x axis -- not part of the normal procedure
	//		% kept for future refenence
	//		R_body_to_nav = q2R([0;1;0;0])*R_body_to_nav
	//		R_nav_to_body = R_body_to_nav';
	//		q_nb = R2q(R_nav_to_body);
	//	end
	//
	//	R_nav_to_body'*mean_acc
	//
	//	init_lQb = R2q(R_nav_to_body);
  }

  const Eigen::Isometry3d& Odometry::getIMU2Body() {
	return IMU_to_body;
  }

  void Odometry::setHeading(const double &psi) {
	orc.setYaw(psi);
  }

}


