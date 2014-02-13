#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include <iostream>
#include <atomic>

#include <boost/thread/mutex.hpp>

#include <inertial-odometry/InertialOdometry_Types.hpp>
#include <inertial-odometry/OrientationComputer.hpp>
#include <inertial-odometry/VP_Mechanization.hpp>
#include <inertial-odometry/IMUCompensation.hpp>

#define USE_TRIGNOMETRIC_EXMAP

namespace InertialOdometry {

  typedef std::atomic<bool> Lock;

  class Odometry {
    private:
	  double Ts_imu;

	  OrientationComputer orc;
	  VP_Mechanization avp;
	  Parameters param;
	  Eigen::Isometry3d IMU_to_body;

	  Lock mINSUpdateAtomic;

	  void UpdateStructState(InertialOdomOutput &output_data);// ??

	  InertialOdomOutput PropagatePrediction_wo_IMUCompensation(IMU_dataframe &_imu);
	  InertialOdomOutput PropagatePrediction_wo_IMUCompensation(IMU_dataframe &_imu, const Eigen::Quaterniond &orient);

	  //Eigen::Matrix<double, 3, 3> Expmap(Eigen::Vector3d const &w);
	  
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      IMUCompensation imu_compensator;
      mutable boost::mutex mutex_; // Protect INS propagation from EKF update to INS state -- slaved to kinematics message


      // Constructor with standard parameters
      Odometry(double imu_sampletime);
      // Propagate the internal state registers with new IMU data
      DynamicState PropagatePrediction(IMU_dataframe &_imu, const Eigen::Quaterniond &orient);
      DynamicState PropagatePrediction(IMU_dataframe &_imu);

      //DynamicState getDynamicState();
      Eigen::Quaterniond lQb();

      //unsigned long long get_utime();
      void setPositionState(const Eigen::Vector3d &P_set);
      void setVelocityState(const Eigen::Vector3d &V_set);

      Eigen::Vector3d ResolveBodyToRef(const Eigen::Vector3d &_va);

      void incorporateERRUpdate(const InertialOdometry::INSUpdatePacket &updateData);

	  void enterCritical();
	  void exitCritical();
	  void setIMU2Body(const Eigen::Isometry3d &imu2body);
	  void sensedImuToBodyTransform(IMU_dataframe &_imu);
	  void setInitPitchRoll(const std::vector<Eigen::Vector3d> &initacceldata);
	  const Eigen::Isometry3d& getIMU2Body();
  };

}

#endif
