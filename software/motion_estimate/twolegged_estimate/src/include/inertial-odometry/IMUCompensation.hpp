#ifndef __IMUCOMPENSATION_H__
#define __IMUCOMPENSATION_H__

#include <iostream>
#include <Eigen/Dense>
#include <inertial-odometry/InertialOdometry_Types.hpp>

namespace InertialOdometry
{
  class IMUCompensation {
    private:
    	
    	// TODO
    	// This should maybe be a CalibrationState struct from InertialOdometry_Types.hpp
       Eigen::Matrix<double, 3, 3> gyro_errors;
       Eigen::Matrix<double, 3, 3> accel_errors;
       Eigen::Vector3d gyro_biases;
       Eigen::Vector3d accel_biases;

    public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW

       IMUCompensation();
       void UpdateGyroBiases(const double biases[3]);
       void AccumulateGyroBiases(const Eigen::Vector3d &_delta_biases);
       void UpdateAccelBiases(const double biases[3]);
       void AccumulateAccelBiases(const Eigen::Vector3d &_delta_biases);
       void UpdateGyroScaleFactor(const double sf[3]);
       void UpdateAccelScaleFactor(const double sf[3]);
       void SetGyroMisalignments(const double MA[3]);
       void SetAccelMisalignments(const double MA[3]);
       void Full_Compensation(IMU_dataframe &_imu);
       void Gyro_Compensation(IMU_dataframe &_imu);
       void Accel_Compensation(IMU_dataframe &_imu);

       Eigen::Vector3d getGyroBiases();
       Eigen::Vector3d getAccelBiases();
       Eigen::Vector3d get_accel_biases();

       void cout_current_values();
  };

}

#endif

