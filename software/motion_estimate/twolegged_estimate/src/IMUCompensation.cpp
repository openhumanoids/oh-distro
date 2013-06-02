
#include "IMUCompensation.hpp"


namespace InertialOdometry {


  void IMUCompensation::UpdateGyroBiases(const double biases[3])
  {
    for (int i=0;i<3;i++)
      gyro_biases(i) = biases[i];
    //std::cout << "The gyro biases were updated" << std::endl;
  }
                                                                         
  void IMUCompensation::UpdateAccelBiases(const double biases[3])
  {
	  std::cout << "Accel biases being updated: " << biases[0] << ", " << biases[1] << ", " << biases[2] << std::endl;
    for (int i=0;i<3;i++)
      accel_biases(i) = biases[i];
  }

  void IMUCompensation::AccumulateAccelBiases(const double delta_biases[3]) {
	  for (int i=0;i<3;i++) {
	        accel_biases(i) += delta_biases[i];
	  }
  }

  void IMUCompensation::UpdateGyroScaleFactor(const double sf[3])
  {
    for (int i=0;i<3;i++)
      gyro_errors(i,i) = sf[i];

  }
           
  void IMUCompensation::UpdateAccelScaleFactor(const double sf[3])
  {
    for (int i=0;i<3;i++)
      accel_errors(i,i) = sf[i];
  }
             
  void IMUCompensation::SetGyroMisalignments(const double MA[3])
  {
      gyro_errors(0,1) = MA[0];
      gyro_errors(1,0) = MA[0];

      gyro_errors(0,2) = MA[1];
      gyro_errors(2,0) = MA[1];

      gyro_errors(1,2) = MA[2];
      gyro_errors(2,1) = MA[2];
  }
             
  void IMUCompensation::SetAccelMisalignments(const double MA[3])
  {
      accel_errors(0,1) = MA[0];
      accel_errors(1,0) = MA[0];

      accel_errors(0,2) = MA[1];
      accel_errors(2,0) = MA[1];

      accel_errors(1,2) = MA[2];
      accel_errors(2,1) = MA[2];
    
  }

  //Compensation subtracts biases FIRST, then scales according to the scale factors given
  void IMUCompensation::Gyro_Compensation(IMU_dataframe *_imu_pre)
  {    
    _imu_pre->gyro_ = gyro_errors * (_imu_pre->gyro_ - gyro_biases);
    _imu_pre->gyro_compensated_flag = true;
    return;
  }

  //Compensation subtracts biases FIRST, then scales according to the scale factors given
  void IMUCompensation::Accel_Compensation(IMU_dataframe *_imu_pre)
  {
	  //Eigen::Vector3d pre;
	  //pre = _imu_pre->acc_b;
    _imu_pre->acc_comp = accel_errors * (_imu_pre->acc_b - accel_biases);

    //std::cout << "Compensating Accel: " << pre.transpose() << " | " << _imu_pre->accel_.transpose() << " | " << accel_errors <<  std::endl;

    _imu_pre->accel_compensated_flag = true;
    return;
  }

  //Compensation subtracts biases FIRST, then scales according to the scale factors given
   void IMUCompensation::Full_Compensation(IMU_dataframe *_imu_pre)
  {
    //std::cout << "IMUCompensation::Full_Compensation occured" << std::endl;

    Gyro_Compensation(_imu_pre);
    Accel_Compensation(_imu_pre);

    return;
  }

   Eigen::Vector3d IMUCompensation::get_accel_biases() {
	   return accel_biases;
   }


  IMUCompensation::IMUCompensation()
  {

    std::cout << "An InertialOdometry::IMUCompensation Object was created." << std::endl;
    
    double initvector[3];

    for (int i = 0;i<3;i++)
      initvector[i] = 0.0;

    UpdateGyroBiases(initvector);
    UpdateAccelBiases(initvector);
    SetGyroMisalignments(initvector);
    SetAccelMisalignments(initvector);

    for (int i = 0;i<3;i++)
      initvector[i] = 1.0;

    UpdateGyroScaleFactor(initvector);
    UpdateAccelScaleFactor(initvector);
  }

  void IMUCompensation::cout_current_values()
  {
	  std::cout << std::endl << "====================================================" << std::endl;
	  std::cout << "IMUCompensation object" << std::endl;
	  std::cout << "gyro_biases: " << std::endl << gyro_biases << std::endl;

	  std::cout << "accel_biases: " << std::endl << accel_biases << std::endl;

	  std::cout << "gyro_scale_errors: " << std::endl << gyro_errors << std::endl;

	  std::cout << "accel_scale_errors: " << std::endl << accel_errors << std::endl;
	  std::cout << "====================================================" << std::endl;
  }

}
