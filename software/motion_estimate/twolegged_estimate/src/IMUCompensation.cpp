
#include <inertial-odometry/IMUCompensation.hpp>

namespace InertialOdometry {

  void IMUCompensation::UpdateGyroBiases(const double biases[3])
  {
    for (int i=0;i<3;i++)
      gyro_biases(i) = biases[i];
    //std::cout << "The gyro biases were updated" << std::endl;
  }

  void IMUCompensation::AccumulateGyroBiases(const Eigen::Vector3d &_delta_biases) {

	  //std::cout << "IMUCompensation::AccumulateGyroBiases -- deltaGyrobias " << delta_biases[0] << ", " << delta_biases[1] << ", " <<  delta_biases[2] << ", " << std::endl;
	  //	  for (int i=0;i<3;i++) {
	  //        gyro_biases(i) = gyro_biases(i) + delta_biases[i];
	  //	  }

	Eigen::Vector3d tmp;

	tmp = gyro_biases + _delta_biases;
	gyro_biases = tmp;
	//
	//	std::cout << "IMUCompensation::AccumulateGyroBiases -- gyro bias now is " << gyro_biases.transpose() << std::endl;
  }
                                                                         
  void IMUCompensation::UpdateAccelBiases(const double biases[3])
  {
	//std::cout << "IMUCompensation::UpdateAccelBiases -- Accel biases being updated: " << biases[0] << ", " << biases[1] << ", " << biases[2] << std::endl;
    for (int i=0;i<3;i++)
      accel_biases(i) = biases[i];
  }

  void IMUCompensation::AccumulateAccelBiases(const Eigen::Vector3d &_delta_biases) {
	  Eigen::Vector3d tmp;

	  tmp = accel_biases + _delta_biases;
	  accel_biases = tmp;

	  //	  for (int i=0;i<3;i++) {
	  //	        accel_biases(i) += delta_biases[i];
	  //	  }
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
  void IMUCompensation::Gyro_Compensation(IMU_dataframe &_imu)
  {    
    _imu.w_b = gyro_errors * (_imu.w_b_measured - gyro_biases);
    //std::cout << "IMUCompensation::Gyro_Compensation -- gyro biases are" << std::endl << gyro_biases << std::endl;
    _imu.gyro_compensated_flag = true;
    return;
  }

  //Compensation subtracts biases FIRST, then scales according to the scale factors given
  void IMUCompensation::Accel_Compensation(IMU_dataframe &_imu)
  {
    _imu.a_b = accel_errors * (_imu.a_b_measured - accel_biases);
    _imu.accel_compensated_flag = true;
    return;
  }

  //Compensation subtracts biases FIRST, then scales according to the scale factors given
   void IMUCompensation::Full_Compensation(IMU_dataframe &_imu)
  {
    //std::cout << "IMUCompensation::Full_Compensation occured" << std::endl;

    Gyro_Compensation(_imu);
    Accel_Compensation(_imu);

    return;
  }

   Eigen::Vector3d IMUCompensation::getGyroBiases() {

	   return gyro_biases;
   }


  Eigen::Vector3d IMUCompensation::getAccelBiases() {

	  return accel_biases;
  }


  IMUCompensation::IMUCompensation()
  {

    //std::cout << "An InertialOdometry::IMUCompensation Object was created." << std::endl;
    
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
