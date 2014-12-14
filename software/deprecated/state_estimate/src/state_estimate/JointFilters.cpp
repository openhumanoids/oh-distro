
#include "JointFilters.h"

StateEstimate::JointFilters::JointFilters() {
	
}



void StateEstimate::JointFilters::setSize(const int &num_joints) {
  
  // Presently we can only set the size of the vectors once at startup
	
  mNumJoints = num_joints;
  mJointModel.assign(mNumJoints, KalmanFilter_Models::Joint_Model());
  //kfJoints.resize(mNumJoints);
  //mJointEst.resize(mNumJoints);
  
  std::cout << "StateEstimate::JointFilters::setSize -- size of mJointModel " << mJointModel.size() << std::endl;
  std::cout << "StateEstimate::JointFilters::setSize -- size of kfJoints " << kfJoints.size() << std::endl;
  
  for ( int i = 0;i<mNumJoints;i++) {
	//mJointModel.push_back(KalmanFilter_Models::Joint_Model());
	kfJoints.push_back(KalmanFilter(mJointModel[i]));
	mJointEst.push_back(KalmanFilter_Types::State());
	
	// And initialize the filters
	kfJoints[i].Initialize();
  }
  std::cout << "StateEstimate::JointFilters::setSize -- size of kfJoints " << kfJoints.size() << std::endl;
  std::cout << "StateEstimate::JointFilters::setSize -- " << num_joints << " joint KFs created" << std::endl;
  
}


void StateEstimate::JointFilters::updateStates(ulong utime, const std::vector<float> &pos, std::vector<float> &vel, const int &num_joints) {
  
  if (mNumJoints < num_joints) {
    std::cerr << "JointFilters::updateStates -- You asking to filter more joint velocities than you initialized filters for." << std::endl;
  }

  Eigen::VectorXd parameters;
  parameters.setZero();
  Eigen::VectorXd joint_position;

  parameters.resize(1);
  joint_position.resize(1);
  
  
  for ( int i = 0;i<mNumJoints;i++) {
	joint_position(0) = pos[i];
	//std::cout << "StateEstimate::JointFilters::updateStates -- stepping filter with " << _pos[i] << std::endl;
	kfJoints[i].step(utime, parameters, joint_position);
	//std::cout << "StateEstimate::JointFilters::updateStates -- filter updated" << std::endl;
	mJointEst[i] = kfJoints[i].getState();
	vel[i] = mJointEst[i].X(1);// Insert the velocity estimate in the return vector
  }
}
