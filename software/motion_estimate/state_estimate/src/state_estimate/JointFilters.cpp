
#include "JointFilters.h"

StateEstimate::JointFilters::JointFilters() {
	
}



void StateEstimate::JointFilters::setSize(const int &num_joints) {
  
  // Presently we can only set the size of the vectors once at startup
	
  mNumJoints = num_joints;
  mJointModel.resize(mNumJoints);
  kfJoints.resize(mNumJoints);
  mJointEst.resize(mNumJoints);
  
  for ( int i = 0;i<mNumJoints;i++) {
	mJointModel.push_back(KalmanFilter_Models::Joint_Model());
	kfJoints.push_back(KalmanFilter(mJointModel[i]));
	mJointEst.push_back(KalmanFilter_Types::State());
  }
  
  std::cout << "StateEstimate::JointFilters::setSize -- " << num_joints << " joint kfs created" << std::endl;
  
}


void StateEstimate::JointFilters::updateStates(ulong utime, const vector<float> &_pos, vector<float> &_vel) {
  
	Eigen::VectorXd parameters;
	parameters.setZero();
	Eigen::VectorXd joint_position;
	
	parameters.resize(1);
	joint_position.resize(1);
  
  
  for ( int i = 0;i<mNumJoints;i++) {
	joint_position(0) = _pos[i];
	kfJoints[i].step(utime, parameters, joint_position);
	mJointEst[i] = kfJoints[i].getState();
	_vel[i] = mJointEst[i].X(1);// Insert the velocity estimate in the return vector
  }
  
  
}