
#include "JointFilters.h"

StateEstimate::JointFilters::JointFilters() {
	
}



void StateEstimate::JointFilters::setSize(const int &num_joints) {
  kfJoints.resize(num_joints);
  
  std::cout << "StateEstimate::JointFilters::setSize -- " << num_joints << " joint kfs created" << std::endl;
}


