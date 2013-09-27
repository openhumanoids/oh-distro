#ifndef JOINTFILTERS_H_
#define JOINTFILTERS_H_

#include <vector>

#include <Eigen/Dense>

#include <kalman-filter/kalman_filter.hpp>


namespace StateEstimate {

typedef unsigned long ulong;

using namespace std;

class JointFilters {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  JointFilters();
  
  void setSize(const int &num_joints);
  
  void updateStates(ulong utime, vector<float> _pos, float* _vel);
  
private:
  
  
  int mNumJoints;
  vector<KalmanFilter_Models::Joint_Model> mJointModel;
  vector<KalmanFilter> kfJoints;
  vector<KalmanFilter_Types::State> mJointEst;
  
};

} // StateEstimate namespace


#endif /*JOINTFILTERING_H_*/
