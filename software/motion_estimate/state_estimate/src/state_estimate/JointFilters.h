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
  
//  void Test(const std::vector<float> &pos);
  void updateStates(ulong utime, const std::vector<float> &pos, std::vector<float> &vel, const int &num_joints);
  
private:
  
  int mNumJoints;
  vector<KalmanFilter_Models::Joint_Model> mJointModel;
  vector<KalmanFilter> kfJoints;
  vector<KalmanFilter_Types::State> mJointEst;
  
};

} // StateEstimate namespace


#endif /*JOINTFILTERING_H_*/
