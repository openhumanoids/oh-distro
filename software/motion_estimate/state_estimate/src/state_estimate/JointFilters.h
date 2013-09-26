#ifndef JOINTFILTERS_H_
#define JOINTFILTERS_H_

#include <kalman-filter/kalman_filter.hpp>
#include <vector>

namespace StateEstimate {

using namespace std;

class JointFilters {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  JointFilters();
  
  void setSize(const int &num_joints);
  
private:
  
  vector<KalmanFilter> kfJoints;
  
};

} // StateEstimate namespace


#endif /*JOINTFILTERING_H_*/
