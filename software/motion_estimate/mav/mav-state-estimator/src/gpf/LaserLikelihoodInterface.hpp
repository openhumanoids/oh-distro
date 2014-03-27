#include <octomap_utils/octomap_util.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_utils/eigen_utils.hpp>

#include <mav_state_est/rbis.hpp>

#include <lcmtypes/octomap_utils.hpp>

#ifndef LASERLIKELIHOODINTERFACE_HPP_
#define LASERLIKELIHOODINTERFACE_HPP_

//TODO: static polymorphism?

namespace MavStateEst {

class LaserLikelihoodInterface {
public:

  //returns POSITIVE log likelihood of the laser point
  virtual double evaluatePointLogLikelihood(const double point[3])=0;

  //returns POSITIVE log likelihood of the state given the measurement
  virtual double evaluateScanLogLikelihood(const laser_projected_scan * lscan, const RBIS & scan_to_map);

  double cov_scaling_factor; //scaling factor on covariance via dividing summed log likelihoods
  double minNegLogLike;
};

class OctomapLikelihoodInterface: public LaserLikelihoodInterface {
public:
  OctomapLikelihoodInterface(const char * map_name, double _unknown_loglike, 
                             double _information_scaling_factor, double _blur_sigma);
  ~OctomapLikelihoodInterface();

  double evaluatePointLogLikelihood(const double point[3]);

  double unknown_loglike; //log-likelihood of an unknown cell in the map

  octomap::OcTree * ocTree;

  double minxyz[3];
  double maxxyz[3];
  
  
  // If querying the map from lcm (added by mfallon):
  double blur_sigma;
  // Create an LCM thread and listen for the OCTOMAP message
  void getOctomapFromLCM();
  // Handle the OCTOMAP message, blur it and quit the lcm thread
  void handleOctomapMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, const octomap::raw_t* msg);
  bool waiting_for_octomap_msg;
};

}

#endif /* LASERLIKELIHOODINTERFACE_HPP_ */
