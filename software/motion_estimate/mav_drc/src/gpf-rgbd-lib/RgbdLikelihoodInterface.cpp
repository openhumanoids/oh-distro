#include "RgbdLikelihoodInterface.hpp"

namespace MavStateEst {

double RgbdLikelihoodInterface::evaluateScanLogLikelihood(const laser_projected_scan * lscan, const RBIS & scan_to_map)
{
  BotTrans scan_to_map_bt;
  scan_to_map.getBotTrans(&scan_to_map_bt);

  double logLike = 0;
  for (int i = 0; i < lscan->npoints; i++) {
    if (lscan->point_status[i] > laser_valid_projection)
      continue;
    double proj_xyz[3];
    bot_trans_apply_vec(&scan_to_map_bt, point3d_as_array(&lscan->points[i]), proj_xyz); //TODO: would be cleaner to just use eigen
    logLike += evaluatePointLogLikelihood(proj_xyz);
  }
//  double max_log_like = -minNegLogLike * (double) lscan->numValidPoints;
//  double min_log_like = octomap_utils::LOGLIKE_HITS_EMPTY * (double) lscan->numValidPoints;
//  double percent_hit = (logLike - min_log_like) / (max_log_like - min_log_like);
//
//  double abes_magic_exponent = 2.0;
//  double abe_hack = pow(percent_hit, abes_magic_exponent);
//
//  return log(abe_hack);
//  fprintf(stderr,"%f, %d %f %f\n", logLike,lscan->numValidPoints, -minNegLogLike, octomap_utils::LOGLIKE_HITS_EMPTY);

//  double adams_magic_scaling_factor = .015; // .03 was working  1/adams_magic_scaling_factor is the scale of the covariance
//  return adams_magic_scaling_factor * logLike;

  return logLike / cov_scaling_factor;

}

RgbdOctomapLikelihoodInterface::RgbdOctomapLikelihoodInterface(const char * map_name, double _unknown_loglike,
    double _cov_scaling_factor)
{
  std::cout << "loading octomap from: " << map_name << std::endl;
  this->ocTree = octomap_utils::loadOctomap(map_name, &this->minNegLogLike);
  ocTree->getMetricMin(minxyz[0], minxyz[1], minxyz[2]);
  ocTree->getMetricMax(maxxyz[0], maxxyz[1], maxxyz[2]);

  unknown_loglike = _unknown_loglike;
  cov_scaling_factor = _cov_scaling_factor;
}
RgbdOctomapLikelihoodInterface::~RgbdOctomapLikelihoodInterface()
{
  delete ocTree;
}

double RgbdOctomapLikelihoodInterface::evaluatePointLogLikelihood(const double xyz[3])
{
  //check map bounds :-/
  if ((xyz[0] < minxyz[0] || xyz[1] < minxyz[1] || xyz[2] < minxyz[2]) ||
      (xyz[0] > maxxyz[0] || xyz[1] > maxxyz[1] || xyz[2] > maxxyz[2])) {
    return unknown_loglike;
  }

  octomap::OcTreeNode* node = ocTree->search(xyz[0], xyz[1], xyz[2]);
  if (node != NULL) {
    return -node->getLogOdds();
  }
  else {
    return unknown_loglike;
  }

}

}
