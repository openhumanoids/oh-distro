#include "LaserLikelihoodInterface.hpp"

namespace MavStateEst {

double LaserLikelihoodInterface::evaluateScanLogLikelihood(const laser_projected_scan * lscan, const RBIS & scan_to_map)
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



OctomapLikelihoodInterface::OctomapLikelihoodInterface(const char * map_name, double _unknown_loglike,
    double _cov_scaling_factor, double _blur_sigma)
{
  blur_sigma = _blur_sigma;

  if ( strstr(map_name,"from_lcm") != NULL ){
    std::cout << "will wait for an incoming octomap via lcm transmission\n";
    getOctomapFromLCM();
  }else{
    std::cout << "loading octomap from: " << map_name << std::endl;
    this->ocTree = octomap_utils::loadOctomap(map_name, &this->minNegLogLike);
  }
  
  ocTree->getMetricMin(minxyz[0], minxyz[1], minxyz[2]);
  ocTree->getMetricMax(maxxyz[0], maxxyz[1], maxxyz[2]);

  unknown_loglike = _unknown_loglike;
  cov_scaling_factor = _cov_scaling_factor;
}
OctomapLikelihoodInterface::~OctomapLikelihoodInterface()
{
  delete ocTree;
}

double OctomapLikelihoodInterface::evaluatePointLogLikelihood(const double xyz[3])
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


void OctomapLikelihoodInterface::getOctomapFromLCM(){
  // TODO: the parent class, LaserGPF, has an LCM object, should use that instead
  // TODO: destroy the lcm instance and the subscription properly
  lcm::LCM lcm;
  lcm.subscribe( "OCTOMAP"  ,&OctomapLikelihoodInterface::handleOctomapMessage,this);
  std::cout << "Waiting...\n";
  waiting_for_octomap_msg = true;
  while( (0 == lcm.handle()) && waiting_for_octomap_msg );  
}


void OctomapLikelihoodInterface::handleOctomapMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const octomap::raw_t* msg)
{
  std::cout << "received unblurred octomap. blurring with sigma="<< blur_sigma << "\n";
  bool write_output = false;
  
  std::stringstream datastream;
  datastream.write((const char*) msg->data.data(), msg->length);
  octomap::OcTree *unblurred_tree = new octomap::OcTree(1); //resolution will be set by data from message
  unblurred_tree->readBinary(datastream);
  
  
  timeval start; 
  timeval stop; 
  gettimeofday(&start, NULL);  // start timer  
  unblurred_tree->toMaxLikelihood();
  unblurred_tree->expand();
  double minNegLogLike;
  ocTree = octomap_utils::octomapBlur(unblurred_tree, blur_sigma, &minNegLogLike);
  gettimeofday(&stop, NULL);  // stop timer
  double time_to_blur = (stop.tv_sec - start.tv_sec) + 1.0e-6 *(stop.tv_usec - start.tv_usec);
  std::cout << "time to blur : " << time_to_blur << " sec" << std::endl;
  if (write_output){
    std::stringstream s;
    s <<  "/tmp/gpf_octree.bt" << "_blurred_" << blur_sigma ;
    printf("Saving blurred map to: %s\n", s.str().c_str());
    octomap_utils::saveOctomap(ocTree, s.str().c_str(), minNegLogLike);  
  }  
  
  waiting_for_octomap_msg = false;  
}



}
