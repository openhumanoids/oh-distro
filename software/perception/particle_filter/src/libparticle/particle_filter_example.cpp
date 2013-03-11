#include <stdio.h>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <lcm/lcm-cpp.hpp>
#include "particle_filter.hpp"

using namespace Eigen;
using namespace std;
using namespace boost;

class StatePub{
  public:
  StatePub(boost::shared_ptr<lcm::LCM> &lcm_);
  ~StatePub() {}
  boost::shared_ptr<lcm::LCM> lcm_;

  void demoParticleFilter();
  private:
  ParticleFilter* pf; 
};

StatePub::StatePub(boost::shared_ptr<lcm::LCM> &lcm_): lcm_(lcm_){
}

void StatePub::demoParticleFilter(){
  int64_t current_timestamp =0;

  int rng_seed = 1;
  double resample_threshold =0.5;
  int N_p =1000;
  std::vector <double> initial_var;
  initial_var.push_back(0.01);
  initial_var.push_back(0.01);
  initial_var.push_back(0.001);
  initial_var.push_back(0.001);
  
  double elapsed_time = 0.1;
  std::vector <double> success_var;
  success_var.push_back(0.0001);
  success_var.push_back(0.0001);
  success_var.push_back(0.000001);
  success_var.push_back(0.000001);
  
  std::vector <double> failure_var;
  failure_var.push_back(0.001);
  failure_var.push_back(0.001);
  failure_var.push_back(0.0001);
  failure_var.push_back(0.0001);
  
  Eigen::Isometry3d init_pose;
  init_pose.setIdentity();
  pf = new ParticleFilter(lcm_->getUnderlyingLCM(), N_p,init_pose,
            initial_var, rng_seed,resample_threshold);
      
  for (size_t i=0;i < 100; i++){
    pf_state odom_diff;
    odom_diff.pose.setIdentity();
    odom_diff.velocity.setIdentity();
    if (i < 50){
      std::cout << "Successful VO\n";
      odom_diff.pose.translation().x() = 0.1;
      pf->MoveParticles(odom_diff,success_var,elapsed_time,0); //success
    }else{
      std::cout << "Drifting VO\n";
      pf->MoveParticles(odom_diff,failure_var,elapsed_time,1); //failed motion estimation
    }
    
    std::vector<float> loglikelihoods;
    loglikelihoods.assign (N_p,0);    
    pf->LogLikelihoodParticles(loglikelihoods);

    pf->SendParticlesLCM(current_timestamp,0);//vo_estimate_status);

    double ESS;
    ESS= pf->ConsiderResample();
    std::cerr << i << ": " << ESS/N_p << " is ESS | " << current_timestamp << " iteration\n";
    double fraction_sleep =0.1;
    usleep( 1000.0*1000.0*fraction_sleep);    
  }
}

int main (int argc, char ** argv){

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return -1;

  StatePub app(lcm);
  app.demoParticleFilter();
  return 0;
}