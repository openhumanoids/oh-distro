#include <iostream>

#include "rng.hpp"

#include "particle.hpp"


#define VERBOSE_TXT 0

// STATUS OF NOISE DRIFT:
// when vo fails, last dvo is scaled to give dvo with added noise
// - each particle has the same vo except for the added noise.
// TODO:
// smooth the velocity using a fraction of the previous velocity e.g. 50:50
void Particle::MoveParticleDrift(rng *pRng, std::vector<double> move_var,double dtime){
  double sigma_u = 0.1;
  this->state.position(0) = pRng->Normal(this->state.position(0), sigma_u);
  this->state.position(1) = pRng->Normal(this->state.position(1), sigma_u);
  this->state.position(2) = pRng->Normal(this->state.position(2), sigma_u);

  return ;
}


void Particle::MoveParticle(rng *pRng, pf_state odom_diff,std::vector<double> move_var,double dtime){
  // NB: this module doesn't propogate a delta velocity
  // it calculates one from the elapsed time and a delta position whicch
  // is assumed to have come from VO-type motion estimation
  double sigma_u = 0.1;
  this->state.position(0) = pRng->Normal(this->state.position(0), sigma_u);
  this->state.position(1) = pRng->Normal(this->state.position(1), sigma_u);
  this->state.position(2) = pRng->Normal(this->state.position(2), sigma_u);
  return ;
}
  

void Particle::InitializeState(rng *pRng, double init_weight,
	Eigen::Isometry3d init_pose, std::vector <double> initial_var){
  
  this->state.position(0) =
  this->state.position(1) =
  this->state.position(2) =


}  

