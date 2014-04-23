#ifndef KMCL_PARTICLE_FILTER_HPP_
#define KMCL_PARTICLE_FILTER_HPP_

#include <vector>
#include <iostream>
#include <boost/ptr_container/ptr_vector.hpp>

#include <boost/shared_ptr.hpp>
#include "rng.hpp"
#include "particle.hpp"



///////////////////////////////////////////////////////////////
class ParticleFilter{
  public:
    ParticleFilter(long N_p,  int rng_seed,
          double resample_threshold_, const void* userdata):
		      publish_lcm(publish_lcm),N_p(N_p),resample_threshold_(resample_threshold_){
      pRng = new rng(gsl_rng_default,rng_seed); // type, seed

      for (int i=0;i<N_p;i++){
        particleset.push_back(new Particle());
      }

      for (int i=0;i<N_p;i++){
        particleset[i].InitializeState(pRng, log((double) 1/N_p), userdata);
      }
      
      //Default:
      dResampleThreshold = resample_threshold_ * N_p;      
      
      // Allocate (Specifically c arrays because of GSL)
      dRSWeights = new double[N_p];    
      uRSCount  = new unsigned[N_p];
      uRSIndices = new unsigned[N_p];
    }
    
    ~ParticleFilter(){
      delete pRng;
      
      delete [] dRSWeights;
      delete [] uRSCount;
      delete [] uRSIndices;
    }

    // Restart the entire particle filter centered on this location:
    void ReinitializeComplete();
        
    // Move the particles
    // mode: 0: typical propogation with full delta pose value
    //       1: use velocity to propogate pose (when vo fails)
    void MoveParticles();

    void UpdateWithLogLikelihoodParticles();
    void LogLikelihoodParticles(std::vector<float> loglikelihoods);
    
    double ConsiderResample();
    void Resample();
    ///Calculates and Returns the Effective Sample Size.
    double GetESS(void) ;
    
    // Find the weight mean of the weight particle set:
    pf_state Integrate();

    // Find the particle with largest weight:
    pf_state MaxWeight();

    void SetParticleState(int i, pf_state particle_in_){ 
      particleset[i].SetState(particle_in_);
    }     
    
    pf_state GetParticleState(int i){ return particleset[i].GetState(); }     
    double GetParticleLogWeight(int i){ return particleset[i].GetLogWeight(); }     
    double GetParticleWeight(int i){ return particleset[i].GetWeight(); }     
    
    typedef boost::shared_ptr<ParticleFilter> Ptr;
    typedef boost::shared_ptr<const ParticleFilter> ConstPtr;
  private:
    lcm_t* publish_lcm;
    long N_p; // number of particles

    rng* pRng;
    boost::ptr_vector<Particle>    particleset;
    
    ///A flag which tracks whether the ensemble was resampled during this iteration
    int nResampled;
    ///The effective sample size at which resampling should be used.
    double dResampleThreshold;    
    /// normalized version:
    double resample_threshold_; // dResampleThreshold = resample_threshold_ *n_particles

    ///Structures used internally for resampling.
    double* dRSWeights;        
    unsigned int* uRSCount;
    unsigned int* uRSIndices;
    
    //Most recent lhoods [local copy]:
    std::vector <float> loglikelihoods_;

    const void* userdata;
};    



#endif 
