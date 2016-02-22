#ifndef KMCL_PARTICLE_FILTER_HPP_
#define KMCL_PARTICLE_FILTER_HPP_

#include <vector>
#include <limits>
#include <iostream>
#include <boost/ptr_container/ptr_vector.hpp>

#include <boost/shared_ptr.hpp>
#include "rng.hpp"
#include "particle.hpp"

#include <pronto_utils/pronto_vis.hpp>

///////////////////////////////////////////////////////////////
class ParticleFilter{
  public:
    
    ParticleFilter(lcm_t* publish_lcm, long N_p,
          Eigen::Isometry3d init_pose,
          std::vector<double> initial_var,
          int rng_seed,
          double resample_threshold_);

    ~ParticleFilter(){
      delete pRng;
      
      //for(int i = 0; i < N_p ; ++i) {
      // particleset[i].~Particle();
      //}
      //  particleset.clear();

      delete [] dRSWeights;
      delete [] uRSCount;
      delete [] uRSIndices;
    }

    // Restart a fraction of particle filter centered on this location:
    void ReinitializeFraction(Eigen::Isometry3d reinit_pose,
        std::vector<double> reinit_var, double fraction);
    // Restart the entire particle filter centered on this location:
    void ReinitializeComplete(Eigen::Isometry3d reinit_pose,std::vector<double> reinit_var);
        
    // Move the particles
    // mode: 0: typical propogation with full delta pose value
    //       1: use velocity to propogate pose (when vo fails)
    void MoveParticles(pf_state odom_diff, std::vector<double> propogate_var, double dtime,int mode);

    // Specifically set these 3 dimensions used for VO
    // depreciated: use SetDimension below
    void SetHeightPitchRoll(std::vector<double> height_pitch_roll);
    
    // Apply a manifold constraint to a plane
    // @input: xyzypr - the relative offset for each specific dimension 
    // @input: set_xyzypr - a binary vector of which dimensions to project
    // @input: plane_pose - the plane onto which to project. TODO: use something more useful
    void applyPlaneConstraint(std::vector<double> xyzrpy, std::vector<bool> set_xyzrpy, 
                  Eigen::Isometry3d plane_pose);
    
    void LogLikelihoodParticles(std::vector<float> loglikelihoods);
    
    void SendParticlesLCM(int64_t time_stamp,int vo_estimate_status);
    double ConsiderResample();
    void Resample();
    ///Calculates and Returns the Effective Sample Size.
    double GetESS(void) ;
    
    // Find the weight mean of the weight particle set:
    pf_state Integrate();
    // Return the position and attitude:
    Eigen::Isometry3d IntegratePose(){
      pf_state mean_state = Integrate();
      return mean_state.pose;
    }
    pf_state IntegrateVelocity();
    
    // Find the mean of the weight particle set:
    // assuming that the yaw/heading in near to wrapping.
    // this is only useful for 2D. a better solution would be to find the
    // mean of a set of weighted quaternions in a better manner
    pf_state IntegrateWrapSafe();
    
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

    pronto_vis* pc_vis_;

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
    
};    



#endif 
