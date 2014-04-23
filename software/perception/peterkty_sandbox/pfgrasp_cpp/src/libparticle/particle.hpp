#ifndef KMCL_PARTICLE_HPP_
#define KMCL_PARTICLE_HPP_
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "rng.hpp"
#include <pointcloud_tools/pointcloud_math.hpp>

//NaN is the only value, for which is expression value == value always false.
template<typename T>
inline bool isnan_particle(T value)
{
  return value != value;
}

// State Vector
class pf_state{
public:
  Eigen::Vector3d position;
};


///////////////////////////////////////////////////////////////
// A particle
class Particle{
    public:
      Particle(){}

      ~Particle(){};

      void Set(pf_state state_in, double logweight_in){
        state = state_in;
        logweight = logweight_in;
      }

//      void InitializeState(rng *pRng, double init_weight,
//          Eigen::Isometry3d init_pose, std::vector<double> initial_var);

      void InitializeState(rng *pRng, double init_weight, const void* userdata);

      void SetState(const pf_state & sValue){ state = sValue;  }

      pf_state GetState(){ return state; }

      /// Returns the particle's log weight.
      double GetLogWeight(){ return logweight; }
      /// Returns the particle's unnormalised weight.
      double GetWeight(void) const {return exp(logweight);}

      void MoveParticle(rng *pRng);

      double GetLogLikelihood(rng *pRng, const void* userdata);

      void AddToLogWeight(double dIncrement) { logweight += dIncrement;}
      void SetLogWeight(const double & dLogWeight) {logweight = dLogWeight;}

  private:
      pf_state state;
      /// Natural logarithm of this particle's weight.
      double logweight;
};


#endif /* CONFIGURATION_HPP_ */
