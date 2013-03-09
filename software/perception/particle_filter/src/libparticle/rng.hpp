#ifndef KMCL_RNG_HPP_
#define KMCL_RNG_HPP_

extern "C" {
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>
}

///////////////////////////////////////////////////////////////
// a random number generator
class rng{ 
  private:
    ///This is the type of random number generator underlying the class.
    const gsl_rng_type* type;
    ///This is a pointer to the internal workspace of the rng including its current state.
    gsl_rng* pWorkspace;
    
  public:
    ///Initialise the random number generator using default settings
    rng();
    ///Initialise the random number generator using the default seed for the type
    rng(const gsl_rng_type* Type);
    ///Initialise the random number generator using specified type and seed
    rng(const gsl_rng_type* Type,unsigned long int lSeed);    
    
    ///Free the workspace allocated for random number generation
    ~rng(){
      
    }    
    
    ///Generate a multinomial random vector with parameters (n,w[1:k]) and store it in X
    void Multinomial(unsigned n, unsigned k, const double* w, unsigned* X);
    ///Return a random number generated from a normal distribution with a specified mean and standard deviation
    double Normal(double dMean, double dStd);
    
};




#endif /* CONFIGURATION_HPP_ */