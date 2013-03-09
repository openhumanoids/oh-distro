#include "rng.hpp"

///When called without any arguments, the constructor for the smc::rng class simply allocates a buffer for a
///random number generator of type gsl_rng_default (something which can be set at run-time via an environment
///variable) using its default seed (which again can be over-ridden using an environment variable).
rng::rng(void){
    gsl_rng_env_setup();
    type = gsl_rng_default;
    pWorkspace = gsl_rng_alloc(gsl_rng_default);
}

///When called with a single argument, the constructor for the smc::rng class allocates a buffer for a
///random number generator of the specified type and initialises it with the default seed (which can be set using
///and environment variable if one wishes to vary it at run-time).
///
///\param Type The type of a GSL random number generator
rng::rng(const gsl_rng_type* Type){
    gsl_rng_env_setup();
    type = Type;
    pWorkspace = gsl_rng_alloc(Type);
}

///When called with a pair of arguments, the constructor for the smc::rng class allocates a buffer for the specified
///random number generator type and initialises it with the specified seed (note that zero has special significance and
///is used to specify the seed with which the generator was originally used).
///
///\param Type The type of a GSL random number generator
///\param lSeed The value with which the generator is to be seeded
rng::rng(const gsl_rng_type* Type, unsigned long int lSeed){
    gsl_rng_env_setup();
    type = Type;
    pWorkspace = gsl_rng_alloc(Type);
    gsl_rng_set(pWorkspace, lSeed);
}



///This function simply passes the relevant arguments on to gsl_ran_multinomial.
///     \param n Number of entities to assign.
///     \param k Number of categories.
///     \param w Weights of category elements
///     \param X Array in which to return the sample values.
void rng::Multinomial(unsigned n, unsigned k, const double* w, unsigned* X){
    gsl_ran_multinomial(pWorkspace, k, n, w, X);
}


///This function simply calls gsl_ran_gaussian with the specified standard deviation and shifts the result.
///     \param dMean The mean of the distribution.
///     \param dStd  The standard deviation of the distribution
double rng::Normal(double dMean, double dStd){
    return dMean + gsl_ran_gaussian(pWorkspace, dStd);
}