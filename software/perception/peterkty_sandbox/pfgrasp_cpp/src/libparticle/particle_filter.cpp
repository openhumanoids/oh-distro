
#include <lcm/lcm.h>
#include <limits>
#include "visualization/collections.hpp"
#include <lcmtypes/particle_pf_cloud_t.h>

#include "particle_filter.hpp"

using namespace std;
#define VERBOSE_TXT 0


void ParticleFilter::ReinitializeComplete(){
  for (int i=0;i<N_p;i++){
    particleset[i].InitializeState(pRng, log((double) 1/N_p), userdata_);
  }
}

void ParticleFilter::MoveParticles(){
  typedef boost::ptr_vector< Particle >::iterator ParticleIterator;
  for ( ParticleIterator it = particleset.begin(); it != particleset.end(); it++ ){
    it->MoveParticle(pRng);
  }
}


void ParticleFilter::UpdateWithLogLikelihoodParticles(){
  std::vector<float> loglikelihoods(N_p);
  std::cout << "dbg-UpdateWithLogLikelihoodParticles1" << std::endl;
  for(int i = 0; i < N_p; ++i) {
    loglikelihoods[i] = particleset[i].GetLogLikelihood(pRng, userdata_);
  }
  std::cout << "dbg-UpdateWithLogLikelihoodParticles2" << std::endl;
  LogLikelihoodParticles(loglikelihoods);
}

void ParticleFilter::LogLikelihoodParticles(std::vector<float> loglikelihoods){
  for(int i = 0; i < N_p; ++i) {
    particleset[i].AddToLogWeight(loglikelihoods[i]);
  }
    
  //Normalise the weights to sensible values....
  double dMaxWeight = -std::numeric_limits<double>::infinity();
  typedef boost::ptr_vector< Particle >::iterator ParticleIterator;
  for ( ParticleIterator it = particleset.begin(); it != particleset.end(); it++ ){
      dMaxWeight = max(dMaxWeight, it->GetLogWeight());
  }
  //printf("%f is adjustment\n",dMaxWeight);
  for ( ParticleIterator it = particleset.begin(); it != particleset.end(); it++ ){
      it->SetLogWeight(it->GetLogWeight() - (dMaxWeight));    
  }
  
  // keep a local copy:
  loglikelihoods_ = loglikelihoods;  
}


// Residual Resampling
void ParticleFilter::Resample(){
  //Resampling is done in place.
  double dWeightSum = 0;
  unsigned uMultinomialCount;

  //First obtain a count of the number of children each particle has.
  //Sample from a suitable multinomial vector and add the integer replicate
  //counts afterwards.
  dWeightSum = 0;
  ////for ( ParticleIterator it = particleset.begin(); it != particleset.end(); it++ ){
  for(int i = 0; i < N_p; ++i) {
    dRSWeights[i] =   particleset[i].GetWeight();  //pParticles[i].GetWeight();
    dWeightSum += dRSWeights[i];
  }
  
  uMultinomialCount = N_p;
  for(int i = 0; i < N_p; ++i) {
    dRSWeights[i] = N_p*dRSWeights[i] / dWeightSum;
    uRSIndices[i] = unsigned(floor(dRSWeights[i])); //Reuse temporary storage.
    dRSWeights[i] = (dRSWeights[i] - uRSIndices[i]);
    uMultinomialCount -= uRSIndices[i];
  }
  pRng->Multinomial(uMultinomialCount,N_p,dRSWeights,uRSCount);
  for(int i = 0; i < N_p; ++i) 
    uRSCount[i] += uRSIndices[i];


  ///////////////////////
  //Map count to indices to allow in-place resampling
  for (int i=0, j=0; i<N_p; ++i) {
    if (uRSCount[i]>0) {
      uRSIndices[i] = i;
      while (uRSCount[i]>1) {
	while (uRSCount[j]>0) ++j; // find next free spot
	uRSIndices[j++] = i; // assign index
	--uRSCount[i]; // decrement number of remaining offsprings
      }
    }
  }

  //Perform the replication of the chosen.
  for(int i = 0; i < N_p ; ++i) {
    if(uRSIndices[i] != (unsigned int) i)
      particleset[i].SetState(particleset[uRSIndices[i]].GetState());
    particleset[i].SetLogWeight(0);
  }  
  
}

double ParticleFilter::ConsiderResample(){
  //Check if the ESS is below some reasonable threshold and resample if necessary.
  //A mechanism for setting this threshold is required.
  double ESS = GetESS();
  if(ESS < dResampleThreshold) {
    nResampled = 1;
    Resample();
  }else
    nResampled = 0;

  return ESS;
}

double ParticleFilter::GetESS(void) {
  long double sum = 0;
  long double sumsq = 0;

  typedef boost::ptr_vector< Particle >::iterator ParticleIterator;
  for ( ParticleIterator it = particleset.begin(); it != particleset.end(); it++ ){
    sum += expl(it->GetLogWeight());
  }
 
  for ( ParticleIterator it = particleset.begin(); it != particleset.end(); it++ ){
    sumsq += expl(2.0*(it->GetLogWeight()));
  }  
   
  return expl(-log(sumsq) + 2*log(sum));
}


pf_state ParticleFilter::Integrate(){
  pf_state istate;
  long double wSum = 0;
  
  double pos[]={0,0,0};
  for(int i =0; i < N_p; i++){
    pf_state state = particleset[i].GetState();
    double w = expl(particleset[i].GetLogWeight());
    
    Eigen::Vector3d t(state.position);

    /*if(isnan_particle(t[0])){
      cout << t[0] << " is t[0] "<< i <<"\n";
      cout << "paused\n";
      int pause;
      cin >> pause;
    }*/

    pos[0] += w*t[0];
    pos[1] += w*t[1];
    pos[2] += w*t[2];
    wSum  += w ;
  }
  //cout << wSum << " is wSum\n";

  pos[0] /= wSum;
  pos[1] /= wSum;
  pos[2] /= wSum;
  
  Eigen::Vector3d ipos;
  ipos.setIdentity();
    ipos << pos[0],pos[1],pos[2];
  istate.position = ipos;
  
  return istate;
}

pf_state ParticleFilter::MaxWeight(){
  double w_max = numeric_limits<double>::min();
  double i_max =0;
  for(int i =0; i < N_p; i++){
    double w = expl(particleset[i].GetLogWeight());  // no need to do expl
    if (w > w_max){
      w_max = w;
      i_max =i;
    }
  }
  return particleset[i_max].GetState();
}

