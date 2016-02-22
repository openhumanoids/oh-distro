// #include <signal.h>
// #include <stdio.h>
// #include <stdlib.h>
// 
// #include <iostream>
// #include <limits>
// #include <boost/ptr_container/ptr_vector.hpp>
// #include <vector>
// #include <cmath>
// 
#include <lcm/lcm.h>
// #include <lcmtypes/bot_core.h>
#include "visualization/collections.hpp"
#include <lcmtypes/particle_pf_cloud_t.h>
// #include <bot_core/bot_core.h>

// #include <bot_param/param_client.h>
// #include <bot_param/param_util.h>

#include "particle_filter.hpp"

using namespace std;
#define VERBOSE_TXT 0


ParticleFilter::ParticleFilter(lcm_t* publish_lcm, long N_p,
  Eigen::Isometry3d init_pose, std::vector<double> initial_var,
  int rng_seed, double resample_threshold_):
  publish_lcm(publish_lcm),N_p(N_p),resample_threshold_(resample_threshold_){
  
  pRng = new rng(gsl_rng_default,rng_seed); // type, seed

  for (int i=0;i<N_p;i++){
    particleset.push_back(new Particle());
  }

  for (int i=0;i<N_p;i++){
    particleset[i].InitializeState(pRng, log((double) 1/N_p), init_pose, initial_var);
  }
      
  //Default:
  dResampleThreshold = resample_threshold_ * N_p;      
      
  // Allocate (Specifically c arrays because of GSL)
  dRSWeights = new double[N_p];    
  uRSCount  = new unsigned[N_p];
  uRSIndices = new unsigned[N_p];

  // Vis Config:
  pc_vis_ = new pronto_vis(publish_lcm);
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  bool reset = true;
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4002,"Particles",1,reset) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4008,"Mean Estimate",1,reset) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4108,"Mean Estimate [Wrap]",1,reset) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(4010,"Best Estimate",1,reset) );

}


void ParticleFilter::ReinitializeFraction(Eigen::Isometry3d reinit_pose,
    std::vector<double> reinit_var, double fraction){


  if (VERBOSE_TXT){
    std::stringstream ss;// (stringstream::in | stringstream::out);
    print_Isometry3d(reinit_pose,ss);
    std::cout << "in reinit: " << ss.str() << "\n";
  }


  // particles have unnormalized weights. so need the sum of weights in the system:
  // (value varies between  N_p * resamplethreshold   and   N_p
  double total_weight=0;
  for(int i = 0; i < N_p; ++i) {     
    total_weight +=   particleset[i].GetWeight();
  }    

  if (VERBOSE_TXT){
    cout << "total_weight: " << total_weight << "\n";
  }

  
  vector<int> pleft( N_p,0); // four ints with value 100
  for (int i=0;i < N_p;i++){
    pleft[i] = (int) i;
  }
  int size_pleft = N_p;
  
  // Select a random set of particles representing a fraction of the PF weight
  double running_frac =0.0;
  vector <int> chosen_particles;
  while(running_frac < fraction){ 
    int pleft_select = rand() % size_pleft; // number between [0, number of ints in pleft]
    int w_particle = pleft[pleft_select]; // look up which particle that corresponds to
    // remove the list pointer from the list
    pleft.erase (pleft.begin()+pleft_select);
    size_pleft--;
    chosen_particles.push_back(w_particle);

    // Reset location of particles - but keep original weight for now:
    double unnormalized_weight =particleset[w_particle].GetWeight();
    double normalized_weight  = unnormalized_weight/total_weight;
    particleset[w_particle].InitializeState(pRng, log(unnormalized_weight), reinit_pose, reinit_var);
    running_frac += (normalized_weight) ;

    if (VERBOSE_TXT){
      cout << "s " << pleft_select << " | p " << w_particle
          << " | nw " <<  normalized_weight << " | f " << running_frac << "/" << fraction << "\n";
    }
  }

  // Assign a uniform weight to each of the modified particles:
  double new_weight = (total_weight * running_frac)/chosen_particles.size();
  for(size_t i = 0; i < chosen_particles.size(); i++) {
    particleset[chosen_particles[i]].SetLogWeight(log( new_weight));
  }

  if (VERBOSE_TXT){
    double total_weight=0;
    for(int i = 0; i < N_p; ++i) {
      total_weight +=   particleset[i].GetWeight();
    }
    cout << "total_weight AFTER: " << total_weight << "\n";
    cout << "replaced " << chosen_particles.size() << " parts with weight " << new_weight << "\n";

    cout << "checking the particle state\n";
    for (int i=0; i<N_p; i++) {
      pf_state particle_state;
      particle_state =GetParticleState(i);
      Eigen::Quaterniond r_temp(particle_state.velocity.rotation());
      if ( isnan_particle (r_temp.w() )){
        std::stringstream ss6;
        print_Isometry3d(particle_state.velocity,ss6);
        std::cout << "particle_state.velocity after periodic reinit: " << ss6.str() << " ["<< i<< "\n";
        std::cout << "paused\n";
        int pause;
        std::cin >> pause;
      }
    }

    pf_state reinit_state = IntegrateWrapSafe();
    std::stringstream ss;// (stringstream::in | stringstream::out);
    print_Isometry3d(reinit_state.velocity,ss);
    std::cout << "after periodic reinit: " << ss.str() << "\n";
    Eigen::Quaterniond r_temp(reinit_state.velocity.rotation());
    if ( isnan_particle (r_temp.w() )){
      std::cout << "paused\n";
      int pause;
      std::cin >> pause;
    }
  }

}

void ParticleFilter::ReinitializeComplete(Eigen::Isometry3d reinit_pose,std::vector<double> reinit_var){
      for (int i=0;i<N_p;i++){
	particleset[i].InitializeState(pRng, log((double) 1/N_p), reinit_pose, reinit_var);
      }  
}

void ParticleFilter::MoveParticles(pf_state odom_diff, std::vector<double> move_var,
	  double dtime, int mode ){
  typedef boost::ptr_vector< Particle >::iterator ParticleIterator;
  for ( ParticleIterator it = particleset.begin(); it != particleset.end(); it++ ){
    if (mode==0){
      it->MoveParticle(pRng,odom_diff,move_var,dtime);
    }else if (mode==1){
      it->MoveParticleDrift(pRng,move_var,dtime);
    }else{
      cerr << "invalid mode ("<< mode <<") given in ParticleFilter::MoveParticles\n";
      exit(-1);
    }
  }
}


void ParticleFilter::SetHeightPitchRoll(std::vector<double> height_pitch_roll){
  // Set the HPR at fixed values:
  // should this be done outsite the particle filter class?
  for(int i = 0; i < N_p ; ++i) {
    pf_state state = particleset[i].GetState();
    Eigen::Quaterniond r(state.pose.rotation());
    double yaw,pitch,roll;
    quat_to_euler(r, roll, pitch, yaw) ;
    pitch =height_pitch_roll[1];
    roll  =height_pitch_roll[2];

    Eigen::Quaterniond m = euler_to_quat(roll, pitch, yaw);
    Eigen::Isometry3d ipose;
    ipose.setIdentity();
    ipose.translation() << state.pose.translation().x(),state.pose.translation().y(), height_pitch_roll[0];
    ipose.rotate(m);

    state.pose = ipose;
    particleset[i].SetState(state);  
  }
}



void ParticleFilter::applyPlaneConstraint(std::vector<double> xyzrpy,
        std::vector<bool> set_xyzrpy, Eigen::Isometry3d plane_pose){

  for(int i = 0; i < N_p ; ++i) {  
    pf_state state = particleset[i].GetState();
    state.pose = plane_pose.inverse() * state.pose ;
  
    double current_rpy[3];
    quat_to_euler(  Eigen::Quaterniond( state.pose.rotation()) , current_rpy[0], current_rpy[1], current_rpy[2]);
    if (set_xyzrpy[3]){ current_rpy[0] = xyzrpy[3]; }
    if (set_xyzrpy[4]){ current_rpy[1] = xyzrpy[4]; }
    if (set_xyzrpy[5]){ current_rpy[2] = xyzrpy[5]; }
    Eigen::Quaterniond revised_quat = euler_to_quat( current_rpy[0], current_rpy[1], current_rpy[2]);             
    
    Eigen::Isometry3d ipose;
    ipose.setIdentity();
    ipose.translation() << state.pose.translation();
    if (set_xyzrpy[0]){ ipose.translation().x() = xyzrpy[0]; }
    if (set_xyzrpy[1]){ ipose.translation().y() = xyzrpy[1]; }
    if (set_xyzrpy[2]){ ipose.translation().z() = xyzrpy[2]; }

    ipose.rotate(revised_quat);
    state.pose = ipose;    
    state.pose = plane_pose * state.pose;
    particleset[i].SetState(state);  
  }
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
  double rpy[]={0,0,0};
  for(int i =0; i < N_p; i++){
    pf_state state = particleset[i].GetState();
    double w = expl(particleset[i].GetLogWeight());
    
    Eigen::Vector3d t(state.pose.translation());
    Eigen::Quaterniond r(state.pose.rotation());
    double yaw,pitch,roll;
    quat_to_euler(r, roll, pitch, yaw) ;

    /*if(isnan_particle(t[0])){
      cout << t[0] << " is t[0] "<< i <<"\n";
      cout << "paused\n";
      int pause;
      cin >> pause;
    }*/

    pos[0] += w*t[0];
    pos[1] += w*t[1];
    pos[2] += w*t[2];
    rpy[0] += w*roll;
    rpy[1] += w*pitch;
    rpy[2] += w*yaw;
    wSum  += w ;
  }
  //cout << wSum << " is wSum\n";

  pos[0] /= wSum;
  pos[1] /= wSum;
  pos[2] /= wSum;
  rpy[0] /= wSum;
  rpy[1] /= wSum;
  rpy[2] /= wSum;
  
  Eigen::Quaterniond m = euler_to_quat(rpy[0], rpy[1], rpy[2]);
  Eigen::Isometry3d ipose;
  ipose.setIdentity();
    ipose.translation() << pos[0],pos[1],pos[2];
  ipose.rotate(m);
  istate.pose = ipose;
  
  return istate;
}

pf_state ParticleFilter::IntegrateVelocity(){
  pf_state istate;
  long double wSum = 0;
  
  double pos[]={0,0,0};
  double rpy[]={0,0,0};
  for(int i =0; i < N_p; i++){
    pf_state state = particleset[i].GetState();
    double w = expl(particleset[i].GetLogWeight());
    
    Eigen::Vector3d t(state.velocity.translation());
    Eigen::Quaterniond r(state.velocity.rotation());
    double yaw,pitch,roll;
    quat_to_euler(r, roll, pitch, yaw) ;

    pos[0] += w*t[0];
    pos[1] += w*t[1];
    pos[2] += w*t[2];
    rpy[0] += w*roll;
    rpy[1] += w*pitch;
    rpy[2] += w*yaw;
    wSum  += w ;
  }
  pos[0] /= wSum;
  pos[1] /= wSum;
  pos[2] /= wSum;
  rpy[0] /= wSum;
  rpy[1] /= wSum;
  rpy[2] /= wSum;
  
  Eigen::Quaterniond m = euler_to_quat(rpy[0], rpy[1], rpy[2]);
  Eigen::Isometry3d ipose;
  ipose.setIdentity();
  ipose.translation() << pos[0],pos[1],pos[2];
  ipose.rotate(m);
  istate.velocity = ipose;
  
  return istate;
}




pf_state ParticleFilter::IntegrateWrapSafe(){
  pf_state istate;
  long double wSum = 0;
  

  double thres = 0.05*N_p;  // proportion of headings to be withing 5d of 0 to count as ackward
  double count =0.0;
  bool wrapping = false;  
  for(int i =0; i < N_p; i++){
    pf_state state = particleset[i].GetState();
    
    Eigen::Vector3d t(state.pose.translation());
    Eigen::Quaterniond r(state.pose.rotation());
    double yaw,pitch,roll;
    quat_to_euler(r, roll, pitch, yaw) ;
    
    if (yaw < -170*M_PI/180){
      count=count+1.0;
    }else if (yaw > 170*M_PI/180){
      count=count+1.0;
    }
    if (count > thres){
      wrapping=true;
    }
  }  
  
  double pos[]={0,0,0};
  double rpy[]={0,0,0};
  for(int i =0; i < N_p; i++){
    pf_state state = particleset[i].GetState();
    double w = expl(particleset[i].GetLogWeight());
    
    Eigen::Vector3d t(state.pose.translation());
    Eigen::Quaterniond r(state.pose.rotation());
    double yaw,pitch,roll;
    quat_to_euler(r, roll, pitch, yaw) ;

    pos[0] += w*t[0];
    pos[1] += w*t[1];
    pos[2] += w*t[2];
    
    if (wrapping){
//      cerr << yaw << ": " << i<< "\n";
      if (yaw < 0){
	yaw = 2*M_PI + yaw ;
      }
    }
    
    rpy[0] += w*roll;
    rpy[1] += w*pitch;
    rpy[2] += w*yaw;
    wSum  += w ;
  }
  pos[0] /= wSum;
  pos[1] /= wSum;
  pos[2] /= wSum;
  rpy[0] /= wSum;
  rpy[1] /= wSum;
  rpy[2] /= wSum;
  
  Eigen::Quaterniond m = euler_to_quat(rpy[0], rpy[1], rpy[2]);
  Eigen::Isometry3d ipose;
  ipose.setIdentity();
    ipose.translation() << pos[0],pos[1],pos[2];
  ipose.rotate(m);
  istate.pose = ipose;

  
  return istate;
}



pf_state ParticleFilter::MaxWeight(){
  double w_max = numeric_limits<double>::min();
  double i_max =0;
  for(int i =0; i < N_p; i++){
    double w = expl(particleset[i].GetLogWeight());
    if (w > w_max){
      w_max = w;
      i_max =i;
    }
  }
  return particleset[i_max].GetState();
}


bot_core_pose_t getLCMPoseFromIsometry3d(Eigen::Isometry3d pose, int64_t time_stamp){
  bot_core_pose_t pose_msg;
  memset(&pose_msg, 0, sizeof(pose_msg));
  pose_msg.utime =   time_stamp;// msg->timestamp;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();  
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();  
  pose_msg.orientation[1] =  r_x.x();  
  pose_msg.orientation[2] =  r_x.y();  
  pose_msg.orientation[3] =  r_x.z();
  return pose_msg;  
}



void ParticleFilter::SendParticlesLCM(int64_t time_stamp,int vo_estimate_status){
  // NB: these methods compile, but haven't been tested since changing recently

  pf_state mean_state;
  mean_state = Integrate();
  mean_state = IntegrateWrapSafe();

  bot_core_pose_t pose_msg = getLCMPoseFromIsometry3d(mean_state.pose, time_stamp);
  bot_core_pose_t_publish(publish_lcm, "POSE", &pose_msg);
  
  pf_state mean_velocity;
  mean_velocity = IntegrateVelocity();
  bot_core_pose_t vel_msg = getLCMPoseFromIsometry3d(mean_velocity.velocity, time_stamp);
  bot_core_pose_t_publish(publish_lcm, "POSE_VELOCITY", &vel_msg);

  std::vector<Isometry3dTime> particle_poses;
  for (int i=0;i<N_p;i++){
    pf_state particle_state = particleset[i].GetState();
    Isometry3dTime particle_pose(time_stamp + i, particle_state.pose);
    particle_poses.push_back(particle_pose);
  }
  pc_vis_->pose_collection_to_lcm_from_list(4002, particle_poses);

  Isometry3dTime poseT = Isometry3dTime(time_stamp, mean_state.pose);
  pc_vis_->pose_to_lcm_from_list(4008, poseT);

  pf_state mean_state_wrap;
  mean_state_wrap = IntegrateWrapSafe();
  Isometry3dTime poseTwrap = Isometry3dTime(time_stamp, mean_state_wrap.pose);
  pc_vis_->pose_to_lcm_from_list(4108, poseTwrap);

  particle_pf_cloud_t pc;
  pc.utime = time_stamp; 
  pc.nparticles = N_p;
  double likelihoods[N_p];
  double weights[N_p];
  bot_core_pose_t particles[N_p];
  for (int i=0;i<N_p;i++){
    particles[i].utime = time_stamp;
    likelihoods[i] = loglikelihoods_[i];// particleset[i].GetLogWeight();
  }
  pc.weights = weights;
  pc.likelihoods = likelihoods;
  pc.particles = particles;
  // resample thresholds:
  pc.neff = GetESS()/N_p;
  pc.resample_threshold = resample_threshold_;  // currently hard coded
  pc.vo_estimate_status = vo_estimate_status-1; // this -1 is because fovis and fovis_upate_t types are not in sync

  particle_pf_cloud_t_publish(publish_lcm, "PF_CLOUD", &pc);   
  

  // Highest Weight Particle:
  pf_state max_particle;
  max_particle = MaxWeight();
  Isometry3dTime poseTmax = Isometry3dTime(time_stamp, max_particle.pose);
  pc_vis_->pose_to_lcm_from_list(4010, poseTmax);


}
