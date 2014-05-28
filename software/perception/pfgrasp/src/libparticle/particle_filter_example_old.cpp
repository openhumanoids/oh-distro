#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <limits>
#include <boost/ptr_container/ptr_vector.hpp>
#include <vector>
#include <cmath>

//#include <lcm/lcm.h>
//#include <lcmtypes/bot_core.h>
//#include "visualization/collections.hpp"
//#include <lcmtypes/kmcl_pf_cloud_t.h>
#include <bot_core/bot_core.h>

//#include <bot_param/param_client.h>
//#include <bot_param/param_util.h>

#include "particle_filter.hpp"

using namespace std;


double vis_scale =20; // multiply the values so that mr-viewer2 can view them


long load_data(char const * szName, vector <vector <double> > &meas){
  FILE * fObs = fopen(szName,"rt");
  if (!fObs){
    cout << szName << " was not found in the specified path\n";
    exit(-1);
  }
  char* szBuffer = new char[1024];
  fgets(szBuffer, 1024, fObs);
  long lIterates = strtol(szBuffer, NULL, 10);

  for(long i = 0; i < lIterates; ++i){
      fgets(szBuffer, 1024, fObs);
      
      double x_meas = strtod(strtok(szBuffer, ",\r\n "), NULL);
      double y_meas = strtod(strtok(NULL, ",\r\n "), NULL);
      vector<double> meas_one;
      meas_one.push_back(x_meas);
      meas_one.push_back(y_meas);      
      meas.push_back(meas_one);
    }
  fclose(fObs);

  delete [] szBuffer;

  return lIterates;
}  



int main(int argc, char* argv[]){
  
  //lcm_t* publish_lcm = lcm_create(NULL);
  long N_p = 10000;

//  pf = new ParticleFilter(publish_lcm, N_p,init_pose,
//      config->get_initial_var(),  config->get_rng_seed(),resample_threshold_);
  Eigen::Isometry3d init_pose;
  init_pose.setIdentity();


  ParticleFilter pf(publish_lcm, N_p,init_pose,
      config->get_initial_var(),  config->get_rng_seed(),resample_threshold_);


  ParticleFilter pf(N_p);
  vector < vector < double > > meas;
  
  //Load observations
  int lIterates = load_data("data.csv",meas);
  cout << "Input Data:\n";
  for(int n=1 ; n < lIterates ; ++n) {
    cout << n << ": " << meas[n][0]  << ", " << meas[n][1]<< "   is data\n";
  }  
  
  pf_state mean_state = pf.Integrate();
  vector <double> null_meas;
  null_meas.push_back( 0.0);
  null_meas.push_back( 0.0);
  //pf.SendParticlesLCM(mean_state, null_meas);
  int pause;
  cout << "Enter a number to start the particle filter: ";
  cin >> pause;    
  
  
  for (int i=0;i<lIterates;i++){
    vector <double> current_meas;
    current_meas.push_back( meas[i][0]);
    current_meas.push_back( meas[i][1]);
    
    pf.MoveParticles(i,current_meas[0],current_meas[1]);
    double ESS = pf.ConsiderResample();
    cout << ESS/N_p << " is ESS| " << i << " iteration\n";
    pf_state mean_state = pf.Integrate();
    //pf.SendParticlesLCM(mean_state,current_meas);
    
    //    int pause;
    //    cout << "Enter a number to continue the particle filter: ";
    //    cin >> pause;    
    //      sleep(1);
  }
  
  cout << "finished\n";
  
  return 0;  
}
