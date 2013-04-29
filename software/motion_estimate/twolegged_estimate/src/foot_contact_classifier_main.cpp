
#include <iostream>


//#include "TwoLegOdometry.h"
#include "LegOdometry_LCM_Handler.hpp"
#include <csignal>
#include <exception>
#include <ConciseArgs>


using namespace std;



//TwoLegs::TwoLegOdometry* _leg_odo; // excessive, as this gets tested in _legs_motion_estimate
LegOdometry_Handler* _legs_motion_estimate;

void signalHandler( int signum ){
  cout << "Interrupt signal (" << signum << ") received.\n";

  // cleanup and close up 
  try{
    _legs_motion_estimate->terminate();
  } catch (std::exception &e){
    std::cout << "Exception occured during close out\n";
  }
  // terminate program  

  exit(signum);  

}

int main(int argc, char ** argv) {
  command_switches switches;
  
  switches.publish_footcontact_states = true;
  switches.do_estimation = false;
  switches.draw_footsteps = false;
  switches.log_data_files = false;
  switches.lcm_add_ext = false;
  switches.lcm_read_trues = false;
  
  ConciseArgs opt(argc, (char**)argv);
  //opt.add(switches.do_estimation, "e", "do_estimation","Do motion estimation");
  //cout << "Do motion estimation: " << switches.do_estimation<< endl;
  
  // register signal SIGINT and signal handler  
  signal(SIGINT, signalHandler);  

  cout << "Foot contact classifier publishing to LCM" << endl;

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  _legs_motion_estimate = new LegOdometry_Handler(lcm, &switches);

  while(0 == lcm->handle());

  //delete _leg_odo;
  delete _legs_motion_estimate;

  cout << "Foot contact classifier program is exiting -- LCM no longer active." << endl; 

  return 0;
}


// here somewhere we need to listen to the LCM packets and call back the function in the TwoLegOdometry class as to update its internal states
