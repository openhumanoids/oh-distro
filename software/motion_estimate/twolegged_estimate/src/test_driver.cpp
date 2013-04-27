
#include <iostream>


#include "TwoLegOdometry.h"
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
  bool do_estimation = false;
  bool plot_footsteps = false;
  bool log_data_files = false;
  bool lcm_add_ext = false;
  ConciseArgs opt(argc, (char**)argv);
  opt.add(do_estimation, "e", "do_estimation","Do motion estimation");
  opt.add(plot_footsteps, "f", "plot_footsteps","Draw footstep poses in viewer");
  opt.add(log_data_files, "l", "log_data_files","Logging some data to file");
  opt.add(lcm_add_ext, "x", "lcm_add_ext", "Adding extension to the LCM messages");
  opt.parse();
  std::cout << "Do motion estimation: " << do_estimation<< std::endl;
  std::cout << "Draw footsteps: " << plot_footsteps << std::endl;
  std::cout << "Logging of data to file: " << log_data_files << std::endl;
  std::cout << "Adding extenstion to LCM messages: " << lcm_add_ext << std::endl;


  // register signal SIGINT and signal handler  
  signal(SIGINT, signalHandler);  

  cout << "Test driver main function for the twoleg motion estimate pod" << endl;

  boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if(!lcm->good())
    return 1;

  //_leg_odo = new TwoLegs::TwoLegOdometry(); // This is excessive, as the class is also invoked by LegOdometry_Handler() object.
  _legs_motion_estimate = new LegOdometry_Handler(lcm, do_estimation, plot_footsteps, log_data_files, lcm_add_ext);


  // Do some stuff with the objects to test them. Preferably here you must call the internal testing functions of the different objects created..
  //_legs_motion_estimate->run(false); // true means it will operate in testing mode and not listen LCM messages

  while(0 == lcm->handle());

  //delete _leg_odo;
  delete _legs_motion_estimate;

  cout << "Everything ends in test_driver for legs_motion_estimate program" << endl; 

  return 0;
}


// here somewhere we need to listen to the LCM packets and call back the function in the TwoLegOdometry class as to update its internal states
