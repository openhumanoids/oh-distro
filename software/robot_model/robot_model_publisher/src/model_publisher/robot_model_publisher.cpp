// file: robot_model_publisher.cpp
// Broadcasts an URDF only once. 

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <time.h>

#include "urdf/model.h"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc/robot_urdf_t.hpp"
#include <ConciseArgs>

#include <model-client/model-client.hpp>

using namespace std;

int main(int argc, char ** argv)
{
  string urdf_file = "path_to_your.urdf";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(urdf_file, "u", "urdf_file","Robot URDF file");
  opt.parse();
  std::cout << "urdf_file: " << urdf_file << "\n";

  ModelClient* model_client;
  model_client =  new ModelClient(urdf_file);
  
  lcm::LCM lcm("");
  if(!lcm.good())
      return 1;
    
  drc::robot_urdf_t message;
  message.robot_name = model_client->getRobotName();
  message.urdf_xml_string = model_client->getURDFString();
  message.left_hand = model_client->getLeftHand(); 
  message.right_hand = model_client->getRightHand();
  
  std::cout << "Broadcasting urdf of robot [" << model_client->getRobotName() << "] as a string at 1Hz\n";
  struct timeval tv;
  while(true){
    gettimeofday (&tv, NULL);
    message.utime = (int64_t) tv.tv_sec * 1000000 + tv.tv_usec; // TODO: replace with bot_timestamp_now() from bot_core
    lcm.publish("ROBOT_MODEL", &message);
    usleep(2000000); // used to publish at 1Hz, now publish at 0.5Hz
  }
    
  return 0;
}
