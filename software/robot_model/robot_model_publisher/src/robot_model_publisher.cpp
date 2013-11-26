// file: robot_model_publisher.cpp
// Broadcasts an URDF only once. 

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <time.h>

#include "urdf/model.h"
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/drc_lcmtypes.hpp"
#include <ConciseArgs>

using namespace std;

void getHandConfiguration(std::vector<std::string> joint_names, int8_t &left_hand, int8_t &right_hand){

  if(find(joint_names.begin(), joint_names.end(), "left_f0_j0" ) != joint_names.end()){
    std::cout << "Robot fitted with left Sandia hand\n";
    left_hand = drc::robot_urdf_t::LEFT_SANDIA;
  }else if(find(joint_names.begin(), joint_names.end(), "left_finger[0]/joint_base" ) != joint_names.end()){
    std::cout << "Robot fitted with left iRobot hand\n";
    left_hand = drc::robot_urdf_t::LEFT_IROBOT;
  }else if(find(joint_names.begin(), joint_names.end(), "left_finger_1_joint_1" ) != joint_names.end()){
    std::cout << "Robot fitted with left Robotiq hand\n";
    left_hand = drc::robot_urdf_t::LEFT_ROBOTIQ;
  }else{
    std::cout << "Robot has no left hand\n"; 
    left_hand = drc::robot_urdf_t::LEFT_NONE;
  }

  if(find(joint_names.begin(), joint_names.end(), "right_f0_j0" ) != joint_names.end()){
    std::cout << "Robot fitted with right Sandia hand\n";
    right_hand = drc::robot_urdf_t::RIGHT_SANDIA;
  }else if(find(joint_names.begin(), joint_names.end(), "right_finger[0]/joint_base" ) != joint_names.end()){
    std::cout << "Robot fitted with right iRobot hand\n";
    right_hand = drc::robot_urdf_t::RIGHT_IROBOT;
  }else if(find(joint_names.begin(), joint_names.end(), "right_finger_1_joint_1" ) != joint_names.end()){
    std::cout << "Robot fitted with right Robotiq hand\n";
    left_hand = drc::robot_urdf_t::RIGHT_ROBOTIQ;
  }else{
    std::cout << "Robot has no right hand\n"; 
    right_hand = drc::robot_urdf_t::RIGHT_NONE;
  }
  
}




int main(int argc, char ** argv)
{
  string urdf_file = "path_to_your.urdf";
  ConciseArgs opt(argc, (char**)argv);
  opt.add(urdf_file, "u", "urdf_file","Robot URDF file");
  opt.parse();
  std::cout << "urdf_file: " << urdf_file << "\n";


  // get the entire file
  std::string xml_string;
  std::fstream xml_file(urdf_file.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);     
      xml_string += (line + "\n");
    }
    xml_file.close();
    std::cout << "File ["<< urdf_file << "]  parsed successfully.\n";    
  }
  else
  {
    std::cout << "ERROR: Could not open file ["<< urdf_file << "] for parsing.\n";
    return false;
  }
  
  // Get a urdf Model from the xml string and get all the joint names.
  urdf::Model robot_model; 
  if (!robot_model.initString( xml_string ))
  {
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
  }

  typedef std::map<std::string, boost::shared_ptr<urdf::Joint> > joints_mapType;
  std::vector<std::string> joint_names; 
  for( joints_mapType::const_iterator it = robot_model.joints_.begin(); it!=robot_model.joints_.end(); it++)
  {
    if(it->second->type!=6) // All joints that not of the type FIXED.
      joint_names.push_back(it->first);
  }
  std::cout<< "Number of Joints: " << joint_names.size() <<std::endl;    
  
  lcm::LCM lcm("");
  if(!lcm.good())
      return 1;
    
    
  drc::robot_urdf_t message;
  message.robot_name =robot_model.getName();
  message.urdf_xml_string = xml_string;
  getHandConfiguration(joint_names, message.left_hand, message.right_hand);
  
  std::cout << "Broadcasting urdf of robot [" << robot_model.getName() << "] as a string at 1Hz\n";
  struct timeval tv;
  while(true)
  {
    gettimeofday (&tv, NULL);
    message.utime = (int64_t) tv.tv_sec * 1000000 + tv.tv_usec; // TODO: replace with bot_timestamp_now() from bot_core
    lcm.publish("ROBOT_MODEL", &message);
    usleep(2000000); // used to publish at 1Hz, now publish at 0.5Hz
  }
    

    return 0;
}
