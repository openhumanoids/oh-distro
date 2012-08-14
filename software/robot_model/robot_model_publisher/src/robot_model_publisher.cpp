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



int main(int argc, char ** argv)
{

  if (argc < 2){
    std::cerr << "A URDF xml file is expected as an argument. USAGE: \"./robot_model_publisher xxxxx.urdf\"" << std::endl;
    return -1;
  }

  std::string filename;
  filename = argv[1];

  // get the entire file
  std::string xml_string;
  std::fstream xml_file(filename.c_str(), std::fstream::in);
  if (xml_file.is_open())
  {
    while ( xml_file.good() )
    {
      std::string line;
      std::getline( xml_file, line);     
      xml_string += (line + "\n");
    }
    xml_file.close();
    std::cout << "File ["<< filename << "]  parsed successfully.\n";    
  }
  else
  {
    std::cout << "ERROR: Could not open file ["<< filename << "] for parsing.\n";
    return false;
  }
  
  
  urdf::Model robot;
  if (!robot.initFile(filename)){
    std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
    return -1;
  }

    lcm::LCM lcm;
    if(!lcm.good())
        return 1;
    

    
    drc::robot_urdf_t message;
    message.robot_name =robot.getName();
    message.urdf_xml_string = xml_string;
    std::cout << "Broadcasting urdf of robot [" << robot.getName() << "] as a string at 1Hz\n";
   struct timeval tv;
  while(true)
  {
    gettimeofday (&tv, NULL);
    message.utime = (int64_t) tv.tv_sec * 1000000 + tv.tv_usec; // TODO: replace with bot_timestamp_now() from bot_core
    lcm.publish("ROBOT_MODEL", &message);
    usleep(1000000);
  }
    

    return 0;
}
