// file: broadcast_robot_model.cpp
//

#include <stdio.h>
#include <iostream>
#include <fstream>

#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/robot_model/robot_urdf_t.hpp"



int main(int argc, char ** argv)
{

  if (argc < 2){
    std::cerr << "A URDF xml file is expected as an argument. USAGE: \"./broadcast_robot_model xxxxx.urdf\"" << std::endl;
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

    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    robot_model::robot_urdf_t message;
    message.timestamp = 0;
    message.urdf_xml_string = xml_string;

    lcm.publish("ROBOT_MODEL", &message);
    std::cout << "Broadcasting [" << filename <<"] as a string \n";

    return 0;
}
