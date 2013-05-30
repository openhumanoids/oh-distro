#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <tinyxml.h>

struct GraspSeed{
  std::string geometry_name; 
  double xyz[3],rpy[3];
  int grasp_type;
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;

  void setFromXml(TiXmlElement* grasp_seed_xml);
  void writeToOtdf(const std::string& otdf_file);
  void unstoreFromOtdf(const std::string& otdf_file);
  static void clearAllFromOtdf(const std::string& otdf_file);

  bool operator==(const GraspSeed& other);

};
