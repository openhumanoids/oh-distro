#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <tinyxml.h>

struct GraspSeed{

  // information about the grasp similar to what's found in render_affordances::StickyHandStruc
  std::string geometry_name; 
  double xyz[3],rpy[3];
  int grasp_type;
  std::vector<std::string> joint_names;
  std::vector<double> joint_positions;

  // populate this with <grasp_seed> from an otdf file
  void setFromXml(TiXmlElement* grasp_seed_xml);

  // create a <grasp_seed> in given otdf_file
  void writeToOtdf(const std::string& otdf_file);

  // remove matching <grasp_seed> from otdf_file
  void unstoreFromOtdf(const std::string& otdf_file);

  // remove all <grasp_seed>s from otdf_file
  static void clearAllFromOtdf(const std::string& otdf_file);

  // smart comparison (within epsilon) to other
  bool operator==(const GraspSeed& other);

};
