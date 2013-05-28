#pragma once

#include <string>
#include <vector>

struct GraspSeed{
  std::string parent_name;
  double xyz[3],rpy[3];
  int grasp_type;
  std::vector<double> joint_positions;

  //TODO move serializer and deserializer here
};
