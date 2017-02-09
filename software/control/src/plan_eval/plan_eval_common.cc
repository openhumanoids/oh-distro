//
// Created by manuelli on 2/8/17.
//

#include "plan_eval_common.h"

namespace plan_eval {

// load the data from the yaml node
FootContactPointData::FootContactPointData(YAML::Node node) {
  std::map <std::string, FootContactPointLocation> string_to_enum_map;

  string_to_enum_map["left_toe"] = FootContactPointLocation::left_toe;
  string_to_enum_map["right_toe"] = FootContactPointLocation::right_toe;
  string_to_enum_map["left_heel"] = FootContactPointLocation::left_heel;
  string_to_enum_map["right_heel"] = FootContactPointLocation::right_heel;

  for (auto const &it: string_to_enum_map) {
    Eigen::Vector3d contact_point;
    for (int i = 0; i < 3; i++) {
      contact_point(i) = node[it.first][i].as<double>();
    }
    this->contact_point_map_[it.second] = contact_point;
  }


  // initialize all_contact_points_ matrix
  int num_contact_points = this->contact_point_map_.size();
  all_contact_points_ = Eigen::Matrix3Xd(3, num_contact_points);

  int counter = 0;
  for (auto const &it: this->contact_point_map_) {
    all_contact_points_.col(counter) = it.second;
    counter++;
  }

  // initialize toe_contact_points_ matrix
  std::vector <FootContactPointLocation> toe_contact_group = {FootContactPointLocation::left_toe,
                                                              FootContactPointLocation::right_toe};
  toe_contact_points_ = Eigen::Matrix3Xd(3, toe_contact_group.size());

  for (int i = 0; i < toe_contact_group.size(); i++) {
    toe_contact_points_.col(i) = this->contact_point_map_[toe_contact_group[i]];
  }
}

// return all contact points matrix
Eigen::Matrix3Xd FootContactPointData::getAllContactPoints() {
  return this->all_contact_points_;
}

// return toe contact points matrix
Eigen::Matrix3Xd FootContactPointData::getToeContactPoints() {
  return this->toe_contact_points_;
}
}

