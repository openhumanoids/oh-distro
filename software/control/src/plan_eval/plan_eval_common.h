//
// Created by manuelli on 2/8/17.
//

#ifndef CONTROL_PLAN_EVAL_COMMON_H
#define CONTROL_PLAN_EVAL_COMMON_H

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/util/yaml/yamlUtil.h"


namespace plan_eval{

enum FootContactPointLocation{
  left_toe,
  right_toe,
  left_heel,
  right_heel,
};



class FootContactPointData{
public:
  std::map<FootContactPointLocation, Eigen::Vector3d> contact_point_map_;

  // default constructor
  FootContactPointData(){};
  FootContactPointData(YAML::Node node);
  Eigen::Matrix3Xd getAllContactPoints();
  Eigen::Matrix3Xd getToeContactPoints();

private:
  Eigen::Matrix3Xd all_contact_points_;
  Eigen::Matrix3Xd toe_contact_points_;
};


struct WalkingPlanConfig{
  double pelvis_height;
  double ss_duration; // single stance duration
  double ds_duration; // double stance duration
  double pre_weight_transfer_scale;
  double min_Fz;
  bool use_force_bounds;
  double extend_foot_down_z_vel;
  double swing_foot_touchdown_z_vel;
  double swing_foot_touchdown_z_offset;
  double swing_foot_xy_weight_multiplier;
  double swing_foot_z_weight_multiplier;
  double pelvis_z_weight_multiplier;
  double left_foot_zmp_y_shift;
  double right_foot_zmp_y_shift;
  bool constrain_back_bkx;
  bool constrain_back_bky;
  bool constrain_back_bkz;
};

// struct to store all data contained in plan eval config YAML files
struct GenericPlanConfig{
  WalkingPlanConfig walking_plan_config;
  std::map<Side, FootContactPointData> foot_contact_point_data;
  double mu;
  double zmp_height;
  double initial_transition_time;
  double transition_trq_alpha_filter;
  bool flip_foot_ft_sensor;
};

}



#endif //CONTROL_PLAN_EVAL_COMMON_H
