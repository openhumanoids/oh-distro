//
// Created by manuelli on 2/8/17.
//

#ifndef CONTROL_PLAN_EVAL_COMMON_H
#define CONTROL_PLAN_EVAL_COMMON_H

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/util/yaml/yamlUtil.h"
#include "drake/systems/robotInterfaces/BodyMotionData.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"




namespace Eigen {
  typedef Matrix<double, 6, 1> Vector6d;
  typedef Matrix<double, 7, 1> Vector7d;
};

namespace plan_eval{

enum FootContactPointLocation{
  left_toe,
  right_toe,
  left_heel,
  right_heel,
};

// stores contact point data for a specific contact body,
// e.g. L_FOOT, R_FOOT, PELVIS etc.
class ContactPointData{
public:
  Eigen::Matrix3Xd contact_points_;
  ContactPointData(){};
  ContactPointData(Eigen::Matrix3Xd contact_points): contact_points_(contact_points){};
  virtual Eigen::Matrix3Xd getAllContactPoints() const{
    return contact_points_;
  };
};

// subclasses ContactPointData for the specific case of feet
class FootContactPointData : public ContactPointData{
public:
  std::map<FootContactPointLocation, Eigen::Vector3d> contact_point_map_;

  // default constructor
  FootContactPointData(){};
  FootContactPointData(YAML::Node node);
  Eigen::Matrix3Xd getAllContactPoints() const;
  Eigen::Matrix3Xd getToeContactPoints() const;

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
  std::map<ContactState::ContactBody, std::shared_ptr<ContactPointData>> contact_point_data;
  std::map<Side, std::shared_ptr<ContactPointData>> foot_contact_point_data;
  double mu;
  double zmp_height;
  double initial_transition_time;
  double transition_trq_alpha_filter;
  bool flip_foot_ft_sensor;
};

struct RigidBodySupportStateElement {
  int body;
  double total_normal_force_upper_bound;
  double total_normal_force_lower_bound;
  Eigen::Matrix3Xd contact_points;
  bool use_contact_surface;
  Eigen::Vector4d support_surface;
};

typedef std::vector<RigidBodySupportStateElement> RigidBodySupportState;

enum class PlanExecutionStatus{
  UNKNOWN,
  EXECUTING,
  FINISHED,
};

enum class PlanType{
  UNKNOWN,
  STANDING,
  WALKING,
  BRACING,
  RECOVERING,
};

struct PlanStatus {
  PlanExecutionStatus executionStatus;
  PlanType planType;
};

struct DebugData{
  std::string plan_type;
  Eigen::Vector2d com_des;
  Eigen::Vector2d comd_des;
  Eigen::Vector2d comdd_des;
};

struct GenericPlanState{
  double plan_start_time = -1;
  PlanStatus plan_status;
  RigidBodySupportState support_state;
  std::vector<BodyMotionData> body_motions;
  PiecewisePolynomial<double> zmp_traj;

};

}



#endif //CONTROL_PLAN_EVAL_COMMON_H
