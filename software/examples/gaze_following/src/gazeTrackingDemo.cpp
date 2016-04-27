// Copyright 2015-16 Maurice Fallon, Wolfgang Merkt

#include <bot_core/timestamp.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

#include <drake/systems/plants/RigidBodyIK.h>
#include <drake/systems/plants/RigidBodyTree.h>
#include <drake/systems/plants/constraint/RigidBodyConstraint.h>
#include <drake/systems/plants/IKoptions.h>
#include <ConciseArgs>

#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>
#include <stack>
#include <ctime>

#include <lcm/lcm-cpp.hpp>
#include <model-client/model-client.hpp>

#include "lcmtypes/bot_core/joint_angles_t.hpp"
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/vector_3d_t.hpp"
#include "lcmtypes/bot_core/rigid_transform_t.hpp"

/**
 * Matlab-like tic toc for benchmarking
 * From: http://stackoverflow.com/a/13485583
 */
std::stack<clock_t> tictoc_stack;

void tic() { tictoc_stack.push(clock()); }

void toc() {
  std::cout << "Time elapsed: "
            << static_cast<double>((clock() - tictoc_stack.top())) /
                   CLOCKS_PER_SEC << std::endl;
  tictoc_stack.pop();
}

struct CommandLineConfig {
  std::string urdf_filename;
  Eigen::Vector3d gazeGoal;
};

inline double toRad(double deg) { return (deg * M_PI / 180); }

class App {
 public:
  App(std::shared_ptr<lcm::LCM> lcm_in, const CommandLineConfig &cl_cfg_in);

  ~App() {}

  int getConstraints(Eigen::VectorXd q_star, Eigen::VectorXd &q_sol);

  void solveGazeProblem();

  void robotStateHandler(const lcm::ReceiveBuffer *rbuf,
                         const std::string &channel,
                         const bot_core::robot_state_t *msg);

  void gazeGoalHandler(const lcm::ReceiveBuffer *rbuf,
                       const std::string &channel,
                       const bot_core::vector_3d_t *msg);

  void aprilTagTransformHandler(const lcm::ReceiveBuffer *rbuf,
                                const std::string &channel,
                                const bot_core::rigid_transform_t *msg);

  int get_trans_with_utime(std::string from_frame, std::string to_frame,
                           int64_t utime, Eigen::Isometry3d &mat);

 private:
  std::shared_ptr<lcm::LCM> lcm_;
  CommandLineConfig cl_cfg_;
  RigidBodyTree model_;

  bot_core::robot_state_t rstate_;
  std::map<std::string, int> dofMap_;

  BotParam *botparam_;
  BotFrames *botframes_;

  Eigen::Vector3d last_goal_pose_;
};

App::App(std::shared_ptr<lcm::LCM> lcm_in, const CommandLineConfig &cl_cfg_in)
    : lcm_(lcm_in), cl_cfg_(cl_cfg_in) {
  botparam_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
  botframes_ = bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);

  std::shared_ptr<ModelClient> model_client = std::shared_ptr<ModelClient>(
      new ModelClient(lcm_->getUnderlyingLCM(), 0));
  model_.addRobotFromURDFString(model_client->getURDFString());
  model_.compile();
  dofMap_ = model_.computePositionNameToIndexMap();

  // EST_ROBOT_STATE to update q_initial for planning
  lcm_->subscribe("EST_ROBOT_STATE", &App::robotStateHandler, this);

  // position3d_t with gaze goal
  lcm_->subscribe("GAZE_GOAL", &App::gazeGoalHandler, this);

  // Reads apriltag_to_car_beam frame on every update and sets it as gaze goal
  lcm_->subscribe("APRIL_TAG_TO_CAMERA_LEFT", &App::aprilTagTransformHandler,
                  this);
}

int App::get_trans_with_utime(std::string from_frame, std::string to_frame,
                              int64_t utime, Eigen::Isometry3d &mat) {
  int status;
  double matx[16];
  status = bot_frames_get_trans_mat_4x4_with_utime(
      botframes_, from_frame.c_str(), to_frame.c_str(), utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i, j) = matx[i * 4 + j];
    }
  }
  return status;
}

// Find the joint position indices corresponding to 'name'
std::vector<int> getJointPositionVectorIndices(const RigidBodyTree &model,
                                               const std::string &name) {
  std::shared_ptr<RigidBody> joint_parent_body = model.findJoint(name);
  int num_positions = joint_parent_body->getJoint().getNumPositions();
  std::vector<int> ret(static_cast<size_t>(num_positions));

  // fill with sequentially increasing values, starting at
  // joint_parent_body->position_num_start:
  std::iota(ret.begin(), ret.end(), joint_parent_body->position_num_start);
  return ret;
}

void findJointAndInsert(const RigidBodyTree &model, const std::string &name,
                        std::vector<int> *position_list) {
  auto position_indices = getJointPositionVectorIndices(model, name);

  position_list->insert(position_list->end(), position_indices.begin(),
                        position_indices.end());
}

Eigen::VectorXd robotStateToDrakePosition(
    const bot_core::robot_state_t &rstate,
    const std::map<std::string, int> &dofMap, int num_positions) {
  Eigen::VectorXd q = Eigen::VectorXd::Zero(num_positions, 1);
  for (int i = 0; i < rstate.num_joints; ++i) {
    auto iter = dofMap.find(rstate.joint_name.at(i));
    if (iter != dofMap.end()) {
      int index = iter->second;
      q(index) = rstate.joint_position[i];
    }
  }

  std::map<std::string, int>::const_iterator iter;
  iter = dofMap.find("base_x");
  if (iter != dofMap.end()) {
    int index = iter->second;
    q[index] = rstate.pose.translation.x;
  }
  iter = dofMap.find("base_y");
  if (iter != dofMap.end()) {
    int index = iter->second;
    q[index] = rstate.pose.translation.y;
  }
  iter = dofMap.find("base_z");
  if (iter != dofMap.end()) {
    int index = iter->second;
    q[index] = rstate.pose.translation.z;
  }

  Eigen::Vector4d quat;
  quat[0] = rstate.pose.rotation.w;
  quat[1] = rstate.pose.rotation.x;
  quat[2] = rstate.pose.rotation.y;
  quat[3] = rstate.pose.rotation.z;
  Eigen::Vector3d rpy = quat2rpy(quat);

  iter = dofMap.find("base_roll");
  if (iter != dofMap.end()) {
    int index = iter->second;
    q[index] = rpy[0];
  }

  iter = dofMap.find("base_pitch");
  if (iter != dofMap.end()) {
    int index = iter->second;
    q[index] = rpy[1];
  }

  iter = dofMap.find("base_yaw");
  if (iter != dofMap.end()) {
    int index = iter->second;
    q[index] = rpy[2];
  }

  return q;
}

/////////////////////////////////////////////////
int App::getConstraints(Eigen::VectorXd q_star, Eigen::VectorXd &q_sol) {
  Eigen::Vector2d tspan;
  tspan << 0, 1;

  // 0 Pelvis Position and Orientation Constraints
  int pelvis_link = model_.findLinkId("pelvis");
  Eigen::Vector3d pelvis_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d pelvis_pos0;
  pelvis_pos0(0) = rstate_.pose.translation.x;
  pelvis_pos0(1) = rstate_.pose.translation.y;
  pelvis_pos0(2) = rstate_.pose.translation.z;
  Eigen::Vector3d pelvis_pos_lb = pelvis_pos0;
  // pelvis_pos_lb(0) += 0.001;
  // pelvis_pos_lb(1) += 0.001;
  // pelvis_pos_lb(2) += 0.001;
  Eigen::Vector3d pelvis_pos_ub = pelvis_pos_lb;
  // pelvis_pos_ub(2) += 0.001;
  WorldPositionConstraint kc_pelvis_pos(&model_, pelvis_link, pelvis_pt,
                                        pelvis_pos_lb, pelvis_pos_ub, tspan);
  Eigen::Vector4d pelvis_quat_des(
      rstate_.pose.rotation.w, rstate_.pose.rotation.x, rstate_.pose.rotation.y,
      rstate_.pose.rotation.z);
  double pelvis_tol = 0;
  WorldQuatConstraint kc_pelvis_quat(&model_, pelvis_link, pelvis_quat_des,
                                     pelvis_tol, tspan);

  // 1 Back Posture Constraint
  PostureConstraint kc_posture_back(&model_, tspan);
  std::vector<int> back_idx;
  findJointAndInsert(model_, "torsoYaw", &back_idx);
  findJointAndInsert(model_, "torsoPitch", &back_idx);
  findJointAndInsert(model_, "torsoRoll", &back_idx);
  Eigen::VectorXd back_lb = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd back_ub = Eigen::VectorXd::Zero(3);
  kc_posture_back.setJointLimits(3, back_idx.data(), back_lb, back_ub);

  // 2 Neck Safe Joint Limit Constraints
  PostureConstraint kc_posture_neck(&model_, tspan);
  std::vector<int> neck_idx;
  findJointAndInsert(model_, "lowerNeckPitch", &neck_idx);
  findJointAndInsert(model_, "neckYaw", &neck_idx);
  findJointAndInsert(model_, "upperNeckPitch", &neck_idx);
  Eigen::VectorXd neck_lb = Eigen::VectorXd::Zero(3);
  neck_lb(0) = toRad(0.0);
  neck_lb(1) = toRad(-15.0);
  neck_lb(2) = toRad(-50.0);
  Eigen::VectorXd neck_ub = Eigen::VectorXd::Zero(3);
  neck_ub(0) = toRad(45.0);
  neck_ub(1) = toRad(15.0);
  neck_ub(2) = toRad(0.0);
  kc_posture_neck.setJointLimits(1, neck_idx.data(), neck_lb, neck_ub);

  // 3 Look At constraint:
  int head_link = model_.findLinkId("head");
  Eigen::Vector3d gaze_axis = Eigen::Vector3d(1, 0, 0);
  Eigen::Vector3d target = cl_cfg_.gazeGoal;
  Eigen::Vector3d gaze_origin = Eigen::Vector3d(0, 0, 0);
  double conethreshold = 0;
  WorldGazeTargetConstraint kc_gaze(&model_, head_link, gaze_axis, target,
                                    gaze_origin, conethreshold, tspan);

  // Assemble Constraint Set
  std::vector<RigidBodyConstraint *> constraint_array;
  constraint_array.push_back(&kc_pelvis_pos);
  constraint_array.push_back(&kc_pelvis_quat);
  constraint_array.push_back(&kc_gaze);
  constraint_array.push_back(
      &kc_posture_back);  // leave this out to also use the back
  constraint_array.push_back(
      &kc_posture_neck);  // safe neck joint limits - does not adhere to yaw?

  // Solve
  IKoptions ikoptions(&model_);
  int info;
  std::vector<std::string> infeasible_constraint;
  inverseKin(&model_, q_star, q_star, constraint_array.size(),
             constraint_array.data(), q_sol, info, infeasible_constraint,
             ikoptions);
  // printf("INFO = %d\n", info);
  if (info != 1) {
    for (auto it = infeasible_constraint.begin();
         it != infeasible_constraint.end(); it++)
      std::cout << *it << std::endl;
  }

  return info;
}

void App::solveGazeProblem() {
  tic();

  // Solve the IK problem for the neck:
  Eigen::VectorXd q_star(
      robotStateToDrakePosition(rstate_, dofMap_, model_.num_positions));
  Eigen::VectorXd q_sol(model_.num_positions);
  int info = getConstraints(q_star, q_sol);
  if (info != 1) {
    std::cout << "Problem not solved\n";
    toc();
    return;
  }

  std::vector<std::string> jointNames;
  for (int i = 0; i < model_.num_positions; i++) {
    jointNames.push_back(model_.getPositionName(i));
  }

  bot_core::joint_angles_t joint_position_goal_msg = bot_core::joint_angles_t();
  joint_position_goal_msg.utime = bot_timestamp_now();
  joint_position_goal_msg.num_joints = 3;
  joint_position_goal_msg.joint_name.assign(3, "");
  joint_position_goal_msg.joint_position.assign(3, (const float &)0.);

  std::vector<std::string>::iterator it1 =
      std::find(jointNames.begin(), jointNames.end(), "lowerNeckPitch");
  int lowerNeckPitchIndex = std::distance(jointNames.begin(), it1);

  std::vector<std::string>::iterator it2 =
      std::find(jointNames.begin(), jointNames.end(), "neckYaw");
  int neckYawIndex = std::distance(jointNames.begin(), it2);

  std::vector<std::string>::iterator it3 =
      std::find(jointNames.begin(), jointNames.end(), "upperNeckPitch");
  int upperNeckPitchIndex = std::distance(jointNames.begin(), it3);

  joint_position_goal_msg.joint_name[0] = "lowerNeckPitch";
  joint_position_goal_msg.joint_position[0] = q_sol[lowerNeckPitchIndex];
  joint_position_goal_msg.joint_name[1] = "neckYaw";
  joint_position_goal_msg.joint_position[1] = q_sol[neckYawIndex];
  joint_position_goal_msg.joint_name[2] = "upperNeckPitch";
  joint_position_goal_msg.joint_position[2] = q_sol[upperNeckPitchIndex];

  lcm_->publish("DESIRED_NECK_ANGLES", &joint_position_goal_msg);

  std::cout << "DESIRED_NECK_ANGLES " << (q_sol[lowerNeckPitchIndex]) << "*, "
            << (q_sol[neckYawIndex]) << "*, " << (q_sol[upperNeckPitchIndex])
            << "*" << std::endl;

  // Update last goal pose
  last_goal_pose_ = cl_cfg_.gazeGoal;

  toc();
}

void App::robotStateHandler(const lcm::ReceiveBuffer *rbuf,
                            const std::string &channel,
                            const bot_core::robot_state_t *msg) {
  rstate_ = *msg;
}

void App::gazeGoalHandler(const lcm::ReceiveBuffer *rbuf,
                          const std::string &channel,
                          const bot_core::vector_3d_t *msg) {
  cl_cfg_.gazeGoal(0) = msg->x;
  cl_cfg_.gazeGoal(1) = msg->y;
  cl_cfg_.gazeGoal(2) = msg->z;
  std::cout << "Updated gaze goal to " << cl_cfg_.gazeGoal << std::endl;
}

void App::aprilTagTransformHandler(const lcm::ReceiveBuffer *rbuf,
                                   const std::string &channel,
                                   const bot_core::rigid_transform_t *msg) {
  Eigen::Isometry3d aprilTagLocation;
  get_trans_with_utime("april_tag_car_beam", "local", msg->utime,
                       aprilTagLocation);
  cl_cfg_.gazeGoal(0) = aprilTagLocation.translation().x();
  cl_cfg_.gazeGoal(1) = aprilTagLocation.translation().y();
  cl_cfg_.gazeGoal(2) = aprilTagLocation.translation().z();

  double distance = std::sqrt(pow(last_goal_pose_(0) - cl_cfg_.gazeGoal(0), 2) +
                              pow(last_goal_pose_(1) - cl_cfg_.gazeGoal(1), 2) +
                              pow(last_goal_pose_(2) - cl_cfg_.gazeGoal(2), 2));

  // Only solve problem and publish goal if distance was significant
  if (distance > 0.06) {
    std::cout << "New gaze goal: " << cl_cfg_.gazeGoal.transpose() << std::endl;
    solveGazeProblem();
  }
}

int main(int argc, char *argv[]) {
  CommandLineConfig cl_cfg;
  cl_cfg.gazeGoal =
      Eigen::Vector3d(2, 1, 1.2);  // Position we would like the head to gaze at

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.gazeGoal(0), "x", "goal_x", "goal_x");
  parser.add(cl_cfg.gazeGoal(1), "y", "goal_y", "goal_y");
  parser.add(cl_cfg.gazeGoal(2), "z", "goal_z", "goal_z");
  parser.parse();

  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
  if (!lcm->good()) std::cerr << "ERROR: lcm is not good()" << std::endl;

  App app(lcm, cl_cfg);
  std::cout << "Ready" << std::endl
            << "============================" << std::endl;

  while (0 == lcm->handle()) {
  }
}
