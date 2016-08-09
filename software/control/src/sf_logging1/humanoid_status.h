#pragma once

#include "drake/systems/robotInterfaces/Side.h"
#include "drake/systems/controllers/controlUtil.h"
#include "mrd_logger.h"
#include "../RobotStateDriver.hpp"
#include "bot_core/robot_state_t.hpp"

#include "drake/systems/controllers/QPCommon.h"
#include "drake/util/yaml/yamlUtil.h"

using namespace Eigen;

/**
 * A handy struct that stores important kinematic properties.
 * For all the velocity / acceleration / wrench, the first 3 are always angular,
 * and the last 3 are linear.
 */
struct BodyOfInterest {
  /// Name of the BodyOfInterest
  std::string name;
  /// The link which this BOI is attached to
  const RigidBody* body;

  Eigen::Isometry3d pose;
  Vector3d rpy;
  Vector3d local_offset;
  /// This is the task space velocity, or twist of a frame that has the same
  /// orientation as the world frame, but located at the origin of the body
  /// frame.
  Vector6d vel;

  /// task space Jacobian, xdot = J * v
  MatrixXd J;
  /// task space Jd * v
  Vector6d Jdot_times_v;

  void Update(const RigidBodyTree &robot, const KinematicsCache<double> &cache) {
    pose = Isometry3d::Identity();
    pose.translation() = local_offset;
    pose = robot.relativeTransform(cache, 0, body->body_index) * pose;
    rpy = rotmat2rpy(pose.linear());

    vel = GetTaskSpaceVel(robot, cache, *body, local_offset);
    J = GetTaskSpaceJacobian(robot, cache, *body, local_offset);
    Jdot_times_v = GetTaskSpaceJacobianDotTimesV(robot, cache, *body, local_offset);
  }

  void AddToLog(const std::string &prefix, MRDLogger &logger) const {
    logger.AddChannel(prefix+name+"[x]", "m", pose.translation().data());
    logger.AddChannel(prefix+name+"[y]", "m", pose.translation().data()+1);
    logger.AddChannel(prefix+name+"[z]", "m", pose.translation().data()+2);
    logger.AddChannel(prefix+name+"[r]", "rad", rpy.data());
    logger.AddChannel(prefix+name+"[p]", "rad", rpy.data()+1);
    logger.AddChannel(prefix+name+"[y]", "rad", rpy.data()+2);

    logger.AddChannel(prefix+name+"d[x]", "m/s", vel.data()+3);
    logger.AddChannel(prefix+name+"d[y]", "m/s", vel.data()+4);
    logger.AddChannel(prefix+name+"d[z]", "m/s", vel.data()+5);
    logger.AddChannel(prefix+name+"d[wx]", "m/s", vel.data()+0);
    logger.AddChannel(prefix+name+"d[wy]", "m/s", vel.data()+1);
    logger.AddChannel(prefix+name+"d[wz]", "m/s", vel.data()+2);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Mostly a thin wrapper on RigidBodyTree.
 * It has kinematic values such as task space velocity of various body parts,
 * some measured contact force / torque information, joint torque, etc.
 */
class HumanoidStatus {
 public:
  /// Offset from the foot frame to contact position in the foot frame.
  static const Vector3d kFootToContactOffset;
  /// Offset from the foot frame to force torque sensor in the foot frame.
  static const Vector3d kFootToSensorOffset;

  explicit HumanoidStatus(const std::string &name, std::unique_ptr<RigidBodyTree> robot_in)
      : robot_(std::move(robot_in)), cache_(robot_->bodies) {

    // load from config file.
    YAML::Node config = YAML::LoadFile(name);
    rpc_ = parseKinematicTreeMetadata(config["kinematic_tree_metadata"], *robot_);

    std::map<Side, std::string> side_identifiers = {{Side::LEFT, "l"}, {Side::RIGHT, "r"}};
    for (const auto& side : Side::values)
      rpc_.hand_ids[side] = robot_->findLinkId(config["kinematic_tree_metadata"]["body_names"]["hands"][side_identifiers[side]].as<std::string>());
    rpc_.pelvis_id = robot_->findLinkId(config["kinematic_tree_metadata"]["body_names"]["pelvis"].as<std::string>());

    for (auto it = rpc_.foot_ids.begin(); it != rpc_.foot_ids.end(); it++) {
      std::cout << it->first.toString() << " foot name: " << robot_->getBodyOrFrameName(it->second) << std::endl;
    }

    flip_ft_ = false;
    if (config["flip_foot_ft_sensor"].as<int>())
      flip_ft_ = true;

    pelv_.name = config["kinematic_tree_metadata"]["body_names"]["pelvis"].as<std::string>();
    pelv_.body = robot_->findLink(pelv_.name).get();
    pelv_.local_offset.setZero();

    torso_.name = config["kinematic_tree_metadata"]["body_names"]["torso"].as<std::string>();
    torso_.body = robot_->findLink(torso_.name).get();
    torso_.local_offset.setZero();

    foot_[Side::LEFT].name = config["kinematic_tree_metadata"]["body_names"]["feet"]["l"].as<std::string>();
    foot_[Side::LEFT].body = robot_->findLink(foot_[Side::LEFT].name).get();
    foot_[Side::LEFT].local_offset.setZero();

    foot_[Side::RIGHT].name = config["kinematic_tree_metadata"]["body_names"]["feet"]["r"].as<std::string>();
    foot_[Side::RIGHT].body = robot_->findLink(foot_[Side::RIGHT].name).get();
    foot_[Side::RIGHT].local_offset.setZero();

    foot_sensor_[Side::LEFT].name = std::string("leftFootSensor");
    foot_sensor_[Side::LEFT].body = robot_->findLink(foot_[Side::LEFT].name).get();
    foot_sensor_[Side::LEFT].local_offset = kFootToSensorOffset;

    foot_sensor_[Side::RIGHT].name = std::string("rightFootSensor");
    foot_sensor_[Side::RIGHT].body = robot_->findLink(foot_[Side::RIGHT].name).get();
    foot_sensor_[Side::RIGHT].local_offset = kFootToSensorOffset;

    // build map
    body_name_to_id_ = std::unordered_map<std::string, int>();
    for (auto it = robot_->bodies.begin(); it != robot_->bodies.end(); ++it) {
      body_name_to_id_[(*it)->linkname] = it - robot_->bodies.begin();
    }

    joint_name_to_position_index_ = std::unordered_map<std::string, int>();
    for (int i = 0; i < robot_->num_positions; i++) {
      joint_name_to_position_index_[robot_->getPositionName(i)] = i;
    }
    for (size_t i = 0; i < robot_->actuators.size(); i++) {
      actuator_name_to_id_[robot_->actuators[i].name] = i;
    }

    time_ = time0_ = 0;

    position_.resize(robot_->num_positions);
    velocity_.resize(robot_->num_velocities);
    joint_torque_.resize(robot_->actuators.size());

    // init state driver
    int num_states = robot_->num_positions + robot_->num_velocities;
    std::vector<std::string> state_coordinate_names(num_states);
    for (int i = 0; i < num_states; i++) {
      state_coordinate_names[i] = robot_->getStateName(i);
    }
    state_driver_.reset(new RobotStateDriver(state_coordinate_names));

    inited_ = false;
  }

  /**
   * Do kinematics and compute useful information based on kinematics and
   * measured force torque information.
   * @param time is in seconds
   * @param q is the vector or generalized positions.
   * @param v is the vector of generalized velocities.
   * @param trq is joint torque, should be in the same order as @p v, not
   * in robot->actuators order
   * @param l_ft is wrench measured at the foot force torque sensor
   * location.
   * @param r_ft is wrench measured at the foot force torque sensor
   * location.
   * @param rot rotates @p l_ft and @p r_ft in the same orientation as
   * the foot frame. This is useful if the foot ft sensor has a different
   * orientation than the foot.
   */
  void Update(double t, const VectorXd& q, const VectorXd& v,
              const VectorXd& trq, const Vector6d& l_ft, const Vector6d& r_ft,
              const Matrix3d& rot = Matrix3d::Identity());

  inline const RigidBodyTree& robot() const { return *robot_; }
  inline const KinematicsCache<double>& cache() const { return cache_; }
  inline const std::unordered_map<std::string, int>& body_name_to_id() const {
    return body_name_to_id_;
  }
  inline const std::unordered_map<std::string, int>&
  joint_name_to_position_index() const {
    return joint_name_to_position_index_;
  }
  inline const std::unordered_map<std::string, int>& actuator_name_to_id()
      const {
    return actuator_name_to_id_;
  }

  inline double time() const { return time_; }
  inline const VectorXd& position() const { return position_; }
  inline const VectorXd& velocity() const { return velocity_; }
  inline const VectorXd& joint_torque() const { return joint_torque_; }
  inline const MatrixXd& M() const { return M_; }
  inline const VectorXd& bias_term() const { return bias_term_; }
  inline const Vector3d& com() const { return com_; }
  inline const Vector3d& comd() const { return comd_; }
  inline const MatrixXd& J_com() const { return J_com_; }
  inline const Vector3d& Jdot_times_v_com() const { return Jdot_times_v_com_; }
  inline const BodyOfInterest& pelv() const { return pelv_; }
  inline const BodyOfInterest& torso() const { return torso_; }
  inline const BodyOfInterest& foot(Side::SideEnum s) const { return foot_[s]; }
  inline const BodyOfInterest& foot(int s) const {
    return foot(Side::values.at(s));
  }
  inline const BodyOfInterest& foot_sensor(Side::SideEnum s) const {
    return foot_sensor_[s];
  }
  inline const Vector2d& cop() const { return cop_; }
  inline const Vector2d& cop_in_body_frame(Side::SideEnum s) const {
    return cop_in_body_frame_[s];
  }
  inline const Vector6d& foot_wrench_in_body_frame(Side::SideEnum s) const {
    return foot_wrench_in_body_frame_[s];
  }
  inline const Vector6d& foot_wrench_in_world_frame(Side::SideEnum s) const {
    return foot_wrench_in_world_frame_[s];
  }

  inline const BodyOfInterest& foot_sensor(int s) const {
    return foot_sensor(Side::values.at(s));
  }
  inline const Vector2d& cop_in_body_frame(int s) const {
    return cop_in_body_frame(Side::values.at(s));
  }
  inline const Vector6d& foot_wrench_in_body_frame(int s) const {
    return foot_wrench_in_body_frame(Side::values.at(s));
  }
  inline const Vector6d& foot_wrench_in_world_frame(int s) const {
    return foot_wrench_in_world_frame(Side::values.at(s));
  }
  inline bool has_init() const { return inited_; }

  void AddToLog(MRDLogger &logger) const;

  void Init(const bot_core::robot_state_t &msg);
  void ParseMsg(const bot_core::robot_state_t &msg);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  bool inited_;
  bool flip_ft_;

  RobotPropertyCache rpc_;
  std::unique_ptr<RigidBodyTree> robot_;
  KinematicsCache<double> cache_;
  /// Maps body name to its index
  std::unordered_map<std::string, int> body_name_to_id_;
  /// Maps joint name to its index
  std::unordered_map<std::string, int> joint_name_to_position_index_;
  /// Maps actuator name to its index
  std::unordered_map<std::string, int> actuator_name_to_id_;

  std::shared_ptr<RobotStateDriver> state_driver_;

  double time0_;
  double time_;

  // Pos and Vel include 6 dof for the floating base.
  VectorXd position_;  /// Position in generalized coordinate
  VectorXd velocity_;  /// Velocity in generalized coordinate
  // In the same order as vel, but trq contains only actuated joints.
  VectorXd joint_torque_;  /// Joint torque

  MatrixXd M_;          ///< Inertial matrix
  VectorXd bias_term_;  ///< Bias term: M * vd + h = tau + J^T * lambda

  // computed from kinematics
  Vector3d com_;               ///< Center of mass
  Vector3d comd_;              ///< Com velocity
  MatrixXd J_com_;             ///< Com Jacobian: comd = J_com * v
  Vector3d Jdot_times_v_com_;  ///< J_com_dot * v

  // These are at the origin of the each body (defined by the urdf) unless
  // specified otherwise.
  BodyOfInterest pelv_;     ///< Pelvis link
  BodyOfInterest torso_;    ///< Torso
  BodyOfInterest foot_[2];  ///< At the bottom of foot, right below the ankle.
  BodyOfInterest foot_sensor_[2];  ///< At the foot sensor, inside foot

  Vector2d cop_;  ///< Center of pressure
  Vector2d
      cop_in_body_frame_[2];  ///< Individual center of pressure in foot frame

  Vector6d
      foot_wrench_in_body_frame_[2];  ///< Wrench measured in the body frame
  Vector6d foot_wrench_in_world_frame_[2];  ///< Wrench rotated to world frame
};
