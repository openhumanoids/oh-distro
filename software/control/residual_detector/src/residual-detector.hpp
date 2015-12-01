//
// Created by manuelli on 11/25/15.
//


#include <mutex>
#include <thread>
#include <chrono>
#include "drake/ForceTorqueMeasurement.h"
#include "drake/QPCommon.h"
#include "drake/Side.h"
#include "RobotStateDriver.hpp"
#include "ContactFilter.hpp"
#include "FootContactDriver.hpp"
#include "lcmtypes/drc/robot_state_t.hpp"
#include "lcmtypes/drc/foot_force_torque_t.hpp"
#include "lcmtypes/drc/residual_observer_state_t.hpp"
#include <bot_param/param_client.h>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Core>
#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>

#ifndef CONTROL_RESIDUAL_DETECTOR_H
#define CONTROL_RESIDUAL_DETECTOR_H

#endif //CONTROL_RESIDUAL_DETECTOR_H

std::vector<ContactFilterPoint> constructContactFilterPoints();
std::vector<ContactFilterPoint> constructContactFilterPointsFromFile(std::string filename);

struct ResidualDetectorState{
  double t_prev;
  VectorXd r;
  VectorXd integral;
  VectorXd gamma;
  VectorXd p_0;
  bool running;

  // for debugging purposes
  VectorXd gravity;
  VectorXd torque;
  VectorXd foot_contact_joint_torque;

};

struct ResidualArgs{
  // information that updateResidualState will need
  std::shared_ptr<DrakeRobotStateWithTorque> robot_state;
  std::map<Side, ForceTorqueMeasurement> foot_force_torque_measurement;
  std::map<Side, ForceTorqueMeasurement> foot_ft_meas_6_axis;
  std::map<Side, bool> b_contact_force;
};

class ResidualDetector{

public:
  // forward declaration
  ResidualDetector(std::shared_ptr<lcm::LCM> &lcm_, bool verbose_, std::string urdfFilename="none");
  ~ResidualDetector(){
  }
  void residualThreadLoop();
  void useFootForce(bool useGeometricJacobian = false);
  void useFootForceTorque();
  void kinematicChain(std::string linkName, int body_id =-1);
  void testContactFilterRotationMethod(bool useRandom=false);
  void testQP();
  void contactFilterThreadLoop(std::string filename);
  void computeActiveLinkContactFilter(bool publish);
  void activeLinkContactFilterThreadLoop();

private:
  std::shared_ptr<lcm::LCM> lcm_;
  bool running_;
  bool verbose_;
  bool useFootForceFlag;
  bool useFootFTFlag;
  bool newStateAvailable;
  bool newResidualStateAvailable;
  bool newResidualStateAvailableForActiveLink;
  bool foot_FT_6_axis_available;
  bool useGeometricJacobianFlag;
  int nq;
  int nv;
  double t_prev;
  BotParam* botparam_;
  RigidBodyManipulator drake_model;
  std::mutex pointerMutex;
  std::vector<std::string> state_coordinate_names;
  std::string publishChannel;

  VectorXd residualGainVector;
  double residualGain;
  ResidualArgs args;
  ResidualDetectorState residual_state;
//  ResidualDetectorState residual_state_w_forces;
  drc::residual_observer_state_t residual_state_msg;


  std::map<Side, int> foot_body_ids;

  std::shared_ptr<RobotStateDriver> state_driver;
  std::shared_ptr<FootContactDriver> foot_contact_driver;

  ContactFilter contactFilter;

  // forward declarations
  void onRobotState(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  drc::robot_state_t* msg);
  void onFootContact(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::foot_contact_estimate_t* msg);
  void onFootForceTorque(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const drc::foot_force_torque_t* msg);
  void updateResidualState();
  void publishResidualState(std::string publish_channel, const ResidualDetectorState &);
  void computeContactFilter(bool publishMostLikely, bool publishAll=false);
};

