#include <boost/program_options.hpp>

#include "drake/thirdParty/tinyxml2/tinyxml2.h"

#include "finalPosePlanner/FinalPosePlanner.hpp"
#include "fppUtil/fppUtil.hpp"

using namespace std;
using namespace Eigen;
using namespace tinyxml2;
namespace po = boost::program_options;

template<typename T>
void addTextToElement(XMLElement *element, T value)
{
  stringstream ss;
  if (element->GetText())
  {
    ss << element->GetText() << " ";
  }
  ss << value;
  element->SetText(ss.str().c_str());
}

int main(int argc, char* argv[])
{
  Matrix<double, 7, 1> endeffector_pose;
  endeffector_pose << 0.8, 0, 1.0625, 0.707106781186548, 0, 0, -0.707106781186547;

  Vector3d endeffector_point(0.08, 0.07, 0);

  string endeffector_name = "leftPalm";

  stringstream ss;

  ss << getenv("DRC_BASE") << "/software/planning/capabilityMap.log";
  CapabilityMap cm(ss.str());
  ss.str("");
  ss << getenv("DRC_BASE") << "/../drc-testing-data/final_pose_planner/val_description/CapabilityMapMatlab.bin";
  cm.loadFromMatlabBinFile(ss.str());

  boost::shared_ptr<lcm::LCM> theLCM(new lcm::LCM);
  if (!theLCM->good())
  {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  ss.str("");
  ss << getenv("DRC_BASE") << "/software/models/val_description/urdf/valkyrie_sim_drake.urdf";
  RigidBodyTree robot(ss.str());
  std::vector<RigidBodyConstraint *> constraints;
  FinalPosePlanner fpp;

  VectorXd start_configuration;
  start_configuration.resize(robot.num_positions);
  start_configuration << 0, 0, 1.0250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.3002, 1.2500, 0, 0.7854, 1.5710, 0, 0, 0.3002, -1.2500, 0, -0.7854, 1.5710, 0, 0, 0, 0, -0.4900, 1.2050, -0.7100, 0, 0, 0, -0.4900, 1.2050, -0.7100, 0;

  KinematicsCache<double> cache = robot.doKinematics(start_configuration);
  IKoptions ik_options(&robot);
  VectorXd cost(robot.num_positions, 1);
  MatrixXd Q;
  cost << 10, 10, 10, 10, 10, 10, 20, 19, 18, 3, 2, 1, 3.50000000000000, 3, 2.50000000000000, 2, 1.50000000000000, 1, 0.500000000000000, 3.50000000000000, 3, 2.50000000000000, 2, 1.50000000000000, 1, 0.500000000000000, 6, 5, 4, 3, 2, 1, 6, 5, 4, 3, 2, 1;
  ik_options.setQ(cost.asDiagonal());
  ik_options.getQ(Q);
  ik_options.setMajorIterationsLimit(100);
  ik_options.setMajorOptimalityTolerance(1e-3);

  // FEET CONSTRAINTS
  int left_foot_id = robot.findLinkId("LeftFoot");
  int right_foot_id = robot.findLinkId("RightFoot");
  int world_id = robot.findLinkId("world");
  Transform<double, 3, 1> left_foot_transform = robot.relativeTransform(cache, world_id,
      left_foot_id);
  Transform<double, 3, 1> right_foot_transform = robot.relativeTransform(cache, world_id,
      right_foot_id);

  Vector3d left_des_orient_lb = rotmat2rpy(left_foot_transform.rotation());
  Vector3d left_des_orient_ub = rotmat2rpy(left_foot_transform.rotation());
  left_des_orient_lb(2) = -numeric_limits<double>::infinity();
  left_des_orient_ub(2) = numeric_limits<double>::infinity();
  Vector3d right_des_orient_lb = rotmat2rpy(right_foot_transform.rotation());
  Vector3d right_des_orient_ub = rotmat2rpy(right_foot_transform.rotation());
  right_des_orient_lb(2) = -numeric_limits<double>::infinity();
  right_des_orient_ub(2) = numeric_limits<double>::infinity();
  WorldEulerConstraint left_foot_euler_constraint(&robot, left_foot_id, left_des_orient_lb,
      left_des_orient_ub);
  WorldEulerConstraint right_foot_euler_constraint(&robot, right_foot_id, right_des_orient_lb,
      right_des_orient_ub);

  Vector3d left_position_lb = left_foot_transform.translation();
  Vector3d left_position_ub = left_foot_transform.translation();
  Vector3d right_position_lb = right_foot_transform.translation();
  Vector3d right_position_ub = right_foot_transform.translation();
  left_position_lb(0) = -numeric_limits<double>::infinity();
  left_position_lb(1) = -numeric_limits<double>::infinity();
  left_position_ub(0) = numeric_limits<double>::infinity();
  left_position_ub(1) = numeric_limits<double>::infinity();
  right_position_lb(0) = -numeric_limits<double>::infinity();
  right_position_lb(1) = -numeric_limits<double>::infinity();
  right_position_ub(0) = numeric_limits<double>::infinity();
  right_position_ub(1) = numeric_limits<double>::infinity();
  WorldPositionConstraint left_foot_pos_constraint(&robot, left_foot_id, Vector3d(0, 0, 0),
      left_position_lb, left_position_ub);
  WorldPositionConstraint right_foot_pos_constraint(&robot, right_foot_id, Vector3d(0, 0, 0),
      right_position_lb, right_position_ub);

  Vector3d relative_distance = left_foot_transform.translation()
      - right_foot_transform.translation();
  VectorXd bTbp(7, 1);
  bTbp << 0, 0, 0, 1, 0, 0, 0;
  Vector3d bound(1e-3, 1e-3, 1e-3);
  RelativePositionConstraint relative_pos_constraint(&robot, Vector3d(0, 0, 0),
      relative_distance - bound, relative_distance + bound, left_foot_id, right_foot_id, bTbp,
      Vector2d(0, 1));
  constraints.push_back(&left_foot_pos_constraint);
  constraints.push_back(&right_foot_pos_constraint);
  constraints.push_back(&left_foot_euler_constraint);
  constraints.push_back(&right_foot_euler_constraint);
  constraints.push_back(&relative_pos_constraint);

  // QUASI-STATIC CONSTRAINT
  Matrix3Xd left_contact_points;
  Matrix3Xd right_contact_points;
  robot.getTerrainContactPoints(*(robot.bodies[left_foot_id]), left_contact_points);
  robot.getTerrainContactPoints(*(robot.bodies[right_foot_id]), right_contact_points);
  QuasiStaticConstraint quasi_static_constraint(&robot);
  quasi_static_constraint.setShrinkFactor(0.5);
  quasi_static_constraint.setActive(true);
  quasi_static_constraint.addContact(1, &left_foot_id, &left_contact_points);
  quasi_static_constraint.addContact(1, &right_foot_id, &right_contact_points);
  constraints.push_back(&quasi_static_constraint);

  VectorXd nominal_configuration, final_configuration;
  nominal_configuration.resize(robot.num_positions);
  nominal_configuration << 0, 0, 1.0250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.3002, 1.2500, 0, 0.7854, 1.5710, 0, 0, 0.3002, -1.2500, 0, -0.7854, 1.5710, 0, 0, 0, 0, -0.4900, 1.2050, -0.7100, 0, 0, 0, -0.4900, 1.2050, -0.7100, 0;


  std::cout << "Load scene\n";
  ss.str("");
  ss << getenv("DRC_BASE") << "/../drc-testing-data/final_pose_planner/scene1.bin";
  string point_cloud_file = ss.str();
  int info = 0;

  ifstream inputFile(point_cloud_file.c_str(), ifstream::binary);

  if (!inputFile.is_open())
  {
    std::cout << "Failed to open " << point_cloud_file.c_str() << '\n';
    throw runtime_error("test-FPP failed");
  } else
  {
    vector<Vector3d> point_cloud;
    unsigned int n_points;
    inputFile.read((char *) &n_points, sizeof(unsigned int));
    point_cloud.resize(n_points);
    for (int point = 0; point < n_points; point++)
    {
      inputFile.read((char *) point_cloud[point].data(), 3 * sizeof(Vector3d::Scalar));
    }
    inputFile.close();

    cm.setActiveSide("left");

    std::cout << "Starting iteration\n";
    int n_iter = 0;
    while (n_iter < 10 && info != 1)
    {
      FPPOutput output;
      info = fpp.findFinalPose(robot, endeffector_name, "left", start_configuration,
          final_configuration, endeffector_pose, constraints, nominal_configuration, cm, point_cloud,
          ik_options, theLCM, output, endeffector_point);
      n_iter++;
    }
  }
  std::cout <<  info << "\n";
  if (info != 1) {throw runtime_error("test-FPP failed");}
  return 0;
}
