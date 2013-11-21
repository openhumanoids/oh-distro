#ifndef SHAREDUTILITIES_HPP_
#define SHAREDUTILITIES_HPP_

#include <string>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>
#include <model-client/model-client.hpp>

#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "forward_kinematics/treefksolverposfull_recursive.hpp"

#include "lcmtypes/drc_lcmtypes.hpp"

#include <leg-odometry/BotTransforms.hpp>
#include <leg-odometry/QuaternionLib.h>

// These utilities are kept as functions for now -- no direct need to make an object of these items yet

namespace TwoLegs {

// Structure to collect all bits of data need for TwoLegs::getFKTransforms
struct FK_Data {
	unsigned long long utime;
	boost::shared_ptr<ModelClient> model_;
	KDL::Tree tree;
	boost::shared_ptr<KDL::TreeFkSolverPosFull_recursive> fksolver_;
	std::vector<drc::link_transform_t> _link_tfs;
	std::map<std::string, double> jointpos_in;
	BotTransforms bottransforms;
};

class RobotModel {
 public:
   lcm::LCM lcm;
   std::string robot_name;
   std::string urdf_xml_string;
   std::vector<std::string> joint_names_;
 };

// updates left, right and body_to_head Isometry3d tranforms
void getFKTransforms(TwoLegs::FK_Data &_fk_data, Eigen::Isometry3d &left, Eigen::Isometry3d &right, Eigen::Isometry3d &body_to_head);
  
} // namespace TwoLegs

#endif /*SHAREDUTILITIES_HPP_*/
