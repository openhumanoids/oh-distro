#ifndef FINALPOSEPLANNER_HPP_
#define FINALPOSEPLANNER_HPP_

#include <Eigen/Dense>

#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/drakeShapes_export.h"
#include "capabilityMap/capabilityMap.hpp"
#include "drake/systems/plants/IKoptions.h"

class FinalPosePlanner
{
public:
	FinalPosePlanner();
	int findFinalPose(RigidBodyTree robot, unsigned int end_effector_id, Eigen::VectorXd start_configuration, Eigen::VectorXd endeffector_final_pose,
			std::vector<RigidBodyConstraint> additional_constraints, Eigen::VectorXd nominal_configuration, CapabilityMap capability_map, IKoptions ik_options,
			std::string reaching_hand = "left", double min_distance = 0.005, Eigen::Vector3d endeffector_point = Eigen::Vector3d(0,0,0)); //todo: active collision options?
private:
};



#endif /* FINALPOSEPLANNER_HPP_ */
