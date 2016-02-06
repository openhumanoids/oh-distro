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
	int findFinalPose(RigidBodyTree &robot, std::string end_effector, Eigen::VectorXd start_configuration, Eigen::VectorXd endeffector_final_pose,
			std::vector<RigidBodyConstraint> &additional_constraints, Eigen::VectorXd nominal_configuration, CapabilityMap &capability_map, IKoptions ik_options,
			double min_distance = 0.005, Eigen::Vector3d endeffector_point = Eigen::Vector3d(0,0,0)); //todo: active collision options?
	/*
	 * findFinalPose finds a suitable final configuration for reaching a given end-effector pose.
	 * @param robot   A RigidBodyTree. The manipulating robot.
	 * @param end_effector    A string containing the name of the end-effector to be used.
	 * @param start_configuration     A robot.num_positions x 1 double vector. The starting robot configuration.
	 * @param endeffector_final_pose    A 6x1(rpy-xyz) or 7x1(quat-xyz) double vector. The final end effector pose.
	 * @param additional_constraints A vector of RigidBodyConstraints. Additional constraints for the final pose.
	 * @param nominal_configuration     A robot.num_positions x 1 double vector. The nominal robot configuration to be used for IK.
	 * @param capability_map     A CapabilityMap object. The map representing robot reaching capabilities.
	 * @param ik_options     IKOptions object to be used for IK.
	 * @param endeffector_point     A 3 x 1 double vector. The position of the reaching point relative to the end-effector expressed in end-effector coordinate frame.
	 * @return q_sol    an robot.num_positions x 1 double vector. The final configuration.
	 * @return INFO     = 1   Final pose has been found.
	 *                  = 12  Fails to find a solution. Incorrect input.
	 */
private:
	int checkConfiguration(RigidBodyTree &robot, Eigen::VectorXd &configuration, std::string variable_name);
};



#endif /* FINALPOSEPLANNER_HPP_ */
