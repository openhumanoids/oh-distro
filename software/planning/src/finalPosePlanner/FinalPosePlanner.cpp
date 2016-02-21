#include "finalPosePlanner/FinalPosePlanner.hpp"

#include <memory>

#include "drake/util/drakeGeometryUtil.h"

using namespace std;
using namespace Eigen;

FinalPosePlanner::FinalPosePlanner()
{

}

int FinalPosePlanner::findFinalPose(RigidBodyTree &robot, string end_effector, string endeffector_side, VectorXd start_configuration,
		VectorXd endeffector_final_pose, const vector<RigidBodyConstraint *> &additional_constraints, VectorXd nominal_configuration,
		CapabilityMap &capability_map, vector<Vector3d> point_cloud, IKoptions ik_options, double min_distance, Vector3d endeffector_point)
{
//	INPUT CHECKS
	int endeffector_id;
	try
	{
		endeffector_id = robot.findLinkId(end_effector);
	}
	catch (const runtime_error &)
	{
		cout << "ERROR: FinalPosePlanner::Robot has no link named " << end_effector << endl;
		return 12;
	}
	if (this->checkConfiguration(robot, start_configuration, "start_configuration") != 0) {return 12;};
	if (endeffector_final_pose.rows() == 6)
	{
		endeffector_final_pose.conservativeResize(7);
		endeffector_final_pose.block(3,0,4,1) << rpy2quat(endeffector_final_pose.block<3,1>(3,0));
	}
	if (endeffector_final_pose.rows() != 7)
	{
		cout << "ERROR: FinalPosePlanner::endeffector_final_pose must be (6x1) or (7x1). Got (" << endeffector_final_pose.size() << "x1)" << endl;
		return 12;
	}
	if (this->checkConfiguration(robot, nominal_configuration, "nominal_configuration") != 0) {return 12;};

//	CAPABILITY MAP PREPARATION
	capability_map.setEndeffectorPose(endeffector_final_pose);
	capability_map.setActiveSide(endeffector_side);
	capability_map.reduceActiveSet(true, point_cloud);
	capability_map.computeOrientationProbabilityDistribution();
	capability_map.computePositionProbabilityDistribution(capability_map.getMapCentre());

//	FINAL POSE SEARCH
	vector<RigidBodyConstraint *> constraints = additional_constraints;
	this->generateEndeffectorConstraints(robot, constraints, endeffector_id, endeffector_final_pose, endeffector_point);
	constraints.resize(constraints.size() + 2);
	VectorXd final_pose(robot.num_positions);
	vector<string> infeasible_constraints;
	int info = 13;
	while (info != 1)
	{
		vector<int> sample = capability_map.drawCapabilityMapSample();

//		GENERATE CONSTRAINTS
		vector<RigidBodyConstraint *> base_constraints;
		int base_id = robot.findLinkId(capability_map.getBaseLink());
		Vector3d orientation = capability_map.getOrientation(sample[1]);
		Vector3d position = rpy2rotmat(orientation) * capability_map.getVoxelCentre(sample[0]) + endeffector_final_pose.block<3,1>(0,0);
		WorldPositionConstraint base_position_constraint(&robot, base_id, capability_map.getMapCentre(), position, position);
		WorldEulerConstraint base_euler_constraint(&robot, base_id, orientation, orientation);
		constraints.end()[-2] = (&base_position_constraint);
		constraints.end()[-1] = (&base_euler_constraint);
		inverseKin(&robot, nominal_configuration, nominal_configuration, constraints.size(), constraints.data(), final_pose, info, infeasible_constraints, ik_options);
	}

	return info;
}

int FinalPosePlanner::checkConfiguration(const RigidBodyTree &robot, const VectorXd &configuration, string variable_name)
{
	if (configuration.rows() != robot.num_positions)
	{
		cout << "ERROR: FinalPosePlanner::" << variable_name << " does not match with DOF number" << endl;
		return 12;
	}
	for (int joint = 0; joint < configuration.size(); joint++)
	{
		if (*(configuration.data() + joint) < *(robot.joint_limit_min.data() + joint) ||
				*(configuration.data() + joint) > *(robot.joint_limit_max.data() + joint))
		{
			cout << "ERROR: FinalPosePlanner::" << variable_name << " has joints outside joint limits (Got q(" << joint << ") = " <<
					*(configuration.data() + joint) << " but it must be "<< *(robot.joint_limit_min.data() + joint) <<
					" <= q(" << joint  << ") <= " << *(robot.joint_limit_max.data() + joint) << endl;
			return 12;
		}
	}
	return 0;
}

void FinalPosePlanner::generateEndeffectorConstraints(RigidBodyTree &robot, vector<RigidBodyConstraint *> &constraint_vector, int endeffector_id,
		Matrix<double, 7, 1> endeffector_final_pose, Vector3d endeffector_point, Vector3d position_tolerance, double angular_tolerance)
{
	int world_id = robot.findLinkId("world");
	Point2PointDistanceConstraint position_constraint(&robot, endeffector_id, world_id, endeffector_point, endeffector_final_pose.block<3,1>(0,0), -position_tolerance / 2, position_tolerance / 2);
	WorldQuatConstraint quaternion_constraint(&robot, endeffector_id, endeffector_final_pose.block<4,1>(3,0), angular_tolerance);
	constraint_vector.push_back(&position_constraint);
	constraint_vector.push_back(&quaternion_constraint);
}
