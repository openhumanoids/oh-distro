#include "finalPosePlanner/FinalPosePlanner.hpp"

#include <memory>
#include <chrono>

#include "drake/util/drakeGeometryUtil.h"
#include "drawingUtil/drawingUtil.hpp"
#include "bot_lcmgl_client/lcmgl.h"

using namespace std;
using namespace Eigen;

FinalPosePlanner::FinalPosePlanner()
{

}

int FinalPosePlanner::findFinalPose(RigidBodyTree &robot, string end_effector, string endeffector_side, VectorXd start_configuration,
		VectorXd endeffector_final_pose, const vector<RigidBodyConstraint *> &additional_constraints, VectorXd nominal_configuration,
		CapabilityMap &capability_map, vector<Vector3d> point_cloud, IKoptions ik_options, boost::shared_ptr<lcm::LCM> lcm, FPPOutput &output, Vector3d endeffector_point, int max_iterations,  double min_distance){

//	timing variables
    FPPTimer FPP_timer, IK_timer, CM_timer, collision_timer, constraints_timer, kin_timer, sampling_timer;
    FPP_timer.start();
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
	CM_timer.start();
	capability_map.setEndeffectorPose(endeffector_final_pose);
	capability_map.setActiveSide(endeffector_side);
	capability_map.reduceActiveSet(true, point_cloud, output);
	output.n_valid_samples = capability_map.getNActiveOrientations();
	capability_map.computeOrientationProbabilityDistribution();
	capability_map.computePositionProbabilityDistribution(capability_map.getMapCentre());
	CM_timer.stop();


//	FINAL POSE SEARCH
	CandidateRobotPosePublisher publisher;
	constraints_timer.start();
	vector<RigidBodyConstraint *> constraints = additional_constraints;
	Vector3d bound(1e-3, 1e-3, 1e-3);
	WorldPositionConstraint position_constraint(&robot, endeffector_id, endeffector_point, endeffector_final_pose.block<3,1>(0,0)-bound, endeffector_final_pose.block<3,1>(0,0) + bound);
	WorldQuatConstraint quaternion_constraint(&robot, endeffector_id, endeffector_final_pose.block<4,1>(3,0), 1./180.*M_PI);
	constraints.push_back(&position_constraint);
	constraints.push_back(&quaternion_constraint);
	constraints.resize(constraints.size() + 2);
	constraints_timer.stop();

	VectorXd final_configuration(robot.num_positions);
	vector<string> infeasible_constraints;
    VectorXd phi;
    Matrix3Xd normal, xA, xB;
    vector<int> bodyA_idx, bodyB_idx;
	int info = 14;
	int ik_info;
	int sample_info;
	int n_iter = 0;
	vector<int> sample(2);
	while (info != 1 && n_iter <= max_iterations)
	{
		n_iter++;
		sampling_timer.start();
		sample_info = capability_map.drawCapabilityMapSample(sample);
		if (sample_info != 1)
		{
			cout << "Error: FinalPosePlanner::No more sample to use" << endl;
			return sample_info;
		}
		sampling_timer.stop();
//		GENERATE CONSTRAINTS
		constraints_timer.start();
		int base_id = robot.findLinkId(capability_map.getBaseLink());
		Vector3d orientation = capability_map.getOrientation(sample[1]);
		Vector3d position = rpy2rotmat(orientation) * capability_map.getVoxelCentre(sample[0]) + endeffector_final_pose.block<3,1>(0,0);
		cout << capability_map.getMapCentre() << endl << orientation << endl;
		WorldPositionConstraint base_position_constraint(&robot, base_id, capability_map.getMapCentre(), position, position);
		WorldEulerConstraint base_euler_constraint(&robot, base_id, orientation, orientation);
		constraints.end()[-1] = (&base_position_constraint);
		constraints.end()[-2] = (&base_euler_constraint);
		constraints_timer.stop();

//		COMPUTE CONFIGURATION
		IK_timer.start();
		inverseKin(&robot, nominal_configuration, nominal_configuration, constraints.size(), constraints.data(), final_configuration, ik_info, infeasible_constraints, ik_options);
		IK_timer.stop();
		publisher.publish(lcm, robot, final_configuration);
		vector<string> name;
		VectorXd lb;
		VectorXd ub;
//		double time = 0.;
//		for (auto constraint : constraints)
//		{
//			if(constraint->getCategory() == constraint->SingleTimeKinematicConstraintCategory)
//			{
//				name.clear();
//				((SingleTimeKinematicConstraint*)constraint)->name(&time, name);
//				((SingleTimeKinematicConstraint*)constraint)->bounds(&time, lb, ub);
//
//				for (auto n : name)
//				{
//					cout << n << endl;
//				}
//				cout << lb << endl << ub << endl;
//			}
//			else if(constraint->getCategory() == constraint->QuasiStaticConstraintCategory)
//			{
//				name.clear();
//				((QuasiStaticConstraint*)constraint)->name(&time, name);
//
//				for (auto n : name)
//				{
//					cout << n << endl;
//				}
//			}
//		}
		if (ik_info < 10)
		{
			kin_timer.start();
			KinematicsCache<double> kinsol = robot.doKinematics(final_configuration);
			kin_timer.stop();
			collision_timer.start();
			bool is_valid = !robot.collidingPointsCheckOnly(kinsol, point_cloud, min_distance);
			if (is_valid)
			{
				robot.collisionDetect(kinsol, phi, normal, xA, xB, bodyA_idx, bodyB_idx, false);
				collision_timer.stop();
				if (((ArrayXd)phi > min_distance).all())
				{
					info = 1;
//					capability_map.log << final_configuration << endl;
					publisher.publish(lcm, robot, final_configuration);
				}
			}
		}
	}
	FPP_timer.stop();
    output.n_valid_samples_used = n_iter;
    output.IK_time = IK_timer.getDuration();
    output.computation_time = FPP_timer.getDuration();
    output.capability_map_time = CM_timer.getDuration();
    output.collision_time = collision_timer.getDuration();
    output.constraints_time = constraints_timer.getDuration();
    output.kinematics_time = kin_timer.getDuration();
    output.sampling_time = sampling_timer.getDuration();
    if(info == 1)
    {
    	cout << "Solution found in " << output.computation_time << " s" << endl;
    	MatrixXd cost_matrix;
    	ik_options.getQ(cost_matrix);
    	output.cost = (nominal_configuration - final_configuration).transpose() * cost_matrix * (nominal_configuration - final_configuration);
    }
    else
    {
		cout << "Error: FinalPosePlanner::Iteration limit reached" << endl;
		output.cost = numeric_limits<double>::infinity();
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
