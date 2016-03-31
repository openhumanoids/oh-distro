#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <random>
#include <stdio.h>
#include <cstdio>

#include "bot_lcmgl_client/lcmgl.h"

#include "capabilityMap/CapabilityMap.hpp"
#include "finalPosePlanner/FinalPosePlanner.hpp"
#include "drawingUtil/drawingUtil.hpp"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"

extern "C" {
double *mt19937ar();
}

using namespace std;
using namespace Eigen;

int main()
{
//	typedef mersenne_twister_engine< uint32_t, 32, 625, 397, 31, 0x9908b0df, 11, 0xffffffff, 7, 0x9d2c5680, 15, 0xefc60000, 18, 1812433253 > mt;
//	mt g(11);
//	cout << g.max() << endl;
//	double* random_sequence = mt19937ar();
//	for (int i = 0; i < 10; i++)
//	{
//		cout << random_sequence[i] << endl;
//	}
	CapabilityMap cm("/home/marco/oh-distro/software/planning/capabilityMap.log");
	cm.loadFromMatlabBinFile("/home/marco/drc-testing-data/final_pose_planner/val_description/CapabilityMapMatlab.bin");
//
	boost::shared_ptr<lcm::LCM> theLCM(new lcm::LCM);
	if(!theLCM->good()){
		std::cerr <<"ERROR: lcm is not good()" <<std::endl;
	}

	RigidBodyTree robot("/home/marco/oh-distro/software/models/val_description/urdf/valkyrie_sim_drake.urdf");
//	FinalPosePlanner fpp;

	CandidateRobotPosePublisher publisher;
	VectorXd nominal_configuration;
	nominal_configuration.resize(robot.num_positions);
	nominal_configuration <<	0, 0, 1.0250, 0, 0 ,0 ,0, 0 ,0 ,0 ,0 ,0 ,0.3002,1.2500, 0, 0.7854, 1.5710 ,0, 0 ,0.3002, -1.2500,
			0, -0.7854, 1.5710 ,0, 0, 0, 0, -0.4900, 1.2050 ,-0.7100, 0 ,0, 0, -0.4900 ,1.2050, -0.7100 ,0;

	KinematicsCache<double> cache = robot.doKinematics(nominal_configuration);
	IKoptions ik_options(&robot);
	VectorXd cost(robot.num_positions, 1);
	MatrixXd Q;
	cost << 10,10,10,10,10,10,20,19,18,3,2,1,3.50000000000000,3,2.50000000000000,2,1.50000000000000,1,0.500000000000000,3.50000000000000,3,2.50000000000000,2,1.50000000000000,1,0.500000000000000,6,5,4,3,2,1,6,5,4,3,2,1;
	ik_options.setQ(cost.asDiagonal());
	ik_options.getQ(Q);
	ik_options.setMajorIterationsLimit(100);
	ik_options.setMajorOptimalityTolerance(1e-3);

	std::vector<RigidBodyConstraint *> constraints;

	int left_foot_id = robot.findLinkId("LeftFoot");
	int right_foot_id = robot.findLinkId("RightFoot");
	int world_id = robot.findLinkId("world");
	int endeffector_id = robot.findLinkId("leftPalm");
	Vector3d endeffector_point(3);
	endeffector_point << 0.08, 0.07, 0;
	VectorXd endeffector_final_pose(7);
	endeffector_final_pose << 0.8, 0, 1.0625, 0.707106781186548, 0, 0, -0.707106781186547;
	Vector3d bound(1e-3, 1e-3, 1e-3);
	WorldPositionConstraint position_constraint(&robot, endeffector_id, endeffector_point, endeffector_final_pose.block<3,1>(0,0)-bound, endeffector_final_pose.block<3,1>(0,0) + bound);
	WorldQuatConstraint quaternion_constraint(&robot, endeffector_id, endeffector_final_pose.block<4,1>(3,0), 1./180.*M_PI);
	constraints.push_back(&position_constraint);
	constraints.push_back(&quaternion_constraint);

	Transform<double, 3, 1> left_foot_transform = robot.relativeTransform(cache, world_id, left_foot_id);
	Transform<double, 3, 1> right_foot_transform = robot.relativeTransform(cache, world_id, right_foot_id);

	Vector3d left_des_orient_lb = rotmat2rpy(left_foot_transform.rotation());
	Vector3d left_des_orient_ub = rotmat2rpy(left_foot_transform.rotation());
	left_des_orient_lb(2) = -numeric_limits<double>::infinity();
	left_des_orient_ub(2) = numeric_limits<double>::infinity();
	Vector3d right_des_orient_lb = rotmat2rpy(right_foot_transform.rotation());
	Vector3d right_des_orient_ub = rotmat2rpy(right_foot_transform.rotation());
	right_des_orient_lb(2) = -numeric_limits<double>::infinity();
	right_des_orient_ub(2) = numeric_limits<double>::infinity();
	WorldEulerConstraint left_foot_euler_constraint(&robot, left_foot_id, left_des_orient_lb, left_des_orient_ub);
	WorldEulerConstraint right_foot_euler_constraint(&robot, right_foot_id, right_des_orient_lb, right_des_orient_ub);

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
	WorldPositionConstraint left_foot_pos_constraint(&robot, left_foot_id, Vector3d(0,0,0), left_position_lb, left_position_ub);
	WorldPositionConstraint right_foot_pos_constraint(&robot, right_foot_id, Vector3d(0,0,0), right_position_lb, right_position_ub);

	Vector3d relative_distance = left_foot_transform.translation() - right_foot_transform.translation();
	VectorXd bTbp(7,1);
	bTbp << 0,0,0,1,0,0,0;
	RelativePositionConstraint relative_pos_constraint(&robot, Vector3d(0,0,0), relative_distance-bound, relative_distance+bound, left_foot_id, right_foot_id, bTbp, Vector2d(0,1));
	constraints.push_back(&left_foot_pos_constraint);
	constraints.push_back(&right_foot_pos_constraint);
	constraints.push_back(&left_foot_euler_constraint);
	constraints.push_back(&right_foot_euler_constraint);
	constraints.push_back(&relative_pos_constraint);

//	QUASI-STATIC CONSTRAINT
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

	Vector3d orientation(0, -0.08726646259971647390241145103573217056692, -0.5235987755982989266811955531011335551739);
	Vector3d position(0.3069465410512374758944531549786916002631, 0.1403269799515377558307704930484760552645, 1.295054564170092348263096937444061040878);
	int base_id = robot.findLinkId(cm.getBaseLink());
	WorldPositionConstraint base_position_constraint(&robot, base_id, Vector3d(-0.0316, 0.2499, 0.2984), position, position);
	WorldEulerConstraint base_euler_constraint(&robot, base_id, orientation, orientation);
	constraints.push_back(&base_position_constraint);
	constraints.push_back(&base_euler_constraint);


	VectorXd final_configuration(robot.num_positions);
	vector<string> infeasible_constraints;
	int ik_info;
	vector<string> name;
	double time = 0.;
	VectorXd lb;
	VectorXd ub;
	for (auto constraint : constraints)
	{
		if(constraint->getCategory() == constraint->SingleTimeKinematicConstraintCategory)
		{
			name.clear();
			((SingleTimeKinematicConstraint*)constraint)->name(&time, name);
			((SingleTimeKinematicConstraint*)constraint)->bounds(&time, lb, ub);

			for (auto n : name)
			{
				cout << n << endl;
			}
			cout << lb << endl << ub << endl;
		}
		else if(constraint->getCategory() == constraint->QuasiStaticConstraintCategory)
		{
			name.clear();
			((QuasiStaticConstraint*)constraint)->name(&time, name);

			for (auto n : name)
			{
				cout << n << endl;
			}
		}
	}

	inverseKin(&robot, nominal_configuration, nominal_configuration, constraints.size(), constraints.data(), final_configuration, ik_info, infeasible_constraints, ik_options);
	cout << ik_info << endl;
	for (auto constraint : infeasible_constraints)
	{
		cout << constraint << endl;
	}

	publisher.publish(theLCM, robot, final_configuration);
	IOFormat fmt(20);
	cm.log << final_configuration.format(fmt) << endl;
	cm.log << robot.joint_limit_min.format(fmt) << endl;
	cm.log << robot.joint_limit_max.format(fmt) << endl;
	MatrixXd mat;
	ik_options.getQv(mat);
	cout << "MajorOptimalityTolerance: " << ik_options.getMajorOptimalityTolerance() << endl;
	cout << "MajorFeasibilityTolerance: " << ik_options.getMajorFeasibilityTolerance() << endl;
	cout << "SuperbasicsLimit: " << ik_options.getSuperbasicsLimit() << endl;
	cout << "MajorIterationsLimit: " << ik_options.getMajorIterationsLimit() << endl;
	cout << "IterationsLimit: " << ik_options.getIterationsLimit() << endl;

//	for (int i = 0; i < start_configuration.size(); i++)
//	{
//		cout << robot.joint_limit_min(i) << "\t"<< start_configuration(i) << "\t" << robot.joint_limit_max(i) << "\t" << (start_configuration(i) > robot.joint_limit_min(i) && start_configuration(i) < robot.joint_limit_max(i)) << endl;
//	}
	/*

//	FEET CONSTRAINTS
	int left_foot_id = robot.findLinkId("LeftFoot");
	int right_foot_id = robot.findLinkId("RightFoot");
	int world_id = robot.findLinkId("world");
	Transform<double, 3, 1> left_foot_transform = robot.relativeTransform(cache, world_id, left_foot_id);
	Transform<double, 3, 1> right_foot_transform = robot.relativeTransform(cache, world_id, right_foot_id);

	Vector3d left_des_orient_lb = rotmat2rpy(left_foot_transform.rotation());
	Vector3d left_des_orient_ub = rotmat2rpy(left_foot_transform.rotation());
	left_des_orient_lb(2) = -numeric_limits<double>::infinity();
	left_des_orient_ub(2) = numeric_limits<double>::infinity();
	Vector3d right_des_orient_lb = rotmat2rpy(right_foot_transform.rotation());
	Vector3d right_des_orient_ub = rotmat2rpy(right_foot_transform.rotation());
	right_des_orient_lb(2) = -numeric_limits<double>::infinity();
	right_des_orient_ub(2) = numeric_limits<double>::infinity();
	WorldEulerConstraint left_foot_euler_constraint(&robot, left_foot_id, left_des_orient_lb, left_des_orient_ub);
	WorldEulerConstraint right_foot_euler_constraint(&robot, right_foot_id, right_des_orient_lb, right_des_orient_ub);

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
	WorldPositionConstraint left_foot_pos_constraint(&robot, left_foot_id, Vector3d(0,0,0), left_position_lb, left_position_ub);
	WorldPositionConstraint right_foot_pos_constraint(&robot, right_foot_id, Vector3d(0,0,0), right_position_lb, right_position_ub);

	Vector3d relative_distance = left_foot_transform.translation() - right_foot_transform.translation();
	VectorXd bTbp(7,1);
	bTbp << 0,0,0,1,0,0,0;
	RelativePositionConstraint relative_pos_constraint(&robot, Vector3d(0,0,0), relative_distance, relative_distance, left_foot_id, right_foot_id, bTbp, Vector2d(0,1));
	constraints.push_back(&left_foot_pos_constraint);
	constraints.push_back(&right_foot_pos_constraint);
	constraints.push_back(&left_foot_euler_constraint);
	constraints.push_back(&right_foot_euler_constraint);
	constraints.push_back(&relative_pos_constraint);

	VectorXd endeffector_final_pose;
	endeffector_final_pose.resize(7);
	endeffector_final_pose << 0.8000, 0, 1.0625 ,0.7071 ,0 ,0 ,-0.7071;

	VectorXd nominal_configuration;
	nominal_configuration.resize(robot.num_positions);
	nominal_configuration <<	0, 0, 1.0250, 0, 0 ,0 ,0, 0 ,0 ,0 ,0 ,0 ,0.3002,1.2500, 0, 0.7854, 1.5710 ,0, 0 ,0.3002, -1.2500,
			0, -0.7854, 1.5710 ,0, 0, 0, 0, -0.4900, 1.2050 ,-0.7100, 0 ,0, 0, -0.4900 ,1.2050, -0.7100 ,0;

	string point_cloud_file = getenv("DRC_BASE");
	point_cloud_file += "/../drc-testing-data/final_pose_planner/scene1.bin";

	ifstream inputFile(point_cloud_file.c_str(), ifstream::binary);

	if (!inputFile.is_open())
	{
		std::cout << "Failed to open " << point_cloud_file.c_str() << '\n';
	}
	else
	{
		vector<Vector3d> point_cloud;
		unsigned int n_points;
		inputFile.read((char *) &n_points, sizeof(unsigned int));
		point_cloud.resize(n_points);
		for (int point = 0; point < n_points; point++)
		{
			inputFile.read((char *) point_cloud[point].data(), 3* sizeof(Vector3d::Scalar));
		}
		inputFile.close();

		bot_lcmgl_t* lcmgl_pc = bot_lcmgl_init(theLCM->getUnderlyingLCM(), "Point Cloud");
		drawPointCloud(lcmgl_pc, point_cloud);

		cm.setActiveSide("left");
		Vector3d endeffector_point = Vector3d(0.08, 0.07, 0);

		FPPOutput output;

		fpp.findFinalPose(robot, "leftPalm", "left", start_configuration, endeffector_final_pose, constraints, nominal_configuration , cm, point_cloud, IKoptions(&robot), theLCM, output, endeffector_point);
//		bot_lcmgl_t* lcmgl = bot_lcmgl_init(theLCM->getUnderlyingLCM(), "Capability map");
//		cm.drawActiveMap(lcmgl, 52, Vector3d(0,0,0), false);
	}

//	bot_lcmgl_t* lcmglOM = bot_lcmgl_init(theLCM->getUnderlyingLCM(), "Occupancy map");
//	cm.drawOccupancyMap(lcmglOM, 16001, 52);
 *
 */

	return 0;
}
