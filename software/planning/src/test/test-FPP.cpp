#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include "bot_lcmgl_client/lcmgl.h"

#include "capabilityMap/CapabilityMap.hpp"
#include "finalPosePlanner/FinalPosePlanner.hpp"
#include "drawingUtil/drawingUtil.hpp"

using namespace std;
using namespace Eigen;

int main()
{
	CapabilityMap cm("/home/marco/oh-distro/software/planning/capabilityMap.log");
	cm.loadFromMatlabBinFile("/home/marco/drc-testing-data/final_pose_planner/val_description/eigenexport_occ.bin");

	boost::shared_ptr<lcm::LCM> theLCM(new lcm::LCM);
	if(!theLCM->good()){
		std::cerr <<"ERROR: lcm is not good()" <<std::endl;
	}

	RigidBodyTree robot("/home/marco/oh-distro/software/models/val_description/urdf/valkyrie_sim_drake.urdf");
	std::vector<RigidBodyConstraint *> constraints;
	FinalPosePlanner fpp;

	VectorXd start_configuration;
	start_configuration.resize(robot.num_positions);
	start_configuration <<	0, 0, 1.0250, 0, 0 ,0 ,0, 0 ,0 ,0 ,0 ,0 ,0.3002,1.2500, 0, 0.7854, 1.5710 ,0, 0 ,0.3002, -1.2500,
			0, -0.7854, 1.5710 ,0, 0, 0, 0, -0.4900, 1.2050 ,-0.7100, 0 ,0, 0, -0.4900 ,1.2050, -0.7100 ,0;

	KinematicsCache<double> cache = robot.doKinematics(start_configuration);

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

		fpp.findFinalPose(robot, "leftPalm", "left", start_configuration, endeffector_final_pose, constraints, nominal_configuration , cm, point_cloud, IKoptions(&robot), theLCM, output, 0.005, endeffector_point);
//		bot_lcmgl_t* lcmgl = bot_lcmgl_init(theLCM->getUnderlyingLCM(), "Capability map");
//		cm.drawActiveMap(lcmgl, 52, Vector3d(0,0,0), false);
	}

//	bot_lcmgl_t* lcmglOM = bot_lcmgl_init(theLCM->getUnderlyingLCM(), "Occupancy map");
//	cm.drawOccupancyMap(lcmglOM, 16001, 52);

	return 0;
}
