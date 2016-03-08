#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <time.h>
#include <boost/program_options.hpp>

#include "bot_lcmgl_client/lcmgl.h"

#include "capabilityMap/CapabilityMap.hpp"
#include "finalPosePlanner/FinalPosePlanner.hpp"
#include "drawingUtil/drawingUtil.hpp"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"

using namespace std;
using namespace Eigen;
using namespace tinyxml2;
namespace po = boost::program_options;

template <typename T>
void addTextToElement(XMLElement *element, T value)
{
	stringstream ss;
	if (element->GetText()){ss << element->GetText() << " ";}
	ss << value;
	element->SetText(ss.str().c_str());
}

int main(int argc, char* argv[])
{
	int n_iter;
	vector<int> scenes;
	string grasping_hands_str;
	vector<string> models(1, "val2");

	po::options_description hidden("parameters");
	hidden.add_options()
		("scenes,s", po::value<vector<int>>(&scenes)->multitoken(), "space separated list of integers representing the scenes to compute")
	    ;

	po::positional_options_description p;
	p.add("scenes", -1);

	po::options_description desc("options");
	desc.add_options()
			("help,h", "Produce this message")
			("n_iterations,i", po::value<int>(&n_iter)->default_value(10), "number of iterations")
			("grasping_hands,g", po::value<string>(&grasping_hands_str)->default_value("left"), "hands to use in the computation. Can be \"left\", \"right\" or \"both\"")
			("models,m", po::value<vector<string>>(&models)->multitoken(), "space separated list of models to include in the simulation (only val2 is currently supported)")
			;

	po::options_description all_options("all options");
	all_options.add(hidden).add(desc);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(all_options).positional(p).run(), vm);
	po::notify(vm);

	if (vm.count("help") || scenes.size() == 0)
	{
		cout << "Usage: " << endl << argv[0] << " <scenes> [OPTIONS]" << endl << endl;
		cout << "scenes is a space separated list of integers representing the scenes to compute" << endl << endl;
		cout << desc << endl;
		return 1;
	}

	vector<string> grasping_hands;
	if (grasping_hands_str.compare("both") == 0)
	{
		cout << grasping_hands_str << endl;
		grasping_hands.push_back("left");
		grasping_hands.push_back("right");
	}
	else if (grasping_hands_str.compare("left") == 0 || grasping_hands_str.compare("right") == 0)
	{
		grasping_hands.push_back(grasping_hands_str);
	}
	else
	{
		cout << "grasping hands can be \"left\", \"right\" or \"both\"" << endl;
		return 1;
	}

	for (auto m : models)
	{
		if (m.compare("val2") != 0)
		{
			cout << "only val2 is currently supported. You don't need to specify the model option" << endl;
			return 1;
		}
	}

	cout << "n iterations: " << n_iter << endl;
	cout << "scenes: ";
	for (auto s : scenes){cout << s << ";";}
	cout << endl;
	cout << "grasping hands: ";
	for (auto h : grasping_hands){cout << h << ";";}
	cout << endl;
	cout << "models: ";
	for (auto m : models){cout << m << ";";}
	cout << endl;


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

	string point_cloud_file = "/home/marco/drc-testing-data/final_pose_planner/scene1.bin";

	ifstream inputFile(point_cloud_file.c_str(), ifstream::binary);

	if (!inputFile.is_open())
	{
		std::cout << "Failed to open " << point_cloud_file.c_str() << '\n';
		return 1;
	}
	else
	{

//        INITIALIZE OUTPUT FILE
		string output_file_name = "output.fpp";
		XMLDocument xml_doc;
		XMLElement *results_node = xml_doc.NewElement("results");
		xml_doc.LinkEndChild(results_node);

		XMLElement *details_node = xml_doc.NewElement("details");
		results_node->LinkEndChild(details_node);

		XMLElement *models_node = xml_doc.NewElement("models");
		details_node->LinkEndChild(models_node);
		for (auto m : models){addTextToElement(models_node, m);}

		XMLElement *n_iterations_node = xml_doc.NewElement("n_iterations");
		details_node->LinkEndChild(n_iterations_node);
		n_iterations_node->SetText(n_iter);

		XMLElement *scenes_node = xml_doc.NewElement("scenes");
		details_node->LinkEndChild(scenes_node);
		for (auto s : scenes){addTextToElement(scenes_node, s);}

		XMLElement *grasping_hands_node = xml_doc.NewElement("grasping_hands");
		details_node->LinkEndChild(grasping_hands_node);
		for (auto h : grasping_hands){addTextToElement(grasping_hands_node, h);}

		time_t rawtime;
		char buffer [80];
		time (&rawtime);
		struct tm *timeinfo = localtime (&rawtime);
		strftime (buffer,80,"%d-%b-%Y %T",timeinfo);
		XMLElement *time_node = xml_doc.NewElement("time");
		details_node->LinkEndChild(time_node);
		time_node->SetText(buffer);

		//results
		for (auto m : models)
		{
			XMLElement *model_node = xml_doc.NewElement("model");
			model_node->SetAttribute("name", m.c_str());
			results_node->LinkEndChild(model_node);
			for (auto s : scenes)
			{
				stringstream ss;
				ss << s;
				XMLElement *scene_node = xml_doc.NewElement("scene");
				scene_node->SetAttribute("name", ss.str().c_str());
				model_node->LinkEndChild(scene_node);

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

				for (auto h : grasping_hands)
				{
					XMLElement *hand_node = xml_doc.NewElement("hand");
					hand_node->SetAttribute("name", h.c_str());
					scene_node->LinkEndChild(hand_node);

					XMLElement *computation_time_node = xml_doc.NewElement("computation_time");
					hand_node->LinkEndChild(computation_time_node);

					XMLElement *info_node = xml_doc.NewElement("info");
					hand_node->LinkEndChild(info_node);

					cm.setActiveSide("left");
					Vector3d endeffector_point = Vector3d(0.08, 0.07, 0);

					int info;

					for (int i = 0; i < n_iter; i++)
					{
						FPPOutput output;
						cout << "Model: " << m << " Scene: " << s << " Hand: " << h << " Iteration: " << i + 1 << endl;
						info = fpp.findFinalPose(robot, "leftPalm", "left", start_configuration, endeffector_final_pose, constraints, nominal_configuration , cm, point_cloud, IKoptions(&robot), theLCM, output, 0.005, endeffector_point);
						addTextToElement(info_node, info);
						addTextToElement(computation_time_node, output.computation_time);
					}
				}
			}
		}
		xml_doc.SaveFile(output_file_name.c_str());
	}
	return 0;
}
