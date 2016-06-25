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
  typedef Matrix<double, 7, 1> endeffector_pose;
  map<string, map<int, map<string, endeffector_pose>>> endeffector_poses;
  endeffector_poses["val2"][1]["left"] << 0.8, 0, 1.0625, 0.707106781186548, 0, 0, -0.707106781186547;
  endeffector_poses["val2"][1]["right"] << 0.8, 0, 1.0625, 0.707106781186548, 0, 0, 0.707106781186547;
  endeffector_poses["val2"][2]["left"] << 0.8, 0, 1.0625, 0.707106781186548, 0, 0, -0.707106781186547;
  endeffector_poses["val2"][2]["right"] << 0.8, 0, 1.0625, 0.707106781186548, 0, 0, 0.707106781186547;
  endeffector_poses["val2"][3]["left"] << 0.7, 0, 0.6, 0.707106781186548, 0, 0, -0.707106781186547;
  endeffector_poses["val2"][3]["right"] << 0.7, 0, 0.6, 0.707106781186548, 0, 0, 0.707106781186547;
  endeffector_poses["val2"][4]["left"] << 0.5, 0, 1.0625, 0.382683432365090, 0, 0, -0.923879532511287;
  endeffector_poses["val2"][4]["right"] << 0.5, 0, 1.0625, 0.382683432365090, 0, 0, 0.923879532511287;
  endeffector_poses["val2"][5]["left"] << 0.65, 0, 1.1, 0.707106781186548, 0, 0, -0.707106781186547;
  endeffector_poses["val2"][5]["right"] << 0.65, 0, 1.1, 0.707106781186548, 0, 0, 0.707106781186547;
  endeffector_poses["val2"][6]["left"] << .5, -.2, .96, 0.707106781186548, 0, 0, -0.707106781186547;
  endeffector_poses["val2"][6]["right"] << .5, -.2, .96, 0.707106781186548, 0, 0, 0.707106781186547;
  endeffector_poses["val2"][7]["left"] << .7, -.1, .96, 0.707106781186548, 0, 0, -0.707106781186547;
  endeffector_poses["val2"][7]["right"] << .7, -.1, .96, 0.923879532511287, 0, 0, 0.382683432365090;
  endeffector_poses["val2"][8]["left"] << .7, .1, .975, 0.707106781186548, 0, 0, -0.707106781186547;
  endeffector_poses["val2"][8]["right"] << .7, .1, .975, 0.707106781186548, 0, 0, 0.707106781186547;
  endeffector_poses["val2"][9]["left"] << .5, .3, .97, 0.707106781186548, 0, 0, -0.707106781186547;
  endeffector_poses["val2"][9]["right"] << .5, .3, .97, 0.707106781186548, 0, 0, 0.707106781186547;

  map<string, map<string, Vector3d>> endeffector_points;
  endeffector_points["val2"]["left"] << 0.08, 0.07, 0;
  endeffector_points["val2"]["right"] << 0.08, -0.07, 0;

  map<string, map<string, string>> endeffector_names;
  endeffector_names["val2"]["left"] = "leftPalm";
  endeffector_names["val2"]["right"] = "rightPalm";

  int n_iter;
  vector<int> scenes;
  string grasping_hands_str;
  vector<string> models(1, "val2");
  stringstream ss;

  po::options_description hidden("parameters");
  hidden.add_options()("scenes,s", po::value<vector<int>>(&scenes)->multitoken(),
      "space separated list of integers representing the scenes to compute");

  po::positional_options_description p;
  p.add("scenes", -1);

  po::options_description desc("options");
  desc.add_options()("help,h", "Produce this message")("n_iterations,i",
      po::value<int>(&n_iter)->default_value(10), "number of iterations")("grasping_hands,g",
      po::value<string>(&grasping_hands_str)->default_value("left"),
      "hands to use in the computation. Can be \"left\", \"right\" or \"both\"")("models,m",
      po::value<vector<string>>(&models)->multitoken(),
      "space separated list of models to include in the simulation (only val2 is currently supported)");

  po::options_description all_options("all options");
  all_options.add(hidden).add(desc);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(all_options).positional(p).run(), vm);
  po::notify(vm);

  if (vm.count("help") || scenes.size() == 0)
  {
    cout << "Usage: " << endl << argv[0] << " <scenes> [OPTIONS]" << endl << endl;
    cout << "scenes is a space separated list of integers representing the scenes to compute"
        << endl << endl;
    cout << desc << endl;
    return 1;
  }

  vector<string> grasping_hands;
  if (grasping_hands_str.compare("both") == 0)
  {
    cout << grasping_hands_str << endl;
    grasping_hands.push_back("left");
    grasping_hands.push_back("right");
  } else if (grasping_hands_str.compare("left") == 0 || grasping_hands_str.compare("right") == 0)
  {
    grasping_hands.push_back(grasping_hands_str);
  } else
  {
    cout << "grasping hands can be \"left\", \"right\" or \"both\"" << endl;
    return 1;
  }

  for (const auto &m : models)
  {
    if (m.compare("val2") != 0)
    {
      cout << "only val2 is currently supported. You don't need to specify the model option"
          << endl;
      return 1;
    }
  }

  cout << "n iterations: " << n_iter << endl;
  cout << "scenes: ";
  for (const auto &s : scenes)
  {
    cout << s << ";";
  }
  cout << endl;
  cout << "grasping hands: ";
  for (const auto &h : grasping_hands)
  {
    cout << h << ";";
  }
  cout << endl;
  cout << "models: ";
  for (const auto &m : models)
  {
    cout << m << ";";
  }
  cout << endl;

  ss.str("");
  ss << getenv("DRC_BASE") << "/software/planning/capabilityMap.log";
  CapabilityMap cm(ss.str());
  ss.str("");
  ss << getenv("DRC_BASE") << "/../drc-testing-data/final_pose_planner/val_description/CapabilityMapMatlab.bin";
  if (FILE *file = fopen(ss.str().c_str(), "r"))
  {
    fclose(file);
  }
  else
  {
    string path = ss.str();
    ss.str("");
    ss << "ERROR: No capability map found. Please download it from http://terminator.robots.inf.ed.ac.uk/public/final_pose_planner/CapabilityMapMatlab.bin and save it to " << path;
    throw runtime_error(ss.str());
  }
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

//	FEET CONSTRAINTS
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

  VectorXd nominal_configuration, final_configuration;
  nominal_configuration.resize(robot.num_positions);
  nominal_configuration << 0, 0, 1.0250, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.3002, 1.2500, 0, 0.7854, 1.5710, 0, 0, 0.3002, -1.2500, 0, -0.7854, 1.5710, 0, 0, 0, 0, -0.4900, 1.2050, -0.7100, 0, 0, 0, -0.4900, 1.2050, -0.7100, 0;

//  INITIALIZE OUTPUT FILE
  string output_file_name = "output.fpp";
  XMLDocument xml_doc;
  XMLDeclaration *declaration_node = xml_doc.NewDeclaration();
  xml_doc.LinkEndChild(declaration_node);
  XMLElement *results_node = xml_doc.NewElement("results");
  xml_doc.LinkEndChild(results_node);

  XMLElement *details_node = xml_doc.NewElement("details");
  results_node->LinkEndChild(details_node);

  XMLElement *models_node = xml_doc.NewElement("models");
  details_node->LinkEndChild(models_node);
  for (const auto &m : models)
  {
    addTextToElement(models_node, m);
  }

  XMLElement *n_iterations_node = xml_doc.NewElement("n_iterations");
  details_node->LinkEndChild(n_iterations_node);
  n_iterations_node->SetText(n_iter);

  XMLElement *scenes_node = xml_doc.NewElement("scenes");
  details_node->LinkEndChild(scenes_node);
  for (const auto &s : scenes)
  {
    ss.str("");
    ss << "scene" << s;
    addTextToElement(scenes_node, ss.str().c_str());
  }

  XMLElement *grasping_hands_node = xml_doc.NewElement("grasping_hands");
  details_node->LinkEndChild(grasping_hands_node);
  for (const auto &h : grasping_hands)
  {
    addTextToElement(grasping_hands_node, h);
  }

  time_t rawtime;
  char buffer[80];
  time(&rawtime);
  struct tm *timeinfo = localtime(&rawtime);
  strftime(buffer, 80, "%d-%b-%Y %T", timeinfo);
  XMLElement *time_node = xml_doc.NewElement("time");
  details_node->LinkEndChild(time_node);
  time_node->SetText(buffer);

  //results
  for (const auto &m : models)
  {
    for (const auto &s : scenes)
    {
      ss.str("");
      if (s < 6)
      {
        ss << getenv("DRC_BASE") << "/../drc-testing-data/final_pose_planner/scene" << s << ".bin";
      } else
      {
        ss << getenv("DRC_BASE") << "/../drc-testing-data/final_pose_planner/scene6.bin";
      }
      string point_cloud_file = ss.str();

      ifstream inputFile(point_cloud_file.c_str(), ifstream::binary);

      if (!inputFile.is_open())
      {
        std::cout << "Failed to open " << point_cloud_file.c_str() << '\n';
        return 1;
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

        bot_lcmgl_t* lcmgl_pc = bot_lcmgl_init(theLCM->getUnderlyingLCM(), "Point Cloud");
        drawPointCloud(lcmgl_pc, point_cloud);

        for (const auto &h : grasping_hands)
        {
          XMLElement *iteration_set_node = xml_doc.NewElement("iteration_set");
          iteration_set_node->SetAttribute("model", m.c_str());
          ss.str("");
          ss << "scene" << s;
          iteration_set_node->SetAttribute("scene", ss.str().c_str());
          iteration_set_node->SetAttribute("hand", h.c_str());
          results_node->LinkEndChild(iteration_set_node);

          XMLElement *computation_time_node = xml_doc.NewElement("computation_time");
          computation_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(computation_time_node);

          XMLElement *IK_time_node = xml_doc.NewElement("IK_time");
          IK_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(IK_time_node);

          XMLElement *CM_time_node = xml_doc.NewElement("capability_map_time");
          CM_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(CM_time_node);

          XMLElement *collision_time_node = xml_doc.NewElement("collision_time");
          collision_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(collision_time_node);

          XMLElement *constraints_time_node = xml_doc.NewElement("constraints_time");
          constraints_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(constraints_time_node);

          XMLElement *kin_time_node = xml_doc.NewElement("kin_time");
          kin_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(kin_time_node);

          XMLElement *sampling_time_node = xml_doc.NewElement("sampling_time");
          sampling_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(sampling_time_node);

          XMLElement *info_node = xml_doc.NewElement("info");
          info_node->SetAttribute("format", "%d");
          iteration_set_node->LinkEndChild(info_node);

          XMLElement *n_samples_node = xml_doc.NewElement("n_valid_samples");
          n_samples_node->SetAttribute("format", "%d");
          iteration_set_node->LinkEndChild(n_samples_node);

          XMLElement *n_used_samples_node = xml_doc.NewElement("n_valid_samples_used");
          n_used_samples_node->SetAttribute("format", "%d");
          iteration_set_node->LinkEndChild(n_used_samples_node);

          XMLElement *cost_node = xml_doc.NewElement("cost");
          cost_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(cost_node);

          XMLElement *cm_angle_time_node = xml_doc.NewElement("cm_angle_time");
          cm_angle_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(cm_angle_time_node);

          XMLElement *cm_base_hight_time_node = xml_doc.NewElement("cm_base_hight_time");
          cm_base_hight_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(cm_base_hight_time_node);

          XMLElement *cm_direction_time_node = xml_doc.NewElement("cm_direction_time");
          cm_direction_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(cm_direction_time_node);

          XMLElement *cm_collision_time_node = xml_doc.NewElement("cm_collision_time");
          cm_collision_time_node->SetAttribute("format", "%g");
          iteration_set_node->LinkEndChild(cm_collision_time_node);

          cm.setActiveSide(h);
          Vector3d endeffector_point = endeffector_points[m][h];
          VectorXd endeffector_final_pose = endeffector_poses[m][s][h];

          int info;

          for (int i = 0; i < n_iter; i++)
          {
            FPPOutput output;
            cout << "Model: " << m << " Scene: " << s << " Hand: " << h << " Iteration: " << i + 1
                << endl;
            info = fpp.findFinalPose(robot, endeffector_names[m][h], h, start_configuration,
                final_configuration, endeffector_final_pose, constraints, nominal_configuration, cm,
                point_cloud, ik_options, theLCM, output, endeffector_point);
            addTextToElement(info_node, info);
            addTextToElement(computation_time_node, output.computation_time);
            addTextToElement(IK_time_node, output.IK_time);
            addTextToElement(CM_time_node, output.capability_map_time);
            addTextToElement(collision_time_node, output.collision_time);
            addTextToElement(constraints_time_node, output.constraints_time);
            addTextToElement(kin_time_node, output.kinematics_time);
            addTextToElement(sampling_time_node, output.sampling_time);
            addTextToElement(n_samples_node, output.n_valid_samples);
            addTextToElement(n_used_samples_node, output.n_valid_samples_used);
            addTextToElement(cost_node, output.cost);
            addTextToElement(cm_angle_time_node, output.cm_angle_time);
            addTextToElement(cm_base_hight_time_node, output.cm_base_hight_time);
            addTextToElement(cm_direction_time_node, output.cm_direction_time);
            addTextToElement(cm_collision_time_node, output.cm_collision_time);
          }
        }
      }
    }
    xml_doc.SaveFile(output_file_name.c_str());
  }
  return 0;
}
