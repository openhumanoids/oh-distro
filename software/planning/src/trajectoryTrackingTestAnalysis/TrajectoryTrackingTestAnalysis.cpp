#include <sstream>

#include "lcm_utils/lcm_utils.hpp"
#include "lcmtypes/drc/plan_status_t.hpp"
#include "lcmtypes/drc/robot_plan_t.hpp"
#include "drake/thirdParty/tinyxml2/tinyxml2.h"

using namespace std;
using namespace lcm;
using namespace tinyxml2;

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
  if (argc < 2)
  {
    cout << "Usage:\n TrajectoryTrackingTestAnalysis <lcmlog_file>" << endl;
    return 0;
  }
  string file_name = argv[1];
  stringstream ss;
  ss.str("");
  ss << getenv("DRC_BASE") << "/software/planning/trackingTestResults";
  string output_file_name = ss.str();
  XMLDocument xml_doc;
  XMLDeclaration *declaration_node = xml_doc.NewDeclaration();
  xml_doc.LinkEndChild(declaration_node);
  XMLElement *results_node = xml_doc.NewElement("results");
  xml_doc.LinkEndChild(results_node);

  XMLElement *details_node = xml_doc.NewElement("details");
  results_node->LinkEndChild(details_node);

  time_t rawtime;
  char buffer[80];
  time(&rawtime);
  struct tm *timeinfo = localtime(&rawtime);
  strftime(buffer, 80, "%d-%b-%Y %T", timeinfo);
  XMLElement *time_node = xml_doc.NewElement("created");
  details_node->LinkEndChild(time_node);
  time_node->SetText(buffer);

  vector<drc::plan_status_t> status_msgs = lcm_utils::loadMsgsFromLog<drc::plan_status_t>(file_name, "PLAN_EXECUTION_STATUS");
  vector<drc::robot_plan_t> commited_plan_msgs = lcm_utils::loadMsgsFromLog<drc::robot_plan_t>(file_name, "COMMITTED_ROBOT_PLAN");
  vector<bot_core::robot_state_t> robot_state_msgs = lcm_utils::loadMsgsFromLog<bot_core::robot_state_t>(file_name, "EST_ROBOT_STATE");
  int test_number = 0;
  int64_t plan_start_time = -1;
  int64_t plan_end_time = 0;
  for (auto commited_plan : commited_plan_msgs)
  {
    ss.str("");
    test_number++;
    cout << "saving plan" << test_number << "..." << endl;
    ss << "test" << test_number;
    XMLElement *test_node = xml_doc.NewElement("test");
    test_node->SetAttribute("name", ss.str().c_str());
    results_node->LinkEndChild(test_node);

    XMLElement *committed_plan_node = xml_doc.NewElement("committed_plan");
    test_node->LinkEndChild(committed_plan_node);

    XMLElement *committed_plan_time_node = xml_doc.NewElement("time");
    committed_plan_node->LinkEndChild(committed_plan_time_node);

    vector<bot_core::robot_state_t> plan = commited_plan.plan;
    vector<string> committed_plan_joints = plan[0].joint_name;

    for (int joint = 0; joint < committed_plan_joints.size(); joint++)
    {
      XMLElement *position_node = xml_doc.NewElement("position");
      position_node->SetAttribute("joint_name", committed_plan_joints[joint].c_str());
      committed_plan_node->LinkEndChild(position_node);
      for (auto state : plan)
      {
        addTextToElement(position_node, state.joint_position[joint]);
      }
    }

    for (auto state : plan)
    {
      addTextToElement(committed_plan_time_node, state.utime);
    }

    XMLElement *executed_plan_node = xml_doc.NewElement("executed_plan");
    test_node->LinkEndChild(executed_plan_node);
    vector<string> executed_plan_joints = robot_state_msgs[0].joint_name;

    XMLElement *executed_plan_time_node = xml_doc.NewElement("time");
    executed_plan_node->LinkEndChild(executed_plan_time_node);

    for (auto status : status_msgs)
    {
      if (status.utime > plan_end_time && plan_end_time > plan_start_time && status.execution_status == status.EXECUTION_STATUS_EXECUTING)
      {
        plan_start_time = status.utime;
      }
      if (plan_end_time < plan_start_time && status.execution_status != status.EXECUTION_STATUS_EXECUTING)
      {
        plan_end_time = status.utime;
        break;
      }
    }

    for (int joint = 0; joint < executed_plan_joints.size(); joint++)
    {
      if (executed_plan_joints[joint].compare("hokuyo_joint") != 0)
      {
        XMLElement *position_node = xml_doc.NewElement("position");
        position_node->SetAttribute("joint_name", executed_plan_joints[joint].c_str());
        executed_plan_node->LinkEndChild(position_node);
        for (auto state : robot_state_msgs)
        {
          if (state.utime >= plan_start_time && state.utime < plan_end_time)
          {
            addTextToElement(position_node, state.joint_position[joint]);
            if (joint == 0)
            {
              addTextToElement(executed_plan_time_node, state.utime);
            }
          }
        }
      }
    }
  }
  xml_doc.SaveFile(output_file_name.c_str());
  return 0;
}


