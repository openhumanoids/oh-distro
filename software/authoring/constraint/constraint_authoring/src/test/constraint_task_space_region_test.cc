#include <iostream>
#include "authoring/constraint_task_space_region.h"

using namespace std;
using namespace drc;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Constraint_Task_Space_Region class demo program" << endl;
  Constraint_Task_Space_Region constraint_task_space_region;
  cout << "constraint_task_space_region: " << constraint_task_space_region << endl;
  action_sequence_t action_sequence;
  action_sequence.num_contact_goals = 0;
  constraint_task_space_region.add_to_drc_action_sequence_t( action_sequence );
  cout << "utime: " << action_sequence.utime << endl;
  cout << "robot_name: " << action_sequence.robot_name << endl;
  cout << "num_contact_goals: " << action_sequence.num_contact_goals << endl;
  for( vector< contact_goal_t >::iterator it = action_sequence.contact_goals.begin(); it != action_sequence.contact_goals.end(); it++ ){
    cout << "  object_1_name: " << it->object_1_name << endl; 
    cout << "  object_1_contact_grp: " << it->object_1_contact_grp << endl; 
    cout << "  object_2_name: " << it->object_2_name << endl; 
    cout << "  object_2_contact_grp: " << it->object_2_contact_grp << endl; 
    cout << "  lower_bound_completion_time: " << it->lower_bound_completion_time << endl;
    cout << "  upper_bound_completion_time: " << it->upper_bound_completion_time << endl;
    cout << "  contact_type: " << it->contact_type << endl;
  }
  cout << "end of Constraint_Task_Space_Region class demo program" << endl;
  return status;
}
