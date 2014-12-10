#include <iostream>

#include <kinematics/kinematics_model_gfe.h>

#include "authoring/constraint_task_space_region.h"
#include "authoring/constraint_sequence.h"

using namespace std;
using namespace boost;
using namespace drc;
using namespace urdf;
using namespace state;
using namespace kinematics;
using namespace affordance;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Constraint_Sequence class demo program" << endl;

  vector< AffordanceState > affordance_collection;
  AffordanceState box( 1, 0,
                      KDL::Frame(KDL::Vector(0.0, 0.0, -0.05)),
                      Eigen::Vector3f(0.75, 0.75, 0.0)); //color
  box.setToBox( 100.0, 100.0, 0.01, 0, 0, KDL::Frame(KDL::Vector(0.0, 0.0, -0.01)),
                        Eigen::Vector3f(0.75, 0.75, 0.0));
  affordance_collection.push_back( box );

  Constraint_Sequence constraint_sequence_1;
  Constraint_Task_Space_Region c0( "C0", true, true, 0.0, 1.75, "1st constraint", vector< string >(), "/box0" );
  c0.parents().push_back( "l_foot-toe" );
  c0.ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].second = -1.0;
  c0.ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].second = 2.0;
  c0.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].first = false;
  c0.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].second = -3.0;
  c0.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].second = 4.0;
  c0.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].second = -5.0;
  c0.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].second = 6.0;
  cout << "c0: " << c0 << endl;
  constraint_sequence_1.constraints()[0] = c0;
  Constraint_Task_Space_Region c1( "C1", true, true, 0.0, 1.25, "2nd constraint", vector< string >(), "/box0" );
  c1.ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].second = -1.0;
  c1.ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].second = 2.0;
  c1.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].second = -3.0;
  c1.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].second = 4.0;
  c1.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].second = -5.0;
  c1.ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].second = 6.0;
  cout << "c1: " << c1 << endl;
  constraint_sequence_1.constraints()[1] = c1;  

  cout << "constraint_sequence_1: " << constraint_sequence_1 << endl;
  
  action_sequence_t msg;
  constraint_sequence_1.to_msg( msg, affordance_collection );
  Constraint_Sequence::print_msg( msg );
  constraint_sequence_1.to_xml( "constraint_sequence_1.xml" );
  constraint_sequence_1.from_xml( "constraint_sequence_1.xml" );
  constraint_sequence_1.to_msg( msg, affordance_collection );
  Constraint_Sequence::print_msg( msg );

  Constraint_Sequence constraint_sequence_2;
  cout << "constraint_sequence_2: " << constraint_sequence_2 << endl;
  constraint_sequence_2.to_msg( msg, affordance_collection );
  constraint_sequence_2.to_xml( "constraint_sequence_2.xml" );
  Constraint_Sequence::print_msg( msg );

  cout << "end of Constraint_Sequence class demo program" << endl;
  return status;
}
