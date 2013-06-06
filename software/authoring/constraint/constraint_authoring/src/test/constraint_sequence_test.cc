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
  AffordanceState sphere( 0, 0,
                         KDL::Frame(KDL::Vector(-0.5, -0.5, 0.0)),
                         Eigen::Vector3f(1.0, 0.0, 1.0));   //color
  sphere._params[AffordanceState::RADIUS_NAME] = 0.125;
  sphere.setType(AffordanceState::SPHERE);
  affordance_collection.push_back( sphere );

  AffordanceState box( 1, 0,
                      KDL::Frame(KDL::Vector(0.0, 0.0, -0.05)),
                      Eigen::Vector3f(0.75, 0.75, 0.0)); //color
  box.setToBox( 100.0, 100.0, 0.01, 0, 0, KDL::Frame(KDL::Vector(0.0, 0.0, -0.01)),
                        Eigen::Vector3f(0.75, 0.75, 0.0));
  affordance_collection.push_back( box );

  Constraint_Sequence constraint_sequence_1;
  Constraint_Task_Space_Region * c0 = new Constraint_Task_Space_Region( "C0", true, 0.0, 1.75, "l_foot-heel", &box );
  c0->parents().push_back( "l_foot-toe" );
  c0->ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].second = -1.0;
  c0->ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].second = 2.0;
  c0->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].first = false;
  c0->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].second = -3.0;
  c0->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].second = 4.0;
  c0->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].second = -5.0;
  c0->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].second = 6.0;
  cout << "c0: " << *c0 << endl;
  constraint_sequence_1.constraints().push_back( c0 );
  Constraint_Task_Space_Region * c1 = new Constraint_Task_Space_Region( "C1", true, 0.0, 1.25, "r_foot-heel", &sphere );
  c1->ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MIN_RANGE ].second = -1.0;
  c1->ranges()[ CONSTRAINT_TASK_SPACE_REGION_X_MAX_RANGE ].second = 2.0;
  c1->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MIN_RANGE ].second = -3.0;
  c1->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Y_MAX_RANGE ].second = 4.0;
  c1->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MIN_RANGE ].second = -5.0;
  c1->ranges()[ CONSTRAINT_TASK_SPACE_REGION_Z_MAX_RANGE ].second = 6.0;
  cout << "c1: " << *c1 << endl;
  constraint_sequence_1.constraints().push_back( c1 );  

  cout << "constraint_sequence_1: " << constraint_sequence_1 << endl;
  
  action_sequence_t msg;
  constraint_sequence_1.to_msg( msg );
  Constraint_Sequence::print_msg( msg );
  constraint_sequence_1.save( "constraint_sequence_1.bin" );
  constraint_sequence_1.to_xml( "constraint_sequence_1.xml" );

  Constraint_Sequence constraint_sequence_2;
  cout << "constraint_sequence_2: " << constraint_sequence_2 << endl;
  constraint_sequence_2.load( "constraint_sequence_1.bin", affordance_collection );
  constraint_sequence_2.to_msg( msg );
  constraint_sequence_2.to_xml( "constraint_sequence_2.xml" );
  Constraint_Sequence::print_msg( msg );

  cout << "end of Constraint_Sequence class demo program" << endl;
  return status;
}
