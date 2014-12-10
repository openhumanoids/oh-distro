#include <iostream>

#include <kinematics/kinematics_model_gfe.h>

#include "authoring/constraint_task_space_region.h"
#include "authoring/constraint_sequence.h"

using namespace std;
using namespace boost;
using namespace drc;
using namespace lcm;
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

  action_sequence_t msg;
  LogFile lFileReader( "../control/matlab/data/crawling_main.bin", "r" );
  const LogEvent *eventFromFile = lFileReader.readNextEvent();
  msg.decode( eventFromFile->data,0,eventFromFile->datalen );

  Constraint_Sequence constraint_sequence;
  constraint_sequence.from_msg( msg ); 
  constraint_sequence.to_xml( "crawling_main.xml" );

  cout << "end of Constraint_Sequence class demo program" << endl;
  return status;
}
