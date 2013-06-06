#include <iostream>
#include "authoring/constraint.h"
#include "authoring/constraint_task_space_region.h"

using namespace std;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Constraint class demo program" << endl;
  Constraint * constraint = new Constraint_Task_Space_Region();
  cout << "constraint: " << *constraint << endl;
  if( constraint != NULL ){
    delete constraint;
    constraint = NULL;
  }
  cout << "end of Constraint class demo program" << endl;
  return status;
}
