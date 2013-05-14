#include <iostream>
#include "authoring/constraint_sequence.h"

using namespace std;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Constraint_Sequence class demo program" << endl;
  Constraint_Sequence constraint_sequence;
  cout << "constraint_sequence: " << constraint_sequence << endl;
  cout << "end of Constraint_Sequence class demo program" << endl;
  return status;
}
