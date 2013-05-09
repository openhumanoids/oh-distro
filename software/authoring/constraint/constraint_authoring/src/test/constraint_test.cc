#include <iostream>
#include "authoring/constraint.h"

using namespace std;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Constraint class demo program" << endl;
  Constraint constraint;
  cout << "constraint: " << constraint << endl;
  cout << "end of Constraint class demo program" << endl;
  return status;
}
