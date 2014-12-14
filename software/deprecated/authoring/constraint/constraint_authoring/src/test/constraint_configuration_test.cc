#include <iostream>
#include "authoring/constraint_configuration.h"

using namespace std;
using namespace authoring;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of Constraint_Configuration class demo program" << endl;
  Constraint_Configuration constraint_configuration;
  cout << "constraint_configuration: " << constraint_configuration << endl;
  cout << "end of Constraint_Configuration class demo program" << endl;
  return status;
}
