#include <iostream>
#include "state/state.h"

using namespace std;
using namespace state;

int
main( int argc,
      char* argv[] ) {
  int status = 0;
  cout << "start of State class demo program" << endl;
  State state;
  cout << "state: " << state << endl;
  cout << "end of State class demo program" << endl;
  return status;
}
