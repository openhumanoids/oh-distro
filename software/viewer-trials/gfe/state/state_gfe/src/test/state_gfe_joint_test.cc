#include <iostream>

#include <path_util/path_util.h>
#include <state/state_gfe.h>

using namespace std;
using namespace drc;
using namespace state;

int 
main( int argc, 
      char* argv[] )
{
  State_GFE_Joint state_gfe_joint;
  cout << "state_gfe_joint: " << state_gfe_joint << endl;
  return 0;
}
