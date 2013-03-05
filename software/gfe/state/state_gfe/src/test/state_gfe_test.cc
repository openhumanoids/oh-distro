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
  State_GFE state_gfe;
  cout << state_gfe << endl;

  robot_state_t robot_state;
  robot_state.num_joints = 0;
  state_gfe.from_lcm( &robot_state );
  cout << state_gfe << endl;

  return 0;
}
