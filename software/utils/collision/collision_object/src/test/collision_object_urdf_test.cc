#include <iostream>
#include <Eigen/Dense>
#include <collision/collision_object_urdf.h>

using namespace std;
using namespace Eigen;
using namespace drc;
using namespace kinematics;
using namespace collision;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-urdf-test" << endl << endl;
  // create a collision object class
  Collision_Object_URDF collision_object_urdf( "urdf1" );

//  collision_object_urdf.set( robot_state );

  cout << "collision_object_urdf: " << collision_object_urdf << endl;

  cout << endl << "end of collision-object-urdf-test" << endl << endl;
  return 0;
}
