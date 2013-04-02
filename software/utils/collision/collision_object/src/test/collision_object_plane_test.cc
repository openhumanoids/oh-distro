#include <iostream>
#include <Eigen/Dense>
#include <collision/collision_object_plane.h>

using namespace std;
using namespace Eigen;
using namespace collision;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-plane-test" << endl << endl;
  // create a collision object class
  Collision_Object_Plane collision_object_plane_1( "plane1" );
  cout << "collision_object_plane_1: " << collision_object_plane_1 << endl;
  cout << endl << "end of collision-object-plane-test" << endl << endl;
  return 0;
}
