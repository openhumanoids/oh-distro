#include <iostream>
#include <Eigen/Dense>
#include <collision_detection/collision_object_cone.h>

using namespace std;
using namespace Eigen;
using namespace collision_detection;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-cone-test" << endl << endl;
  // create a collision object class
  Collision_Object_Cone collision_object_cone_1( "cone1" );
  cout << "collision_object_cone_1: " << collision_object_cone_1 << endl;
  cout << endl << "end of collision-object-cone-test" << endl << endl;
  return 0;
}
