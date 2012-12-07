#include <iostream>
#include <Eigen/Dense>
#include <collision_detection/collision_object_cylinder.h>

using namespace std;
using namespace Eigen;
using namespace collision_detection;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-cylinder-test" << endl << endl;
  // create a collision object class
  Collision_Object_Cylinder collision_object_cylinder_1( "cylinder1" );
  cout << "collision_object_cylinder_1: " << collision_object_cylinder_1 << endl;
  cout << endl << "end of collision-object-cylinder-test" << endl << endl;
  return 0;
}
