#include <iostream>
#include <Eigen/Dense>
#include <collision_detection/collision_object_sphere.h>

using namespace std;
using namespace Eigen;
using namespace collision_detection;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-sphere-test" << endl << endl;
  // create a collision object class
  Collision_Object_Sphere collision_object_sphere_1( "sphere1" );
  cout << "collision_object_sphere_1: " << collision_object_sphere_1 << endl;
  cout << endl << "end of collision-object-sphere-test" << endl << endl;
  return 0;
}
