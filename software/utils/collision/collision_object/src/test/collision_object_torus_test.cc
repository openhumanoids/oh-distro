#include <iostream>
#include <Eigen/Dense>
#include <collision/collision_object_torus.h>

using namespace std;
using namespace Eigen;
using namespace collision;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-torus-test" << endl << endl;
  // create a collision object class
  Collision_Object_Torus collision_object_torus_1( "torus1" );
  cout << "collision_object_torus_1: " << collision_object_torus_1 << endl;
  cout << endl << "end of collision-object-torus-test" << endl << endl;
  return 0;
}
