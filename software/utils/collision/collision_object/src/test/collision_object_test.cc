#include <iostream>
#include <Eigen/Dense>
#include <collision/collision_object.h>

using namespace std;
using namespace Eigen;
using namespace collision;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-test" << endl << endl;
  // create a collision object class
  Collision_Object collision_object_1( "object1" );
  cout << "collision_object_1: " << collision_object_1 << endl;
  cout << endl << "end of collision-object-test" << endl << endl;
  return 0;
}
