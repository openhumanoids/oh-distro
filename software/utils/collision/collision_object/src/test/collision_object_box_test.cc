#include <iostream>
#include <Eigen/Dense>
#include <collision/collision_object_box.h>

using namespace std;
using namespace Eigen;
using namespace collision;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-box-test" << endl << endl;
  // create a collision object class
  Collision_Object_Box collision_object_box_1( "box1" );
  cout << "collision_object_box_1: " << collision_object_box_1 << endl;
  cout << endl << "end of collision-object-box-test" << endl << endl;
  return 0;
}
