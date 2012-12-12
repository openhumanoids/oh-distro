#include <iostream>
#include <Eigen/Dense>
#include <path_util/path_util.h>
#include <collision_detection/collision_object_convex_hull.h>

using namespace std;
using namespace Eigen;
using namespace collision_detection;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-convex-hull-test" << endl << endl;
  // create a collision object class
  Collision_Object_Convex_Hull collision_object_convex_hull( "convex_hull", getModelsPath() + string( "/mit_gazebo_models/mit_robot/meshes/head_chull.obj" ) );
  cout << "collision_object_convex_hull: " << collision_object_convex_hull << endl;
  cout << endl << "end of collision-object-convex-hull-test" << endl << endl;
  return 0;
}
