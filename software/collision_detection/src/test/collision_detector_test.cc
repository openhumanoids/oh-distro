#include <iostream>
#include <Eigen/Dense>
#include <collision_detection/collision_detector.h>
#include <collision_detection/collision_object_box.h>

using namespace std;
using namespace Eigen;
using namespace collision_detection;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-detector-test" << endl << endl;

  // create a collision detector class
  Collision_Detector collision_detector;

  // check the number of collisions
  cout << "num_collisions: " << collision_detector.num_collisions() << " (should be zero)" << endl;

  // add a collision object and add it to the collision detector class
  Collision_Object_Box collision_object_1( "box1", Vector3f( 0.5, 0.5, 0.5 ), Vector3f( 0.0, 0.0, 0.0 ), Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
  collision_detector.add_collision_object( &collision_object_1 );

  // check the number of collisions
  cout << "num_collisions: " << collision_detector.num_collisions() << " (should be zero)" << endl;

  // add another collision object and add it to the collision detector class
  Collision_Object_Box collision_object_2( "box2", Vector3f( 0.5, 0.5, 0.5 ), Vector3f( 2.0, 0.0, 0.0 ), Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
  collision_detector.add_collision_object( &collision_object_2 );
 
  cout << "collision_object_2: " << collision_object_2 << endl;

  // check out the copy constructor
  Collision_Object_Box collision_object_3( collision_object_2 );
  cout << "collision_object_3: " << collision_object_3 << endl;
 
  // check the number of collisions
  cout << "num_collisions: " << collision_detector.num_collisions() << " (should be zero)" << endl;

  // move the second collision object too close to the first collision object
  collision_object_2.set_transform( Vector3f( 0.25, 0.0, 0.0 ), Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
  cout << "collision_object_2: " << collision_object_2 << endl;

  // check the number of collisions
  cout << "num_collisions: " << collision_detector.num_collisions() << " (should be non-zero)" << endl;

  // check ray interaction (positive example)
  Collision_Object * intersected_object = NULL;
  collision_detector.ray_test( Vector3f( 10.0, 0.0, 0.0 ), Vector3f( 0.0, 0.0, 0.0 ), intersected_object );
  if( intersected_object != NULL ){
    cout << "intersected " << intersected_object->id().c_str() << " ";
  } else {
    cout << "did not intersect with any objects ";
  }
  cout << "(should intersect with box2)" << endl;

  // check ray interaction (positive example)
  collision_detector.ray_test( Vector3f( -10.0, 0.0, 0.0 ), Vector3f( 0.0, 0.0, 0.0 ), intersected_object );
  if( intersected_object != NULL ){
    cout << "intersected " << intersected_object->id().c_str() << " ";
  } else {
    cout << "did not intersect with any objects ";
  }
  cout << "(should intersect with box1)" << endl;

  // check ray interaction (negative example)
  collision_detector.ray_test( Vector3f( -10.0, 5.0, 0.0 ), Vector3f( 0.0, 5.0, 0.0 ), intersected_object );
  if( intersected_object != NULL ){
    cout << "intersected " << intersected_object->id().c_str() << " ";
  } else {
    cout << "did not intersect with any objects ";
  }
  cout << "(should not intersect with anything)" << endl;

  // clear the collision world
  collision_detector.clear_collision_objects();
  
  // check the number of collisions
  cout << "num_collisions: " << collision_detector.num_collisions() << " (should be zero)" << endl;

  cout << endl << "end of collision-detector-test" << endl << endl;
  return 0;
}
