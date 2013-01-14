#include <iostream>
#include <Eigen/Dense>
#include <collision/collision_object_point_cloud.h>

using namespace std;
using namespace Eigen;
using namespace drc;
using namespace collision;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-point-cloud-test" << endl << endl;
  // create a collision object class
  Collision_Object_Point_Cloud collision_object_point_cloud( "pc1" );
  
  vector< Vector3f > points;
  for( unsigned int i = 0; i < 100; i++ ){
    Vector3f point( -5.0 + 10.0 * ( rand() % 1000 ) / 1000.0, 
                    -5.0 + 10.0 * ( rand() % 1000 ) / 1000.0,
                    -5.0 + 10.0 * ( rand() % 1000 ) / 1000.0 );
    points.push_back( point );
  }

  collision_object_point_cloud.set( points );
  cout << "collision_object_point_cloud: " << collision_object_point_cloud << endl;

  points.clear();
  for( unsigned int i = 0; i < 200; i++ ){
    Vector3f point( -5.0 + 10.0 * ( rand() % 1000 ) / 1000.0,
                    -5.0 + 10.0 * ( rand() % 1000 ) / 1000.0,
                    -5.0 + 10.0 * ( rand() % 1000 ) / 1000.0 );
    points.push_back( point );
  }

  collision_object_point_cloud.set( points );
  cout << "collision_object_point_cloud: " << collision_object_point_cloud << endl;

  points.clear();
  for( unsigned int i = 0; i < 50; i++ ){
    Vector3f point( -5.0 + 10.0 * ( rand() % 1000 ) / 1000.0,
                    -5.0 + 10.0 * ( rand() % 1000 ) / 1000.0,
                    -5.0 + 10.0 * ( rand() % 1000 ) / 1000.0 );
    points.push_back( point );
  }

  collision_object_point_cloud.set( points );
  cout << "collision_object_point_cloud: " << collision_object_point_cloud << endl;

  cout << endl << "end of collision-object-point-cloud-test" << endl << endl;
  return 0;
}
