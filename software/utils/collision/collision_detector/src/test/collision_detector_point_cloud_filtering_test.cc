#include <iostream>
#include <Eigen/Dense>
#include <collision/collision.h>
#include <collision/collision_detector.h>
#include <collision/collision_object_point_cloud.h>

using namespace std;
using namespace drc;
using namespace Eigen;
using namespace collision;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-detector-point-cloud-filtering-test" << endl << endl;

  // create the point cloud collision object (args: id, max # points)
  Collision_Object_Point_Cloud collision_object_point_cloud( "collision-object-point-cloud", 1000 );

  // create the collision detector
  Collision_Detector collision_detector;

  // add the two collision objects to the collision detector with different groups and filters (to prevent checking of self collisions)
  collision_detector.add_collision_object( &collision_object_point_cloud, COLLISION_DETECTOR_GROUP_2, COLLISION_DETECTOR_GROUP_1 ); 

  // create a std::vector< Eigen::Vector3f > of points
  vector< Vector3f > points;
  for( unsigned int i = 0; i < 500; i++ ){
    Vector3f point( -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    -1.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0,
                    0.0 + 2.0 * ( double )( rand() % 1000 ) / 1000.0 );
    points.push_back( point );
  }

  // run the test 10 times to test durability
  for( unsigned int i = 0; i < 10; i++ ){
    // set the state of the collision objects
    collision_object_point_cloud.set( points );
 
    // get the vector of collisions by running collision detection
    vector< Collision > collisions = collision_detector.get_collisions();

    // display the collisions (debugging purposes only)
    for( unsigned int j = 0; j < collisions.size(); j++ ){
      cout << "collisions[" << j << "]: " << collisions[ j ] << endl;
    }

    vector< Vector3f > unfiltered_points = points;
    vector< Vector3f > filtered_points;
        
    vector< unsigned int > filtered_point_indices;
    for( unsigned int j = 0; j < collisions.size(); j++ ){
      filtered_point_indices.push_back( atoi( collisions[ j ].second_id().c_str() ) );
    }

    for( unsigned int j = 0; j < points.size(); j++ ){
      bool point_filtered = false;
      for( unsigned int k = 0; k < filtered_point_indices.size(); k++ ){
        if( j == filtered_point_indices[ k ] ){
          point_filtered = true;
        }
      }
      if( point_filtered ){
        filtered_points.push_back( points[ j ] );
      } else {
        unfiltered_points.push_back( points[ j ] );
      }
    }
 
    cout << "filtered_points.size(): " << filtered_points.size() << endl;
    cout << "unfiltered_points.size(): " << unfiltered_points.size() << endl;
  }

  cout << endl << "end of collision-detector-point-cloud-filtering-test" << endl << endl;
  return 0;
}
