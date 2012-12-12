#include <iostream>
#include <Eigen/Dense>
#include <collision_detection/collision_object_gfe.h>

using namespace std;
using namespace Eigen;
using namespace drc;
using namespace kinematics_model;
using namespace collision_detection;

int 
main( int argc, 
      char** argv )
{
  cout << endl << "start of collision-object-gfe-test" << endl << endl;
  // create a collision object class
  Collision_Object_GFE collision_object_gfe( "gfe1" );

  robot_state_t robot_state;
  robot_state.origin_position.translation.x = 0.0;
  robot_state.origin_position.translation.y = 0.0;
  robot_state.origin_position.translation.z = 1.0;
  robot_state.origin_position.rotation.x = 0.0;
  robot_state.origin_position.rotation.y = 0.0;
  robot_state.origin_position.rotation.z = 0.0;
  robot_state.origin_position.rotation.w = 1.0;
  robot_state.num_joints = collision_object_gfe.kinematics_model().tree().getNrOfJoints();
  robot_state.joint_position.resize( collision_object_gfe.kinematics_model().tree().getNrOfJoints() );
  for( unsigned int i = 0; i < collision_object_gfe.kinematics_model().tree().getNrOfJoints(); i++ ){
    robot_state.joint_position[ i ] = 0.0;
  }

  collision_object_gfe.set( robot_state );

  cout << "collision_object_gfe: " << collision_object_gfe << endl;

  cout << endl << "end of collision-object-gfe-test" << endl << endl;
  return 0;
}
