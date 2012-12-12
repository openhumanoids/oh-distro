#include <iostream>
#include <map>

#include <kinematics_model/kinematics_model_gfe.h>

using namespace std;
using namespace urdf;
using namespace KDL;
using namespace drc;
using namespace kinematics_model;

int 
main( int argc, 
      char* argv[])
{
  cout << endl << "start of kinematics-model-gfe-test" << endl << endl;
  Kinematics_Model_GFE kinematics_model_gfe( "/home/tmhoward/projects/drc/software/models/mit_gazebo_models/mit_robot_PnC/model.sdf" );

  robot_state_t robot_state;
  robot_state.origin_position.translation.x = 0.0;
  robot_state.origin_position.translation.y = 0.0;
  robot_state.origin_position.translation.z = 1.0;
  robot_state.origin_position.rotation.x = 0.0;
  robot_state.origin_position.rotation.y = 0.0;
  robot_state.origin_position.rotation.z = 0.0; 
  robot_state.origin_position.rotation.w = 1.0;
  robot_state.num_joints = kinematics_model_gfe.tree().getNrOfJoints();
  robot_state.joint_position.resize( kinematics_model_gfe.tree().getNrOfJoints() );
  for( unsigned int i = 0; i < kinematics_model_gfe.tree().getNrOfJoints(); i++ ){
    robot_state.joint_position[ i ] = 0.0;
  }

  kinematics_model_gfe.set( robot_state );
  
  cout << endl << "end of kinematics-model-gfe-test" << endl << endl;

  return 0;
}
