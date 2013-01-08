#include <iostream>
#include <map>
#include <path_util/path_util.h>
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
  
  Kinematics_Model_GFE kinematics_model_gfe(getModelsPath() + string( "/mit_gazebo_models/mit_robot_PnC/model.urdf" ));

  robot_state_t robot_state;
  robot_state.origin_position.translation.x = 0.0;
  robot_state.origin_position.translation.y = 0.0;
  robot_state.origin_position.translation.z = 1.0;
  robot_state.origin_position.rotation.x = 0.0;
  robot_state.origin_position.rotation.y = 0.0;
  robot_state.origin_position.rotation.z = 0.0; 
  robot_state.origin_position.rotation.w = 1.0;
  robot_state.num_joints = kinematics_model_gfe.model().joints_.size();
  robot_state.joint_name.resize( kinematics_model_gfe.model().joints_.size() );
  unsigned int index = 0;
  for ( std::map< std::string, boost::shared_ptr< urdf::Joint > >::const_iterator it = kinematics_model_gfe.model().joints_.begin(); it != kinematics_model_gfe.model().joints_.end(); it++ ){
    robot_state.joint_name[ index ] = it->first;
    index++;   
  }  

  robot_state.joint_position.resize( kinematics_model_gfe.model().joints_.size() );
  for( unsigned int i = 0; i < kinematics_model_gfe.model().joints_.size(); i++ ){
    robot_state.joint_position[ i ] = 0.0;
  }

  kinematics_model_gfe.set( robot_state );
  
  cout << endl << "end of kinematics-model-gfe-test" << endl << endl;

  return 0;
}
