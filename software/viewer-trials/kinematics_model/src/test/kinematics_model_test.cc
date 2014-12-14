#include <iostream>
#include <map>

#include <kinematics/kinematics_model.h>

using namespace std;
using namespace KDL;
using namespace drc;
using namespace kinematics;

int 
main( int argc, 
      char* argv[])
{
  cout << endl << "start of kinematics-model-test" << endl << endl;

  position_3d_t position_3d;
  position_3d.translation.x = 0.0;
  position_3d.translation.y = 0.0;
  position_3d.translation.z = 0.0;
  position_3d.rotation.x = 0.0;  
  position_3d.rotation.y = 0.0;  
  position_3d.rotation.z = 0.0;  
  position_3d.rotation.w = 1.0;  

  transform_t transform;
  transform.translation.x = 0.0;
  transform.translation.y = 0.0;
  transform.translation.z = 0.0;
  transform.rotation.x = 0.0;
  transform.rotation.y = 0.0;
  transform.rotation.z = 0.0;
  transform.rotation.w = 1.0;
  
  Frame frame;

  // test conversion of drc::position_3d_t to KDL::Frame 
  Kinematics_Model::drc_position_3d_t_to_kdl_frame( position_3d, frame );

  // test conversion of drc::transform_t to KDL::Frame
  Kinematics_Model::drc_transform_t_to_kdl_frame( transform, frame );
  
  // test conversion of KDL::Frame to drc::transform_t
  Kinematics_Model::kdl_frame_to_drc_transform_t( frame, transform ); 

  cout << endl << "end of kinematics-model-test" << endl << endl;

  return 0;
}
