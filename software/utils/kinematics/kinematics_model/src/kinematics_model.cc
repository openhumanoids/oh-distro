#include <kinematics/kinematics_model.h>

using namespace std;
using namespace KDL;
using namespace drc;
using namespace kinematics;

Kinematics_Model::
Kinematics_Model() {

}

Kinematics_Model::
Kinematics_Model( const Kinematics_Model& other ) {

}

Kinematics_Model::
~Kinematics_Model(){

}

void 
Kinematics_Model::
drc_position_3d_t_to_kdl_frame( const drc::position_3d_t& position3d, 
                                KDL::Frame& frame ){
  frame.p[0] = position3d.translation.x;
  frame.p[1] = position3d.translation.y;
  frame.p[2] = position3d.translation.z;
  frame.M = KDL::Rotation::Quaternion( position3d.rotation.x, position3d.rotation.y, position3d.rotation.z, position3d.rotation.w );
  return;
}


void
Kinematics_Model::
drc_transform_t_to_kdl_frame( const transform_t& transform,
                              Frame& frame ){
  frame.p[0] = transform.translation.x;
  frame.p[1] = transform.translation.y;
  frame.p[2] = transform.translation.z;
  frame.M = KDL::Rotation::Quaternion( transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w );
  return;
}

void
Kinematics_Model::
kdl_frame_to_drc_transform_t( const Frame& frame,
                              transform_t& transform ){
  transform.translation.x = frame.p[0];
  transform.translation.y = frame.p[1];
  transform.translation.z = frame.p[2];
  frame.M.GetQuaternion( transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w );
  return;
}

void 
Kinematics_Model::
kdl_frame_to_drc_position_3d_t( const Frame& frame, 
                                position_3d_t& position3d ){
  position3d.translation.x = frame.p[0];
  position3d.translation.y = frame.p[1];
  position3d.translation.z = frame.p[2];
  frame.M.GetQuaternion( position3d.rotation.x, position3d.rotation.y, position3d.rotation.z, position3d.rotation.w );
  return;
}

namespace kinematics {
  ostream&
  operator<<( ostream& out,
              const Kinematics_Model& other ){
    return out;
  }
} 
