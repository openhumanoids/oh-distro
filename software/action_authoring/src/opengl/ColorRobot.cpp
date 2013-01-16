#include "ColorRobot.h" 

using namespace std;
using namespace boost;
using namespace urdf;
using namespace KDL;
using namespace Eigen;
using namespace drc;
using namespace state;
using namespace kinematics_model;
using namespace opengl;
using namespace robot_opengl;

void
ColorRobot::
draw(void) {
  if( visible() ){
    for( unsigned int i = 0; i < _opengl_objects.size(); i++ ){
      if( _opengl_objects[ i ] != NULL ){
        _opengl_objects[ i ]->set_transform( _kinematics_model.link( _opengl_objects[ i ]->id() ) );
	if (_selected_link_name.compare(_opengl_objects[ i ]->id()) == 0) {
	    Eigen::Vector3f color = Eigen::Vector3f(1.0, 0.0, 0.0);
	    cout << "found link!!" << std::endl;
	    _opengl_objects[ i ]->draw(color);
	} else { 
	    _opengl_objects[ i ]->draw();
	}
      }
    }
  }
  return;
}

void 
ColorRobot::
setSelectedJoint(std::string jointName) {
    urdf::Model m = _kinematics_model.model();
    _selected_link_name = m.getJoint(jointName)->child_link_name;
}
