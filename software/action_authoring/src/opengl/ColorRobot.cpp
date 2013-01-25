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

}

CollisionGroupPtr
ColorRobot::
getCollisionGroupsForLink(std::string link_name) {
    urdf::Model m = _kinematics_model.model();
    boost::shared_ptr<const urdf::Link> t = m.getLink(link_name);
    boost::shared_ptr<std::vector<boost::shared_ptr<Collision > > > cols = t->getCollisions("default");
    return cols;
}

boost::shared_ptr<const urdf::Link>
ColorRobot::
getLinkFromJointName(std::string joint_name) {
    urdf::Model m = _kinematics_model.model();
    return m.getLink(m.getJoint(joint_name)->child_link_name);
}
