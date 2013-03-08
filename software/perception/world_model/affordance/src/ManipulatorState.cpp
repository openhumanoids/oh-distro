#include "ManipulatorState.h"
#include <affordance/ToString.h>

using namespace affordance;
using namespace std;
using namespace boost;
using namespace Eigen;

ManipulatorState::ManipulatorState(const std::string &name,
				   const GlobalUID &guid)
  : _name(name), _guid(guid), _link()     
{
    _contact_group_name = "default";
}

ManipulatorState::ManipulatorState(shared_ptr<const urdf::Link> link, 
                                   const string &contact_group_name,
                                   KDL::Frame link_frame,
                                   const GlobalUID &guid)
  : _name(link->name + "." + contact_group_name), 
    _guid(guid), _link(link), _link_frame(link_frame),
    _contact_group_name(contact_group_name)
{
  _link_name = link->name;
}


/*ManipulatorState::ManipulatorState(const ManipulatorState &other) {
  }*/


/*ManipulatorState& ManipulatorState::operator=( const ManipulatorState& rhs ) {
  }*/

ManipulatorState::~ManipulatorState() {

}

GlobalUID ManipulatorState::getGlobalUniqueId() const
{
  return _guid;
}


string ManipulatorState::getName() const
{
  return _name;
}

Vector3f ManipulatorState::getColor() const
{
  return Vector3f(0,0,1); //todo
}
 

Vector3f ManipulatorState::getXYZ() const
{
  throw NotImplementedException("getXYZ");
}
 
Vector3f ManipulatorState::getRPY() const
{
  throw NotImplementedException("getRPY");
}
 
bool ManipulatorState::isAffordance() const
{
    return false;
}
 
bool ManipulatorState::isManipulator() const
{
    return true;
}

bool ManipulatorState::hasChildren() const
{
  throw NotImplementedException("hasChildren");
}
 
bool ManipulatorState::hasParent() const
{
  throw NotImplementedException("hasParent");
}
 
void ManipulatorState::getChildren(vector<ModelStateConstPtr> &children) const 
{
  throw NotImplementedException("getChildren");
}
 
void ManipulatorState::getParents(vector<ModelStateConstPtr> &parents) const 
{
  throw NotImplementedException("getParents");
}

 void ManipulatorState::getCopy(ModelState &copy) const 
 {
 throw NotImplementedException("getCopy");
 }



//=========specific to this class
shared_ptr<const urdf::Link> ManipulatorState::getLink() const 
{
  return _link;
}





/**get the collision contact points for this manipulator.*/
void ManipulatorState::getCollisionContactPoints(vector<KDL::Frame> &pts) const
{
  pts.clear();

  CollisionGroupPtr colgroup = _link->getCollisions(_contact_group_name);
  if (colgroup == NULL || colgroup->size() == 0) 
    return;

  //get collision contact points
  for (uint i = 0; i < colgroup->size(); i++) 
    {
      urdf::Pose ctPtPose = (*colgroup)[i]->origin; //contact point pose in link frame
      double q1, q2, q3, q4;
      ctPtPose.rotation.getQuaternion(q1, q2, q3, q4);
      KDL::Frame cPtAsFrame(KDL::Rotation::Quaternion(q1, q2, q3, q4),  //expressed as a frame
			    KDL::Vector(ctPtPose.position.x, 
					ctPtPose.position.y, 
					ctPtPose.position.z));
      KDL::Frame f = cPtAsFrame * getLinkFrame(); //manipulator is in robot-oriented frame?  
      pts.push_back(f);

      /*
	cout << "\n\n=========================================" << endl;
	cout << "\n cPtAsFrame = " << toStr(cPtAsFrame) << endl;
	cout << "\n link frame in world = " << toStr(_manipulator->getLinkFrame()) << endl;
	cout << "cPtAsFrame * manipulator->getLinkFrame() = " << toStr(f) << endl;
	cout << "manipulator->getLinkFrame() * cPtAsFrame = " << toStr(_manipulator->getLinkFrame() * cPtAsFrame) << endl;
      */
    }
}

/*void ManipulatorState::setContactGroupName(std::string contactGroupName)
{
  _contact_group_name = contactGroupName;
  }*/
