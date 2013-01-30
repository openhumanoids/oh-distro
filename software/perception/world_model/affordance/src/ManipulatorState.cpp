#include "ManipulatorState.h"

using namespace affordance;
using namespace std;
using namespace boost;
using namespace Eigen;

ManipulatorState::ManipulatorState(const std::string &name,
				   const GlobalUID &guid)
  : _name(name), _guid(guid), _link()     
{
}

ManipulatorState::ManipulatorState(shared_ptr<const urdf::Link> link, 
				   KDL::Frame link_frame,
				   const GlobalUID &guid)
    : _name(link->name), _guid(guid), _link(link), _link_frame(link_frame)
{
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

string ManipulatorState::getGUIDAsString()  const 
{
    return ModelState::toStr(getGlobalUniqueId().first) + "," + ModelState::toStr(getGlobalUniqueId().second);
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



string toStr(const KDL::Vector &v)
{
  return string("(") + ModelState::toStr(v.x()) + string(", ") 
    + ModelState::toStr(v.y()) + string(", ") 
    + ModelState::toStr(v.z()) + string(")");
}


string toStr(const KDL::Rotation &r)
{
  double q1,q2,q3,q4;
  r.GetQuaternion(q1,q2,q3,q4);
  return string("(") + ModelState::toStr(q1) + string(", ") 
    + ModelState::toStr(q2) + string(", ") 
    + ModelState::toStr(q3)
    + string(", ")
    + ModelState::toStr(q4) + string(")");
}


string toStr(const KDL::Frame &frame)
{
  return string("Position = ") + toStr(frame.p)
    + string("\t Rotation = ") + toStr(frame.M);
}


std::ostream& operator << (std::ostream &out, const KDL::Rotation &r)
{
  out << toStr(r);
  return out;
}

std::ostream& operator << (std::ostream &out, const KDL::Vector &v)
{
  out << toStr(v);
  return out;
}

std::ostream& operator <<(std::ostream& out, const KDL::Frame &frame)
{
  out << "Position = " << frame.p << "\t Rotation = " << frame.M << endl;
  return out;
}

/**get the collision contact points for this manipulator.*/
void ManipulatorState::getCollisionContactPoints(vector<KDL::Frame> &pts) const
{
  pts.clear();

  CollisionGroupPtr colgroup = _link->getCollisions("default");
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
