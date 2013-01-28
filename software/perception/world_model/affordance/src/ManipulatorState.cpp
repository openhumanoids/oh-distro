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
				   const GlobalUID &guid)
  : _name(link->name), _guid(guid), _link(link)
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

string ManipulatorState::getName() const
{
	return _name;
}

Vector3f ManipulatorState::getColor() const
{
 throw NotImplementedException("manip state");
}
 

Vector3f ManipulatorState::getXYZ() const
{
 throw NotImplementedException("manip state");
}
 
Vector3f ManipulatorState::getRPY() const
{
 throw NotImplementedException("manip state");
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
    throw NotImplementedException("manip state");
}
 
bool ManipulatorState::hasParent() const
{
 throw NotImplementedException("manip state");
}
 
void ManipulatorState::getChildren(vector<shared_ptr<const ManipulatorState> > &children) const 
{
 throw NotImplementedException("manip state");
}
 
void ManipulatorState::getParents(vector<shared_ptr<const ManipulatorState> > &children) const 
{
 throw NotImplementedException("manip state");
}

 void ManipulatorState::getCopy(ManipulatorState &copy) const 
 {
 throw NotImplementedException("manip state");
 }



//=========specific to this class
shared_ptr<const urdf::Link> ManipulatorState::getLink() const 
{
  return _link;
}
