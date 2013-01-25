#include "ManipulatorState.h"

using namespace affordance;
using namespace std;
using namespace boost;
using namespace Eigen;

ManipulatorState::ManipulatorState(std::string name)
{
    _name = name;
    // todo actually unique id
    _id1 = rand();
    _id2 = rand();
}

ManipulatorState::ManipulatorState(const ManipulatorState &other) {
}

ManipulatorState& ManipulatorState::operator=( const ManipulatorState& rhs ) {
}

ManipulatorState::~ManipulatorState() {

}

GlobalUID ManipulatorState::getGlobalUniqueId() const
{
    return GlobalUID(_id1, _id2);
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
