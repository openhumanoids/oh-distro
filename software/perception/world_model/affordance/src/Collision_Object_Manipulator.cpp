#include <affordance/Collision_Object_Manipulator.h>
#include <collision/collision_object_sphere.h>

using namespace std;
using namespace boost;
using namespace Eigen;

using namespace affordance;
using namespace collision;


//==========constructor / destructor
Collision_Object_Manipulator::Collision_Object_Manipulator(ManipulatorStateConstPtr manipulator) 
  : Collision_Object(manipulator->getGUIDAsString()), 
    _manipulator(manipulator), 
    _cObjs()
{
  if (!isSupported(manipulator))
    throw ArgumentException("Unsupported manipulator passed to Collision_Object_Manipulator Constructor: name = " 
			    + manipulator->getName());
  

  vector<KDL::Frame> collisionPts;
  manipulator->getCollisionContactPoints(collisionPts);

  if (collisionPts.size() == 0)
    return;
  
  for (uint i = 0; i < collisionPts.size(); i++)
    {
      const KDL::Frame &f = collisionPts[i];
      _cObjs.push_back(new Collision_Object_Sphere(manipulator->getGUIDAsString(),
						   0.05, //todo pick radius
						   ModelState::extractXYZ(f), 
						   ModelState::extractQuaternion(f)));
    }

  _bt_collision_objects.clear();
  for (uint i = 0; i < _cObjs.size(); i++)
    {
      vector< btCollisionObject* > next = _cObjs[i]->bt_collision_objects();
      _bt_collision_objects.insert( _bt_collision_objects.end(),
                                    next.begin(),
                                    next.end());
    }
}

Collision_Object_Manipulator::~Collision_Object_Manipulator()
{
  for (uint i = 0; i < _cObjs.size(); i++)
    delete _cObjs[i];
}


//==========Collision_Object interface
void Collision_Object_Manipulator::set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation )
{
  throw NotImplementedException("Collision_Object_Manipulator: set_transform");
}

void Collision_Object_Manipulator::set_transform( const KDL::Frame& transform ){
  throw NotImplementedException("Collision_Object_Manipulator: set_transform");
}

//---------
bool Collision_Object_Manipulator::isSupported(ManipulatorStateConstPtr manipulator)
{
  return true; //todo
}
