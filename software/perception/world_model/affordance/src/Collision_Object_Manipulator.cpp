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

Eigen::Vector3f Collision_Object_Manipulator::position( void ) const
{
  throw NotImplementedException("Collision_Object_Manipulator: position()");
}

Eigen::Vector4f Collision_Object_Manipulator::orientation( void ) const
{
  throw NotImplementedException("Collision_Object_Manipulator: orietation()");
}


vector< btCollisionObject* > Collision_Object_Manipulator::bt_collision_objects( void )
{
  vector< btCollisionObject* > bt_collision_objects;
  for (uint i = 0; i < _cObjs.size(); i++)
    {
      vector< btCollisionObject* > next = _cObjs[i]->bt_collision_objects();	      
      bt_collision_objects.insert(bt_collision_objects.end(),
				  next.begin(),
				  next.end());
    }
  
  return bt_collision_objects;
}

vector< const btCollisionObject* > Collision_Object_Manipulator::bt_collision_objects( void ) const
{
  vector< const btCollisionObject* > bt_collision_objects;
  for (uint i = 0; i < _cObjs.size(); i++)
    {
      vector<const btCollisionObject* > next = ((const Collision_Object *) _cObjs[i])->bt_collision_objects();	      
      bt_collision_objects.insert(bt_collision_objects.end(),
				  next.begin(),
				  next.end());
    }
  return bt_collision_objects;
}


//---------
bool Collision_Object_Manipulator::isSupported(ManipulatorStateConstPtr manipulator)
{
  return true; //todo
}
