/*
 * OpenGLAffordance.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: mfleder
 */

#include "Collision_Object_Affordance.h"
#include <collision/collision_object_box.h>
#include <collision/collision_object_sphere.h>
#include <collision/collision_object_cylinder.h>

using namespace std;
using namespace boost;
using namespace Eigen;

using namespace affordance;
using namespace collision;

//--------------constructor/destructor
Collision_Object_Affordance::Collision_Object_Affordance(AffConstPtr affordance) 
  : Collision_Object(affordance->getGUIDAsString()), 
    _affordance(affordance), 
    _obj(NULL)
{
  if (!isSupported(affordance))
    throw ArgumentException("Unsupported affordance passed to Collision_Object_Affordance Constructor: otdf_type = " 
			    + affordance->_otdf_type);
  
  Vector3f xyz = affordance->getXYZ();
  Vector4f q = affordance->getQuaternion();
  
  if (affordance->_otdf_type == AffordanceState::BOX) 
    {
      _obj = new Collision_Object_Box(affordance->getGUIDAsString(), 
				      Vector3f(affordance->length(), affordance->width(), affordance->height()), 
				      xyz, q);
    }
  else if (affordance->_otdf_type == AffordanceState::CYLINDER) 
    {
      _obj = new Collision_Object_Cylinder(affordance->getGUIDAsString(),
					   affordance->radius(), affordance->length(),
					   xyz, q);
    }
  else if (affordance->_otdf_type == AffordanceState::SPHERE) 
    {
      _obj = new Collision_Object_Sphere(affordance->getGUIDAsString(),
					 affordance->radius(),
					 xyz, q);
    }
  else throw std::runtime_error("Collision_Object_Affordance constructor : unrecognized object");
}

Collision_Object_Affordance::~Collision_Object_Affordance()
{
  if (_obj != NULL)
    delete _obj;
}

bool Collision_Object_Affordance::isSupported(AffConstPtr affordance)
{
  return affordance->_otdf_type == AffordanceState::CYLINDER ||
    affordance->_otdf_type == AffordanceState::BOX ||
    affordance->_otdf_type == AffordanceState::SPHERE;
}


//==========Collision_Object interface
void Collision_Object_Affordance::set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation )
{
  _obj->set_transform(position, orientation);
}

Eigen::Vector3f Collision_Object_Affordance::position( void ) const
{
  return _obj->position();
}

Eigen::Vector4f Collision_Object_Affordance::orientation( void ) const
{
  return _obj->orientation();
}


vector< btCollisionObject* > Collision_Object_Affordance::bt_collision_objects( void )
{
  vector< btCollisionObject* > bt_collision_objects = _obj->bt_collision_objects();
  return bt_collision_objects;
}

vector< const btCollisionObject* > Collision_Object_Affordance::bt_collision_objects( void ) const
{
  vector< const btCollisionObject* > bt_collision_objects = ((const Collision_Object*)_obj)->bt_collision_objects();
  return bt_collision_objects;
}
