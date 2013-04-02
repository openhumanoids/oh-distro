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
			    + affordance->getType());
  
  Vector3f xyz = affordance->getXYZ();
  Vector4f q = affordance->getQuaternion();
  
  if (affordance->getType() == AffordanceState::BOX) 
    {
      _obj = new Collision_Object_Box(affordance->getGUIDAsString(), 
				      Vector3f(affordance->length(), affordance->width(), affordance->height()), 
				      xyz, q);
    }
  else if (affordance->getType() == AffordanceState::CYLINDER) 
    {
      _obj = new Collision_Object_Cylinder(affordance->getGUIDAsString(),
					   affordance->radius(), affordance->length(),
					   xyz, q);
    }
  else if (affordance->getType() == AffordanceState::SPHERE) 
    {
      _obj = new Collision_Object_Sphere(affordance->getGUIDAsString(),
					 affordance->radius(),
					 xyz, q);
    }
  else throw std::runtime_error("Collision_Object_Affordance constructor : unrecognized object");
  _bt_collision_objects = _obj->bt_collision_objects();
}

Collision_Object_Affordance::~Collision_Object_Affordance()
{
  if (_obj != NULL)
    delete _obj;
}

bool Collision_Object_Affordance::isSupported(AffConstPtr affordance)
{
  return affordance->getType() == AffordanceState::CYLINDER ||
    affordance->getType() == AffordanceState::BOX ||
    affordance->getType() == AffordanceState::SPHERE;
}


//==========Collision_Object interface
void Collision_Object_Affordance::set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation )
{
  _obj->set_transform(position, orientation);
}

void Collision_Object_Affordance::set_transform( const KDL::Frame& transform )
{
  _obj->set_transform(transform);
}
