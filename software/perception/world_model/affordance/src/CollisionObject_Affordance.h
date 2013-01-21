/*
 * OpenGLAffordance.h
 *
 *  Created on: Jan 14, 2013
 *      Author: mfleder
 */

#ifndef COLLISION_OBJECT_AFFORDANCE_H
#define COLLISION_OBJECT_AFFORDANCE_H

#include "opengl/opengl_object.h"
#include "affordance/AffordanceState.h"
#include "collision/collision_object_cylinder.h"
#include "collision/collision_object_sphere.h"
#include "collision/collision_object_box.h"
//todo : cone, hull, point cloud, gfe

namespace affordance
{

  class CollisionObject_Affordance: public collision::Collision_Object
{
	//------------------fields
private:
	/**underlying affordance state*/
	affordance::AffordanceState _affordance;

	//---we're drawing 1 of these
	collision::Collision_Object *_object; //box, cylinder, sphere

	//-----------------constructors
public:
	CollisionObject_Affordance(const affordance::AffordanceState &affordance,
				   const std::string &id);
	virtual ~CollisionObject_Affordance();

	//--------todo : implement the collision object interface
	//by calling the appropriate method on whichever field we are using

};

} /* namespace affordance */
#endif /* COLLISION_OBJECT_AFFORDANCE_H_ */
