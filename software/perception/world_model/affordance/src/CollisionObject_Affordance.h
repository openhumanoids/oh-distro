/*
 * CollisionObject_Affordance.h
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
	affordance::AffConstPtr _affordance;

	//---we're drawing 1 of these
	collision::Collision_Object *_obj;

	//-----------------constructor/destructor
public:
	CollisionObject_Affordance(affordance::AffConstPtr affordance,
				   const std::string &id);
	virtual ~CollisionObject_Affordance();

	//--------todo : implement the collision object interface
	//by calling the appropriate method on whichever field we are using

	
	//---------useful methods
 public:
	static bool isSupported(affordance::AffConstPtr affordance); //check if we support collision objects for this type of affordance

};

} /* namespace affordance */
#endif /* COLLISION_OBJECT_AFFORDANCE_H */
