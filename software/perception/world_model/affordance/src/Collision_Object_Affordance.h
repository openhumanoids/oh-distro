/*
 * Collision_Object_Affordance.h
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

  class Collision_Object_Affordance: public collision::Collision_Object
{
	//------------------fields
private:
	/**underlying affordance state*/
	affordance::AffConstPtr _affordance;

	//---we're drawing 1 of these
	collision::Collision_Object *_obj;

	//-----------------constructor/destructor
public:
	Collision_Object_Affordance(affordance::AffConstPtr affordance);
	virtual ~Collision_Object_Affordance();

	//Collision_Object interface
	virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
	virtual Eigen::Vector3f position( void ) const;
	virtual Eigen::Vector4f orientation( void ) const;
    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;
	

	//---------useful methods
 public:
	static bool isSupported(affordance::AffConstPtr affordance); //check if we support collision objects for this type of affordance

};

} /* namespace affordance */
#endif /* COLLISION_OBJECT_AFFORDANCE_H */
