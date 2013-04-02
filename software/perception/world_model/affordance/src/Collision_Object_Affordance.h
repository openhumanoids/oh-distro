/*
 * Collision_Object_Affordance.h
 *
 *  Created on: Jan 14, 2013
 *      Author: mfleder
 */

#ifndef COLLISION_OBJECT_AFFORDANCE_H
#define COLLISION_OBJECT_AFFORDANCE_H

#include <affordance/AffordanceState.h>
#include <collision/collision_object.h>
namespace affordance
{

  class Collision_Object_Affordance: public collision::Collision_Object
{
	//------------------fields
private:
	/**underlying affordance state*/
	affordance::AffConstPtr _affordance;

	//---will be set to some primitive collision object
	collision::Collision_Object *_obj;

	//-----------------constructor/destructor
public:
	Collision_Object_Affordance(affordance::AffConstPtr affordance);
	virtual ~Collision_Object_Affordance();

	//Collision_Object interface
	virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
	virtual void set_transform( const KDL::Frame& transform );
	
        //---------useful methods
 public:
	static bool isSupported(affordance::AffConstPtr affordance); //check if we support collision objects for this type of affordance

};

} /* namespace affordance */
#endif /* COLLISION_OBJECT_AFFORDANCE_H */
