/**
 * @file collision_object_cone.h
 * @author Thomas Howard
 * @namespace collision_detection
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a cone-shaped collision object.  The 
 *  cone is aligned along the local Z-axis and takes as arguments
 *  the radius and height of the cone.
 */

#ifndef COLLISION_DETECTION_COLLISION_OBJECT_CONE_H
#define COLLISION_DETECTION_COLLISION_OBJECT_CONE_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision_detection/collision_object.h>

namespace collision_detection {
  class Collision_Object_Cone : public Collision_Object {
  public:
    Collision_Object_Cone( std::string id );
    Collision_Object_Cone( std::string id, double radius, double height );
    Collision_Object_Cone( std::string id, double radius, double height, Eigen::Vector3f position, Eigen::Vector4f orientation );
    ~Collision_Object_Cone();

    virtual void set_transform( Eigen::Vector3f position, Eigen::Vector4f orientation );

    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

  protected:
    btCollisionObject   _bt_collision_object;
    btConeShapeZ        _bt_cone_shape;
  private:

  };
}

#endif /* COLLISION_DETECTION_COLLISION_OBJECT_CONE_H */
