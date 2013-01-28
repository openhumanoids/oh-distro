/**
 * @file collision_object_cone.h
 * @author Thomas Howard
 * @namespace collision
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a cone-shaped collision object.  The 
 *  cone is aligned along the local Z-axis and takes as arguments
 *  the radius and height of the cone.
 */

#ifndef COLLISION_COLLISION_OBJECT_CONE_H
#define COLLISION_COLLISION_OBJECT_CONE_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_Cone : public Collision_Object {
  public:
    Collision_Object_Cone( std::string id );
    Collision_Object_Cone( std::string id, double radius, double height );
    Collision_Object_Cone( std::string id, double radius, double height, Eigen::Vector3f position, Eigen::Vector4f orientation );
    ~Collision_Object_Cone();

    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );

    virtual Eigen::Vector3f position( void )const;
    virtual Eigen::Vector4f orientation( void )const;

    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

  protected:
    btCollisionObject   _bt_collision_object;
    btConeShapeZ        _bt_cone_shape;
  private:

  };
}

#endif /* COLLISION_COLLISION_OBJECT_CONE_H */
