/**
 * @file collision_object_sphere.h
 * @author Thomas Howard
 * @namespace collision_detection
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a sphere-shaped collision object.  The 
 *  class constructor takes the radius of the sphere as an argument.
 */

#ifndef COLLISION_DETECTION_COLLISION_OBJECT_SPHERE_H
#define COLLISION_DETECTION_COLLISION_OBJECT_SPHERE_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision_detection/collision_object.h>

namespace collision_detection {
  class Collision_Object_Sphere : public Collision_Object {
  public:
    Collision_Object_Sphere( std::string id );
    Collision_Object_Sphere( std::string id, double radius );
    Collision_Object_Sphere( std::string id, double radius, Eigen::Vector3f position, Eigen::Vector4f orientation );
    Collision_Object_Sphere( const Collision_Object_Sphere& other );
    ~Collision_Object_Sphere();

    virtual void set_active( bool active );
    virtual void set_position( const Eigen::Vector3f position );
    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
  
    virtual Eigen::Vector3f position( void )const;
    virtual Eigen::Vector4f orientation( void )const;
    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

  protected:
    btCollisionObject   _bt_collision_object;
    btSphereShape       _bt_sphere_shape;
  private:

  };
}

#endif /* COLLISION_DETECTION_COLLISION_OBJECT_SPHERE_H */
