/**
 * @file collision_object_sphere.h
 * @author Thomas Howard
 * @namespace collision
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a sphere-shaped collision object.  The 
 *  class constructor takes the radius of the sphere as an argument.
 */

#ifndef COLLISION_COLLISION_OBJECT_SPHERE_H
#define COLLISION_COLLISION_OBJECT_SPHERE_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_Sphere : public Collision_Object {
  public:
    Collision_Object_Sphere( std::string id = "N/A", double radius = 1.0, Eigen::Vector3f position = Eigen::Vector3f(), Eigen::Vector4f orientation = Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
    Collision_Object_Sphere( std::string id, double radius, const KDL::Frame& offset, const KDL::Frame& transform = KDL::Frame::Identity() );
    Collision_Object_Sphere( const Collision_Object_Sphere& other );
    ~Collision_Object_Sphere();

    virtual void set_active( bool active );
    virtual void set_position( const Eigen::Vector3f position );
    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual void set_transform( const KDL::Frame& transform ); 
 
  protected:
    btCollisionObject   _bt_collision_object;
    btSphereShape       _bt_sphere_shape;
  private:

  };
}

#endif /* COLLISION_COLLISION_OBJECT_SPHERE_H */
