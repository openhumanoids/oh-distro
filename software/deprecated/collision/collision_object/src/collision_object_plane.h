/**
 * @file collision_object_plane.h
 * @author Thomas Howard
 * @namespace collision
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a plane-shaped collision object.  The
 *   constructor takes as arguments a Eigen::Vector3f where the
 *   x, y, and z indices equal the length, width, and height of
 *   the plane.  
 */

#ifndef COLLISION_COLLISION_OBJECT_PLANE_H
#define COLLISION_COLLISION_OBJECT_PLANE_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <Eigen/Dense>
#include <kdl/tree.hpp>

#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_Plane : public Collision_Object {
  public:
    Collision_Object_Plane( std::string id = "N/A", double height = 0.0, Eigen::Vector3f position = Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f orientation = Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
    Collision_Object_Plane( std::string id, double height, const KDL::Frame& offset, const KDL::Frame& transform = KDL::Frame::Identity() );
    Collision_Object_Plane( const Collision_Object_Plane& other );
    ~Collision_Object_Plane();

    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual void set_transform( const KDL::Frame& transform );
    
    const btStaticPlaneShape& bt_plane_shape( void )const;

  protected:
    btCollisionObject _bt_collision_object;
    btStaticPlaneShape        _bt_plane_shape;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Collision_Object_Plane& other );
}

#endif /* COLLISION_COLLISION_OBJECT_PLANE_H */
