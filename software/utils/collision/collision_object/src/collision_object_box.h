/**
 * @file collision_object_box.h
 * @author Thomas Howard
 * @namespace collision
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a box-shaped collision object.  The
 *   constructor takes as arguments a Eigen::Vector3f where the
 *   x, y, and z indices equal the length, width, and height of
 *   the box.  
 */

#ifndef COLLISION_COLLISION_OBJECT_BOX_H
#define COLLISION_COLLISION_OBJECT_BOX_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>
#include <kdl/tree.hpp>

#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_Box : public Collision_Object {
  public:
    Collision_Object_Box( std::string id = "N/A", Eigen::Vector3f dims = Eigen::Vector3f( 1.0, 1.0, 1.0 ), Eigen::Vector3f position = Eigen::Vector3f( 0.0, 0.0, 0.0 ), Eigen::Vector4f orientation = Eigen::Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
    Collision_Object_Box( std::string id, Eigen::Vector3f dims, const KDL::Frame& offset, const KDL::Frame& transform = KDL::Frame::Identity() );
    Collision_Object_Box( const Collision_Object_Box& other );
    ~Collision_Object_Box();

    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual void set_transform( const KDL::Frame& transform );
    
    const btBoxShape& bt_box_shape( void )const;

  protected:
    btCollisionObject _bt_collision_object;
    btBoxShape        _bt_box_shape;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Collision_Object_Box& other );
}

#endif /* COLLISION_COLLISION_OBJECT_BOX_H */
