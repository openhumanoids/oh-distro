/**
 * @file collision_object_box.h
 * @author Thomas Howard
 * @namespace collision_detection
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a box-shaped collision object.  The
 *   constructor takes as arguments a Eigen::Vector3f where the
 *   x, y, and z indices equal the length, width, and height of
 *   the box.  
 */

#ifndef COLLISION_DETECTION_COLLISION_OBJECT_BOX_H
#define COLLISION_DETECTION_COLLISION_OBJECT_BOX_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision_detection/collision_object.h>

namespace collision_detection {
  class Collision_Object_Box : public Collision_Object {
  public:
    Collision_Object_Box( std::string id );
    Collision_Object_Box( std::string id, Eigen::Vector3f dims );
    Collision_Object_Box( std::string id, Eigen::Vector3f dims, Eigen::Vector3f position, Eigen::Vector4f orientation );
    Collision_Object_Box( const Collision_Object_Box& other );
    ~Collision_Object_Box();

    void set_dims( Eigen::Vector3f dims );
    virtual void set_transform( Eigen::Vector3f position, Eigen::Vector4f orientation );

    virtual Eigen::Vector3f position( void )const;
    virtual Eigen::Vector4f orientation( void )const;
    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

    const btBoxShape& bt_box_shape( void )const;

  protected:
    btCollisionObject _bt_collision_object;
    btBoxShape        _bt_box_shape;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Collision_Object_Box& other );
}

#endif /* COLLISION_DETECTION_COLLISION_OBJECT_BOX_H */
