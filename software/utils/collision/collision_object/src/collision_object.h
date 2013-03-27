/**
 * @file collision_object.h
 * @author Thomas Howard
 * @namespace collision
 * 
 * @section DESCRIPTION
 * 
 * A class used to generally describe attributes of
 *   a collision object used by the Collision_Detector
 *   class. 
 */

#ifndef COLLISION_COLLISION_OBJECT_H
#define COLLISION_COLLISION_OBJECT_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <kdl/tree.hpp>
#include <btBulletCollisionCommon.h>

namespace collision {
  class Collision_Object {
  public:
    Collision_Object( std::string id, bool active = true, const KDL::Frame& offset = KDL::Frame::Identity() );
    Collision_Object( const Collision_Object& other );
    virtual ~Collision_Object();
  
    virtual void set_active( bool active );
    virtual void set_position( const Eigen::Vector3f position );
    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation ) = 0;
    virtual void set_transform( const KDL::Frame& transform ) = 0;
    virtual void set_offset( const KDL::Frame& offset ); 
 
    virtual Collision_Object* matches_uid( unsigned int uid );

    std::string id( void )const;
    virtual Eigen::Vector3f position( void )const = 0;
    virtual Eigen::Vector4f orientation( void )const = 0;
    virtual KDL::Frame offset( void )const;
    virtual std::vector< btCollisionObject* > bt_collision_objects( void ) = 0;    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const = 0;

  protected:
    std::string _id;
    bool _active;
    KDL::Frame _offset;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Collision_Object& other );
}

#endif /* COLLISION_COLLISION_OBJECT_H */
