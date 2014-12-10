/**
 * @file collision_object.h
 * @author Thomas Howard
 * @namespace collision_detection
 * 
 * @section DESCRIPTION
 * 
 * A class used to generally describe attributes of
 *   a collision object used by the Collision_Detector
 *   class. 
 */

#ifndef COLLISION_DETECTION_COLLISION_OBJECT_H
#define COLLISION_DETECTION_COLLISION_OBJECT_H

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <btBulletCollisionCommon.h>

namespace collision_detection {
  class Collision_Object {
  public:
    Collision_Object( std::string id );
    Collision_Object( const Collision_Object& other );
    ~Collision_Object();
  
    virtual void set_active( bool active );
    virtual void set_position( const Eigen::Vector3f position );
    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
  
    virtual Collision_Object* matches_uid( unsigned int uid );

    std::string id( void )const;
    virtual Eigen::Vector3f position( void )const;
    virtual Eigen::Vector4f orientation( void )const;
    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

  protected:
    std::string _id;
  private:

  };
  std::ostream& operator<<( std::ostream& out, const Collision_Object& other );
}

#endif /* COLLISION_DETECTION_COLLISION_OBJECT_H */
