/**
 * @file collision_object_gfe.h
 * @author Thomas Howard
 * @namespace collision
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a gfe-shaped collision object.  The 
 *  class constructor takes the radius of the gfe as an argument.
 */

#ifndef COLLISION_DETECTION_COLLISION_OBJECT_GFE_H
#define COLLISION_DETECTION_COLLISION_OBJECT_GFE_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <lcmtypes/drc_lcmtypes.hpp>
#include <kinematics_model/kinematics_model_gfe.h>
#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_GFE : public Collision_Object {
  public:
    Collision_Object_GFE( std::string id );
    Collision_Object_GFE( std::string id, std::string xmlString );
    Collision_Object_GFE( std::string id, std::string xmlString, drc::robot_state_t& robotState );
    Collision_Object_GFE( const Collision_Object_GFE& other );
    ~Collision_Object_GFE();

    virtual void set( drc::robot_state_t& robotState );

    virtual Collision_Object * matches_uid( unsigned int uid );
 
    const kinematics_model::Kinematics_Model_GFE& kinematics_model( void )const; 
    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual Eigen::Vector3f position( void )const;
    virtual Eigen::Vector4f orientation( void )const;

  protected:
    void _load_collision_objects( void );
  
    std::vector< Collision_Object* > _collision_objects;
    kinematics_model::Kinematics_Model_GFE _kinematics_model;

  private:

  };
}

#endif /* COLLISION_DETECTION_COLLISION_OBJECT_GFE_H */
