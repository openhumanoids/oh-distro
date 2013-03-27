/**
 * @file collision_object_gfe.h
 * @author Thomas Howard
 * @namespace collision_detection
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a gfe-shaped collision object.  The 
 *  class constructor takes the radius of the gfe as an argument.
 */

#ifndef COLLISION_COLLISION_OBJECT_URDF_H
#define COLLISION_COLLISION_OBJECT_URDF_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <lcmtypes/drc_lcmtypes.hpp>
#include <kinematics/kinematics_model_urdf.h>
#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_URDF : public Collision_Object {
  public:
    Collision_Object_URDF( std::string id );
    Collision_Object_URDF( std::string id, std::string urdfFilename );
    Collision_Object_URDF( std::string id, std::string urdfFilename, drc::robot_state_t& robotState );
    Collision_Object_URDF( const Collision_Object_URDF& other );
    ~Collision_Object_URDF();

    virtual void set( void );
    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual void set_transform( const KDL::Frame& transform );

    virtual Collision_Object * matches_uid( unsigned int uid );
 
    const kinematics::Kinematics_Model_URDF& kinematics_model( void )const; 
    virtual Eigen::Vector3f position( void )const;
    virtual Eigen::Vector4f orientation( void )const;
    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

  protected:
    void _load_collision_objects( void );
  
    std::vector< Collision_Object* > _collision_objects;
    kinematics::Kinematics_Model_URDF _kinematics_model;

  private:

  };
}

#endif /* COLLISION_COLLISION_OBJECT_URDF_H */
