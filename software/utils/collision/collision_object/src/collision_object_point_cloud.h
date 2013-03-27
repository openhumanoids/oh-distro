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

#ifndef COLLISION_COLLISION_OBJECT_POINT_CLOUD_H
#define COLLISION_COLLISION_OBJECT_POINT_CLOUD_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <lcmtypes/drc_lcmtypes.hpp>
#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_Point_Cloud : public Collision_Object {
  public:
    Collision_Object_Point_Cloud( std::string id, unsigned int maxPoints = 1000 );
    Collision_Object_Point_Cloud( const Collision_Object_Point_Cloud& other );
    ~Collision_Object_Point_Cloud();

    virtual void set( std::vector< Eigen::Vector3f >& points );

    virtual Collision_Object * matches_uid( unsigned int uid );


    virtual Eigen::Vector3f position( void )const ;
    virtual Eigen::Vector4f orientation( void )const;
    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual void set_transform( const KDL::Frame& transform );
 
    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

  protected:
    void _load_collision_objects( void );
    unsigned int _max_points;
    std::vector< Eigen::Vector3f > _points;  
    std::vector< Collision_Object* > _collision_objects;

  private:

  };
}

#endif /* COLLISION_COLLISION_OBJECT_POINT_CLOUD_H */
