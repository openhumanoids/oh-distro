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

#ifndef COLLISION_DETECTION_COLLISION_OBJECT_POINT_CLOUD_H
#define COLLISION_DETECTION_COLLISION_OBJECT_POINT_CLOUD_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <lcmtypes/drc_lcmtypes.hpp>
#include <kinematics_model/kinematics_model_gfe.h>
#include <collision_detection/collision_object.h>

namespace collision_detection {
  class Collision_Object_Point_Cloud : public Collision_Object {
  public:
    Collision_Object_Point_Cloud( std::string id, unsigned int maxPoints = 1000 );
    Collision_Object_Point_Cloud( const Collision_Object_Point_Cloud& other );
    ~Collision_Object_Point_Cloud();

    virtual void set( std::vector< Eigen::Vector3f >& points );

    virtual Collision_Object * matches_uid( unsigned int uid );
 
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

#endif /* COLLISION_DETECTION_COLLISION_OBJECT_POINT_CLOUD_H */
