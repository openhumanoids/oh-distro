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
    Collision_Object_Point_Cloud( std::string id, unsigned int maxPoints = 1000, double pointRadius = 0.04 );
    ~Collision_Object_Point_Cloud();
    Collision_Object_Point_Cloud( const Collision_Object_Point_Cloud& other );

    virtual void set( std::vector< Eigen::Vector3f >& points );

    virtual Collision_Object * matches_uid( unsigned int uid );

    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual void set_transform( const KDL::Frame& transform );

    inline const double& point_radius( void )const{ return _point_radius; };
 
  protected:
    void _load_collision_objects( double pointRadius );
    unsigned int _max_points;
    double _point_radius;
    std::vector< Eigen::Vector3f > _points;  
    std::vector< Collision_Object* > _collision_objects;

  private:

  };
}

#endif /* COLLISION_COLLISION_OBJECT_POINT_CLOUD_H */
