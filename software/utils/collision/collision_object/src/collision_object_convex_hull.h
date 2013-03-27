/**
 * @file collision_object_convex_hull.h
 * @author Thomas Howard
 * @namespace collision
 * 
 * @section DESCRIPTION
 * 
 * A class used to describe a convex_hull-shaped collision object.  The 
 *  class constructor takes the radius of the convex_hull as an argument.
 */

#ifndef COLLISION_COLLISION_OBJECT_CONVEX_HULL_H
#define COLLISION_COLLISION_OBJECT_CONVEX_HULL_H

#include <iostream>
#include <string>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision/collision_object.h>

namespace collision {
  class Collision_Object_Convex_Hull : public Collision_Object {
  public:
    Collision_Object_Convex_Hull( std::string id );
    Collision_Object_Convex_Hull( std::string id, std::string chullObjFilename );
    Collision_Object_Convex_Hull( std::string id, std::string chullObjFilename, Eigen::Vector3f position, Eigen::Vector4f orientation );
    Collision_Object_Convex_Hull( const Collision_Object_Convex_Hull& other );
    ~Collision_Object_Convex_Hull();

    bool load_chull_obj( std::string chullObjFilename );
    virtual void set_transform( const Eigen::Vector3f position, const Eigen::Vector4f orientation );
    virtual void set_transform( const KDL::Frame& transform );
 
    std::string chull_obj_filename( void )const; 
    virtual Eigen::Vector3f position( void )const;
    virtual Eigen::Vector4f orientation( void )const;
    virtual std::vector< btCollisionObject* > bt_collision_objects( void );    
    virtual std::vector< const btCollisionObject* > bt_collision_objects( void )const;

  protected:
    bool _load_chull_obj( void );

    btCollisionObject   _bt_collision_object;
    btConvexHullShape   _bt_convex_hull_shape;
    std::string         _chull_obj_filename;
  private:

  };
}

#endif /* COLLISION_COLLISION_OBJECT_CONVEX_HULL_H */
