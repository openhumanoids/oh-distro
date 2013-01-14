/**
 * @file collision_detector.h
 * @author Thomas Howard
 * @namespace collision
 * 
 * @section DESCRIPTION
 * 
 * A class used to evaluate collisions between objects
 *   and ray intersections with objects in a scene.  The
 *   most common use case of this class would involve adding
 *   groups of Collision_Object classes to the scene and 
 *   evaluating intersections between them and rays cast
 *   into the scene.
 */

#ifndef COLLISION_COLLISION_DETECTOR_H
#define COLLISION_COLLISION_DETECTOR_H

#include <iostream>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision/collision.h>
#include <collision/collision_object.h>

#ifdef __cplusplus
extern "C" {
#endif

namespace collision {
  typedef enum {
    COLLISION_DETECTOR_GROUP_1 = 1 << 0,
    COLLISION_DETECTOR_GROUP_2 = 1 << 1, 
    COLLISION_DETECTOR_GROUP_3 = 1 << 2, 
    COLLISION_DETECTOR_GROUP_4 = 1 << 3, 
    COLLISION_DETECTOR_GROUP_5 = 1 << 4,
    NUM_COLLISION_DETECTOR_GROUPS 
  } collision_group_t;

  class Collision_Detector {
  public:
    Collision_Detector();
    ~Collision_Detector();

    void add_collision_object( Collision_Object * collisionObject, short int filterGroup = btBroadphaseProxy::DefaultFilter, short int filterMask = btBroadphaseProxy::AllFilter );
    void clear_collision_object( Collision_Object* collisionObject );
    void clear_collision_objects( void );
    
    void update_collision_objects( void );
      
    unsigned int num_collisions( void );
    std::vector< Collision > get_collisions( void );

    void ray_test( Eigen::Vector3f from, Eigen::Vector3f to, Collision_Object*& collisionObject );
    Collision_Object* find_collision_object_by_uid( int uid );

    std::vector< Collision_Object* > collision_objects( void )const;
    btCollisionWorld& bt_collision_world( void );
    const btCollisionWorld& bt_collision_world( void )const;
  
  protected:
    btDefaultCollisionConfiguration _collision_configuration;
    btCollisionDispatcher _collision_dispatcher;
    btDbvtBroadphase _collision_broadphase;
    btCollisionWorld _collision_world;

    std::vector< Collision_Object* > _collision_objects;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const Collision_Detector& other );
}

#ifdef __cplusplus
}
#endif

#endif /* COLLISION_COLLISION_DETECTOR_H */
