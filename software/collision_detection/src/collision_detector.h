#ifndef COLLISION_DETECTION_COLLISION_DETECTOR_H
#define COLLISION_DETECTION_COLLISION_DETECTOR_H

#include <iostream>
#include <vector>
#include <btBulletCollisionCommon.h>
#include <Eigen/Dense>

#include <collision_detection/collision_object.h>

#ifdef __cplusplus
extern "C" {
#endif

namespace collision_detection {
  class Collision_Detector {
  public:
    Collision_Detector();
    ~Collision_Detector();

    void add_collision_object( Collision_Object * collisionObject );

    unsigned int num_collisions( void );
    void ray_test( Eigen::Vector3f from, Eigen::Vector3f to, Collision_Object*& collisionObject );

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

#endif /* COLLISION_DETECTION_COLLISION_DETECTOR_H */
