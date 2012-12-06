#include "collision_detection/collision_detector.h"

using namespace std;
using namespace Eigen;
using namespace collision_detection;

/**
 * Collision_Detector()
 * class constructor
 */
Collision_Detector::
Collision_Detector() : _collision_configuration(),
                        _collision_dispatcher( &_collision_configuration ),
                        _collision_broadphase(),
                        _collision_world( &_collision_dispatcher,
                                          &_collision_broadphase,
                                          &_collision_configuration ),
                        _collision_objects(){
}

/** 
 * ~Collision_Detector
 * class destructor
 */
Collision_Detector::
~Collision_Detector(){

}

/** 
 * add_collision_object
 * adds a collision object to the collision world
 */
void
Collision_Detector::
add_collision_object( Collision_Object* collisionObject ){
  vector< btCollisionObject* > bt_collision_objects = collisionObject->bt_collision_objects();
  for( unsigned int i = 0; i < bt_collision_objects.size(); i++ ){
    _collision_world.addCollisionObject( bt_collision_objects[ i ] );
  }
  _collision_objects.push_back( collisionObject );
  return;
}

/**
 * clear_collision_objects
 * removes all of the collision objects from the collision world
 */
void
Collision_Detector::
clear_collision_objects( void ){
  for( unsigned int i = 0; i < _collision_objects.size(); i++ ){
    vector< btCollisionObject* > bt_collision_objects = _collision_objects[ i ]->bt_collision_objects();
    for( unsigned int j = 0; j < bt_collision_objects.size(); j++ ){
      _collision_world.removeCollisionObject( bt_collision_objects[ j ] );
    } 
  } 
  _collision_objects.clear();
  return;
}

/** 
 * num_collisions
 * performs collision detection and returns the number of object collisions
  */
unsigned int 
Collision_Detector::
num_collisions( void ){
  _collision_world.performDiscreteCollisionDetection();
  unsigned int num_collisions = 0;
  for( unsigned int i = 0; i < _collision_world.getDispatcher()->getNumManifolds(); i++ ){
    num_collisions += _collision_world.getDispatcher()->getManifoldByIndexInternal( i )->getNumContacts();
  }
  return num_collisions;
}

/**
 * ray_test
 * performs a ray intersection test between two points and returns a pointer
 *   to the Collision_Object intersected
 */
void
Collision_Detector::
ray_test( Vector3f from,
          Vector3f to,
          Collision_Object*& collisionObject ){
  btVector3 bt_from( from.x(), from.y(), from.z() );
  btVector3 bt_to( to.x(), to.y(), to.z() );
  btCollisionWorld::ClosestRayResultCallback result( bt_from, bt_to );
  _collision_world.rayTest( bt_from, bt_to, result );
    
  collisionObject = NULL;
  if( result.hasHit() ){
    for( unsigned int i = 0; i < _collision_objects.size(); i++ ){
      if( _collision_objects[ i ]->matches_uid( result.m_collisionObject->getBroadphaseHandle()->getUid() ) ){
        collisionObject = _collision_objects[ i ];
        break;
      }
    }
  }

  return;
}

/**
 * operator<<
 * ostream operator function
 */
namespace collision_detection {
  ostream&
  operator<<( ostream& out,
              const Collision_Detector& other ){
    return out;
  }
}
