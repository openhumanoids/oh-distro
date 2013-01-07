#include "collision_detection/collision_detector.h"
#include "collision_detection/collision_object_box.h"
#include <algorithm> // using std::find

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
add_collision_object( Collision_Object* collisionObject,
                      short int filterGroup,  
                      short int filterMask ){
  vector< btCollisionObject* > bt_collision_objects = collisionObject->bt_collision_objects();
  for( unsigned int i = 0; i < bt_collision_objects.size(); i++ ){
    _collision_world.addCollisionObject( bt_collision_objects[ i ], filterGroup, filterMask );
  }
  _collision_objects.push_back( collisionObject );
  return;
}

/**
 * clear_collision_object
 * removes one collision object from the collision world
 */
void
Collision_Detector::
clear_collision_object(Collision_Object* collisionObject){
  
    vector< btCollisionObject* > bt_collision_objects = collisionObject->bt_collision_objects();
    for( unsigned int j = 0; j < bt_collision_objects.size(); j++ ){
      _collision_world.removeCollisionObject( bt_collision_objects[ j ] );
    } 
    
    std::vector<Collision_Object* >::iterator found;
    found = std::find(_collision_objects.begin(), _collision_objects.end(), collisionObject);
    if(found!=_collision_objects.end()) {
     _collision_objects.erase(found);
    }
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
    btPersistentManifold * manifold = _collision_world.getDispatcher()->getManifoldByIndexInternal( i );
    if( manifold->getNumContacts() > 0 ){
      num_collisions += manifold->getNumContacts();
    }
  }
  return num_collisions;
}

/** 
 * get_collisions
 * performs collision detection and returns a vector of collisions
 */
vector< Collision >
Collision_Detector::
get_collisions( void ){
  vector< Collision > collisions;
  _collision_world.performDiscreteCollisionDetection();
  for( unsigned int i = 0; i < _collision_world.getDispatcher()->getNumManifolds(); i++ ){
    btPersistentManifold * manifold = _collision_world.getDispatcher()->getManifoldByIndexInternal( i );
    if( manifold->getNumContacts() > 0 ){
      Collision_Object * first_collision_object = find_collision_object_by_uid( manifold->getBody0()->getBroadphaseHandle()->getUid() );
      Collision_Object * second_collision_object = find_collision_object_by_uid( manifold->getBody1()->getBroadphaseHandle()->getUid() );
      if( ( first_collision_object != NULL ) && ( second_collision_object != NULL ) ){ 
        Collision collision( first_collision_object->id(), second_collision_object->id() );
        for( unsigned int j = 0; j < manifold->getNumContacts(); j++ ){
          btVector3 contact_point = manifold->getContactPoint( j ).getPositionWorldOnA();
          collision.add_contact_point( Vector3f( contact_point.x(), contact_point.y(), contact_point.z() ) );
        }
        collisions.push_back( collision );
      }
    }
  }
  return collisions;
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
  _collision_world.updateAabbs();
  _collision_world.rayTest( bt_from, bt_to, result );
 
  collisionObject = NULL;
  if( result.hasHit() ){
    collisionObject = find_collision_object_by_uid( result.m_collisionObject->getBroadphaseHandle()->getUid() );
  }

  return;
}

/**
 * find_collision_object_by_uid
 * searches through all of the uid's to determine the Collision_Object that it belongs to
 */
Collision_Object*
Collision_Detector::
find_collision_object_by_uid( int uid ){
  Collision_Object * collision_object = NULL;
  for( unsigned int i = 0; i < _collision_objects.size(); i++ ){
    Collision_Object * tmp = _collision_objects[ i ]->matches_uid( uid );
    if( tmp != NULL ){
      collision_object = tmp;
    }
  }
  return collision_object;
}

vector< Collision_Object* >
Collision_Detector::
collision_objects( void )const{
  return _collision_objects;
}

btCollisionWorld&
Collision_Detector::
bt_collision_world( void ){
  return _collision_world;
}

const btCollisionWorld&
Collision_Detector::
bt_collision_world( void )const{
  return _collision_world;
} 

/**
 * operator<<
 * ostream operator function
 */
namespace collision_detection {
  ostream&
  operator<<( ostream& out,
              const Collision_Detector& other ){
    vector< Collision_Object* > collision_objects = other.collision_objects();
    return out;
  }
}
