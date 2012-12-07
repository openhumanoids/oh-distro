#include <collision_detection/collision_object_sphere.h>

using namespace std;
using namespace Eigen;
using namespace collision_detection;

/**
 * Collision_Object_Sphere
 * class constructor
 */
Collision_Object_Sphere::
Collision_Object_Sphere( string id ) : Collision_Object( id ),
                                      _bt_collision_object(),
                                      _bt_sphere_shape( 0.5 ){
  _bt_collision_object.setCollisionShape( &_bt_sphere_shape );
}

/**
 * Collision_Object_Sphere
 * class constructor with id and dimension arguments
 */
Collision_Object_Sphere::
Collision_Object_Sphere( string id,
                          double radius ) : Collision_Object( id ),
                                            _bt_collision_object(),
                                            _bt_sphere_shape( (btScalar)(radius) ){
  _bt_collision_object.setCollisionShape( &_bt_sphere_shape );
}

/**
 * Collision_Object_Sphere
 * class constructor with id, dimension, position, and orientation arguments
 */
Collision_Object_Sphere::
Collision_Object_Sphere( string id,
                          double radius,
                          Vector3f position,
                          Vector4f orientation ) : Collision_Object( id ),
                                                  _bt_collision_object(),
                                                  _bt_sphere_shape( (btScalar)(radius) ){
  set_transform( position, orientation );
  _bt_collision_object.setCollisionShape( &_bt_sphere_shape );
}

/**
 * Collision_Object_Sphere
 * copy constructor
 */
Collision_Object_Sphere::
Collision_Object_Sphere( const Collision_Object_Sphere& other ): Collision_Object( other ),
                                                                  _bt_collision_object(),
                                                                  _bt_sphere_shape( (btScalar)(0.5) ){
  _bt_collision_object.setCollisionShape( &_bt_sphere_shape );
}   

/**
 * ~Collision_Object_Sphere
 * class destructor
 */
Collision_Object_Sphere::
~Collision_Object_Sphere(){

}

/** 
 * set_transform
 * sets the world-frame position and orientation of the collision object
 */
void
Collision_Object_Sphere::
set_transform( Vector3f position,
                Vector4f orientation ){
  _bt_collision_object.setWorldTransform( btTransform( btQuaternion( orientation.w(), orientation.x(), orientation.y(), orientation.z() ),
                                                        btVector3( position.x(), position.y(), position.z() ) ) );
  return;
}

/** 
 * bt_collision_objects 
 * returns a std::vector of btCollisionObject pointers
 */
vector< btCollisionObject* >
Collision_Object_Sphere::
bt_collision_objects( void ){
  vector< btCollisionObject* > bt_collision_objects;
  bt_collision_objects.push_back( &_bt_collision_object );
  return bt_collision_objects;
}

/**
 * bt_collision_objects
 * return a std::vector of const btCollisionObject pointers
 */
vector< const btCollisionObject* >
Collision_Object_Sphere::
bt_collision_objects( void )const{
  vector< const btCollisionObject* > bt_collision_objects;
  bt_collision_objects.push_back( &_bt_collision_object );
  return bt_collision_objects;
}
