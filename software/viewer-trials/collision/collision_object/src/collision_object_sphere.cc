#include <collision/collision_object_sphere.h>

using namespace std;
using namespace Eigen;
using namespace KDL;
using namespace collision;

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
  _bt_collision_objects.push_back( &_bt_collision_object );
}

/**
 * Collision_Object_Sphere
 * class constructor with id, dimension, position, and orientation arguments
 */
Collision_Object_Sphere::
Collision_Object_Sphere( string id,
                          double radius,
                          const Frame& offset,
                          const Frame& transform ) : Collision_Object( id, true, offset ),
                                                      _bt_collision_object(),
                                                      _bt_sphere_shape( (btScalar)(radius) ){
  set_transform( transform );
  _bt_collision_object.setCollisionShape( &_bt_sphere_shape );
  _bt_collision_objects.push_back( &_bt_collision_object );
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

void
Collision_Object_Sphere::
set_active( bool active ){
  if( !active ){
    _bt_collision_object.setWorldTransform( btTransform( btQuaternion( 0.0, 0.0, 0.0, 1.0 ),
                                                          btVector3( 10000.0, 10000.0, 10000.0 ) ) );
  }
  return;
}

void
Collision_Object_Sphere::
set_position( const Vector3f position ){
  _bt_collision_object.setWorldTransform( btTransform( btQuaternion( 0.0, 0.0, 0.0, 1.0 ),
                                                        btVector3( position.x(), position.y(), position.z() ) ) );
  return;
}

/** 
 * set_transform
 * sets the world-frame position and orientation of the collision object
 */
void
Collision_Object_Sphere::
set_transform( const Vector3f position,
                const Vector4f orientation ){
  _bt_collision_object.setWorldTransform( btTransform( btQuaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()),
                                                        btVector3( position.x(), position.y(), position.z() ) ) );
  return;
}

void
Collision_Object_Sphere::
set_transform( const Frame& transform ){
  Frame origin = transform * _offset;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qs = 0.0;
  origin.M.GetQuaternion( qx, qy, qz, qs );
  _bt_collision_object.setWorldTransform( btTransform( btQuaternion( qx, qy, qz, qs ), btVector3( origin.p[0], origin.p[1], origin.p[2] ) ) );
  return;
}
