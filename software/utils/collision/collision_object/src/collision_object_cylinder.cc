#include <collision/collision_object_cylinder.h>

using namespace std;
using namespace Eigen;
using namespace collision;

/**
 * Collision_Object_Cylinder
 * class constructor
 */
Collision_Object_Cylinder::
Collision_Object_Cylinder( string id ) : Collision_Object( id ),
                                        _bt_collision_object(),
                                        _bt_cylinder_shape( btVector3( 0.5, 0.0, 0.5 ) ){
  _bt_collision_object.setCollisionShape( &_bt_cylinder_shape );
}

/**
 * Collision_Object_Cylinder
 * class constructor with id and dimension arguments
 */
Collision_Object_Cylinder::
Collision_Object_Cylinder( string id,
                            double radius,
                            double height ) : Collision_Object( id ),
                                              _bt_collision_object(),
                                              _bt_cylinder_shape( btVector3( radius, 0.0, height ) ){
  _bt_collision_object.setCollisionShape( &_bt_cylinder_shape );
}

/**
 * Collision_Object_Cylinder
 * class constructor with id, dimension, position, and orientation arguments
 */
Collision_Object_Cylinder::
Collision_Object_Cylinder( string id,
                            double radius,
                            double height,
                            Vector3f position,
                            Vector4f orientation ) : Collision_Object( id ),
                                                    _bt_collision_object(),
                                                    _bt_cylinder_shape( btVector3( radius, 0.0, height ) ){
  set_transform( position, orientation );
  _bt_collision_object.setCollisionShape( &_bt_cylinder_shape );
}

/**
 * ~Collision_Object_Cylinder
 * class destructor
 */
Collision_Object_Cylinder::
~Collision_Object_Cylinder(){

}

/** 
 * set_transform
 * sets the world-frame position and orientation of the collision object
 */
void
Collision_Object_Cylinder::
set_transform( const Vector3f position,
                const Vector4f orientation ){
  _bt_collision_object.setWorldTransform( btTransform( btQuaternion( orientation.x(), orientation.y(), orientation.z(), orientation.w() ),
                                                        btVector3( position.x(), position.y(), position.z() ) ) );
  return;
}

/** 
 * bt_collision_objects 
 * returns a std::vector of btCollisionObject pointers
 */
vector< btCollisionObject* >
Collision_Object_Cylinder::
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
Collision_Object_Cylinder::
bt_collision_objects( void )const{
  vector< const btCollisionObject* > bt_collision_objects;
  bt_collision_objects.push_back( &_bt_collision_object );
  return bt_collision_objects;
}
