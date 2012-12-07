#include <collision_detection/collision_object_box.h>

using namespace std;
using namespace Eigen;
using namespace collision_detection;

/**
 * Collision_Object_Box
 * class constructor
 */
Collision_Object_Box::
Collision_Object_Box( string id ) : Collision_Object( id ),
                                    _bt_collision_object(),
                                    _bt_box_shape( btVector3( 0.5, 0.5, 0.5 ) ){
  _bt_collision_object.setCollisionShape( &_bt_box_shape );
}

/**
 * Collision_Object_Box
 * class constructor with id and dimension arguments
 */
Collision_Object_Box::
Collision_Object_Box( string id,
                      Vector3f dims ) : Collision_Object( id ),
                                        _bt_collision_object(),
                                        _bt_box_shape( btVector3( dims.x()/2.0, dims.y()/2.0, dims.z()/2.0 ) ){
  _bt_collision_object.setCollisionShape( &_bt_box_shape );
}

/**
 * Collision_Object_Box
 * copy constructor 
 */
Collision_Object_Box::
Collision_Object_Box( const Collision_Object_Box& other ) : Collision_Object( other ),
                                                            _bt_collision_object(),
                                                            _bt_box_shape( btVector3(0.5, 0.5, 0.5) ){
  //cout << "calling copy constructor" << endl;
  _bt_collision_object.setCollisionShape( &_bt_box_shape );
}

/**
 * Collision_Object_Box
 * class constructor with id, dimension, position, and orientation arguments
 */
Collision_Object_Box::
Collision_Object_Box( string id,
                      Vector3f dims,
                      Vector3f position,
                      Vector4f orientation ) : Collision_Object( id ),
                                                _bt_collision_object(),
                                                _bt_box_shape( btVector3( dims.x()/2.0, dims.y()/2.0, dims.z()/2.0 ) ){
  set_transform( position, orientation );
  _bt_collision_object.setCollisionShape( &_bt_box_shape );
}

/**
 * ~Collision_Object_Box
 * class destructor
 */
Collision_Object_Box::
~Collision_Object_Box(){

}

/** 
 * set_dims
 * sets the dimensions of the box collision object
 */
void
Collision_Object_Box::
set_dims( Vector3f dims ){
  btVector3 half_extents( dims.x()/2.0, dims.y()/2.0, dims.z()/2.0 );
  _bt_box_shape.setSafeMargin( half_extents );
  btVector3 margin( _bt_box_shape.getMargin(), _bt_box_shape.getMargin(), _bt_box_shape.getMargin() );
  _bt_box_shape.setImplicitShapeDimensions( ( half_extents * _bt_box_shape.getLocalScaling() ) - margin );
  return;
}

/** 
 * set_transform
 * sets the world-frame position and orientation of the collision object
 */
void
Collision_Object_Box::
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
Collision_Object_Box::
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
Collision_Object_Box::
bt_collision_objects( void )const{
  vector< const btCollisionObject* > bt_collision_objects;
  bt_collision_objects.push_back( &_bt_collision_object );
  return bt_collision_objects;
}
