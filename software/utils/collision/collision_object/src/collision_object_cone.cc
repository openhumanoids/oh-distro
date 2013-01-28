#include <stdexcept>
#include <collision/collision_object_cone.h>

using namespace std;
using namespace Eigen;
using namespace collision;

/**
 * Collision_Object_Cone
 * class constructor
 */
Collision_Object_Cone::
Collision_Object_Cone( string id ) : Collision_Object( id ),
                                    _bt_collision_object(),
                                    _bt_cone_shape( 0.5, 0.5 ){
  _bt_collision_object.setCollisionShape( &_bt_cone_shape );
}

/**
 * Collision_Object_Cone
 * class constructor with id and dimension arguments
 */
Collision_Object_Cone::
Collision_Object_Cone( string id,
                        double radius,
                        double height ) : Collision_Object( id ),
                                          _bt_collision_object(),
                                          _bt_cone_shape( (btScalar)(radius), (btScalar)(height) ){
  _bt_collision_object.setCollisionShape( &_bt_cone_shape );
}

/**
 * Collision_Object_Cone
 * class constructor with id, dimension, position, and orientation arguments
 */
Collision_Object_Cone::
Collision_Object_Cone( string id,
                        double radius,
                        double height,
                        Vector3f position,
                        Vector4f orientation ) : Collision_Object( id ),
                                                _bt_collision_object(),
                                                _bt_cone_shape( (btScalar)(radius), (btScalar)(height) ){
  set_transform( position, orientation );
  _bt_collision_object.setCollisionShape( &_bt_cone_shape );
}

/**
 * ~Collision_Object_Cone
 * class destructor
 */
Collision_Object_Cone::
~Collision_Object_Cone(){

}

/** 
 * set_transform
 * sets the world-frame position and orientation of the collision object
 */
void
Collision_Object_Cone::
set_transform( const Vector3f position,
                const Vector4f orientation ){
  _bt_collision_object.setWorldTransform( btTransform( btQuaternion( orientation.x(), orientation.y(), orientation.z(), orientation.w() ),
                                                        btVector3( position.x(), position.y(), position.z() ) ) );
  return;
}



/**position 
   get the world-frame position of the collision objects*/ 
Vector3f Collision_Object_Cone::position() const
{
  Vector3f position( _bt_collision_object.getWorldTransform().getOrigin().getX(),
                      _bt_collision_object.getWorldTransform().getOrigin().getY(),
                      _bt_collision_object.getWorldTransform().getOrigin().getZ() );
  return position;
}


/**orientation
   get the world-frame orientation of the collision objects*/ 
Vector4f Collision_Object_Cone::orientation() const
{
  throw std::runtime_error("Not Implemented: collision_object_cone.cc --> orientation()");
}



/** 
 * bt_collision_objects 
 * returns a std::vector of btCollisionObject pointers
 */
vector< btCollisionObject* >
Collision_Object_Cone::
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
Collision_Object_Cone::
bt_collision_objects( void )const{
  vector< const btCollisionObject* > bt_collision_objects;
  bt_collision_objects.push_back( &_bt_collision_object );
  return bt_collision_objects;
}
