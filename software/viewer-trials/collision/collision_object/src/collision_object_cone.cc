#include <stdexcept>
#include <collision/collision_object_cone.h>

using namespace std;
using namespace Eigen;
using namespace KDL;
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
  _bt_collision_objects.push_back( &_bt_collision_object );
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
  _bt_collision_objects.push_back( &_bt_collision_object );
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
  _bt_collision_objects.push_back( &_bt_collision_object );
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

void
Collision_Object_Cone::
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
