#include <collision/collision_object_torus.h>

using namespace std;
using namespace Eigen;
using namespace KDL;
using namespace collision;

/**
 * Collision_Object_Torus
 * class constructor with id, dimension, position, and orientation arguments
 */
Collision_Object_Torus::
Collision_Object_Torus( string id,
                            double majorRadius,
                            double minorRadius,
                            Vector3f position,
                            Vector4f orientation ) : Collision_Object( id ),
                                                    _bt_collision_object(),
                                                    _bt_torus_shape(),
                                                    _bt_cylinder_shape_1( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_2( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_3( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_4( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_5( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_6( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_7( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_8( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_9( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_10( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_11( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_12( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_13( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_14( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_15( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ),
                                                    _bt_cylinder_shape_16( btVector3( minorRadius, majorRadius * M_PI / 16.0, 0.0 ) ){

  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 1.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 1.0 ), majorRadius * sin( M_PI / 8.0 * 1.0 ), 0.0 ) ), &_bt_cylinder_shape_1 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 2.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 2.0 ), majorRadius * sin( M_PI / 8.0 * 2.0 ), 0.0 ) ), &_bt_cylinder_shape_2 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 3.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 3.0 ), majorRadius * sin( M_PI / 8.0 * 3.0 ), 0.0 ) ), &_bt_cylinder_shape_3 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 4.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 4.0 ), majorRadius * sin( M_PI / 8.0 * 4.0 ), 0.0 ) ), &_bt_cylinder_shape_4 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 5.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 5.0 ), majorRadius * sin( M_PI / 8.0 * 5.0 ), 0.0 ) ), &_bt_cylinder_shape_5 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 6.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 6.0 ), majorRadius * sin( M_PI / 8.0 * 6.0 ), 0.0 ) ), &_bt_cylinder_shape_6 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 7.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 7.0 ), majorRadius * sin( M_PI / 8.0 * 7.0 ), 0.0 ) ), &_bt_cylinder_shape_7 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 8.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 8.0 ), majorRadius * sin( M_PI / 8.0 * 8.0 ), 0.0 ) ), &_bt_cylinder_shape_8 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 9.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 9.0 ), majorRadius * sin( M_PI / 8.0 * 9.0 ), 0.0 ) ), &_bt_cylinder_shape_9 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 10.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 10.0 ), majorRadius * sin( M_PI / 8.0 * 10.0 ), 0.0 ) ), &_bt_cylinder_shape_10 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 11.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 11.0 ), majorRadius * sin( M_PI / 8.0 * 11.0 ), 0.0 ) ), &_bt_cylinder_shape_11 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 12.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 12.0 ), majorRadius * sin( M_PI / 8.0 * 12.0 ), 0.0 ) ), &_bt_cylinder_shape_12 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 13.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 13.0 ), majorRadius * sin( M_PI / 8.0 * 13.0 ), 0.0 ) ), &_bt_cylinder_shape_13 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 14.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 14.0 ), majorRadius * sin( M_PI / 8.0 * 14.0 ), 0.0 ) ), &_bt_cylinder_shape_14 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 15.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 15.0 ), majorRadius * sin( M_PI / 8.0 * 15.0 ), 0.0 ) ), &_bt_cylinder_shape_15 );
  _bt_torus_shape.addChildShape( btTransform( btQuaternion( btVector3( 0.0, 0.0, 1.0 ), M_PI / 8.0 * 16.0 ), btVector3( majorRadius * cos( M_PI / 8.0 * 16.0 ), majorRadius * sin( M_PI / 8.0 * 16.0 ), 0.0 ) ), &_bt_cylinder_shape_16 );

  set_transform( position, orientation );
  _bt_collision_object.setCollisionShape( &_bt_torus_shape );
  _bt_collision_objects.push_back( &_bt_collision_object );
}

/**
 * ~Collision_Object_Torus
 * class destructor
 */
Collision_Object_Torus::
~Collision_Object_Torus(){

}

/** 
 * set_transform
 * sets the world-frame position and orientation of the collision object
 */
void
Collision_Object_Torus::
set_transform( const Vector3f position,
                const Vector4f orientation ){
  _bt_collision_object.setWorldTransform( btTransform( btQuaternion( orientation.x(), orientation.y(), orientation.z(), orientation.w() ),
                                                        btVector3( position.x(), position.y(), position.z() ) ) );
  return;
}

void
Collision_Object_Torus::
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
