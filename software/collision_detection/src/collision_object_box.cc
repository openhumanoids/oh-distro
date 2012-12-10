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
 * Collision_Object_Box
 * copy constructor 
 */
Collision_Object_Box::
Collision_Object_Box( const Collision_Object_Box& other ) : Collision_Object( other ),
                                                            _bt_collision_object(),
                                                            _bt_box_shape( btVector3( other.bt_box_shape().getImplicitShapeDimensions().x() + other.bt_box_shape().getMargin(), other.bt_box_shape().getImplicitShapeDimensions().y() + other.bt_box_shape().getMargin(), other.bt_box_shape().getImplicitShapeDimensions().z() + other.bt_box_shape().getMargin() ) ){
  set_transform( other.position(), other.orientation() );
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
  _bt_collision_object.setWorldTransform( btTransform( btQuaternion( orientation.x(), orientation.y(), orientation.z(), orientation.w() ),
                                                        btVector3( position.x(), position.y(), position.z() ) ) );
  return;
}

Vector3f
Collision_Object_Box::
position( void )const{
  Vector3f position( _bt_collision_object.getWorldTransform().getOrigin().getX(),
                      _bt_collision_object.getWorldTransform().getOrigin().getY(),
                      _bt_collision_object.getWorldTransform().getOrigin().getZ() );
  return position;
}

Vector4f
Collision_Object_Box::
orientation( void )const{
  Vector4f orientation( _bt_collision_object.getWorldTransform().getRotation().getX(),
                        _bt_collision_object.getWorldTransform().getRotation().getY(),
                        _bt_collision_object.getWorldTransform().getRotation().getZ(),
                        _bt_collision_object.getWorldTransform().getRotation().getW() );
  return orientation;
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

/**
 * bt_box_shape
 * return a reference to the bt_box_shape
 */
const btBoxShape&
Collision_Object_Box::
bt_box_shape( void )const{
  return _bt_box_shape;
}

namespace collision_detection {
  ostream&
  operator<<( ostream& out,
              const Collision_Object_Box& other ){
    out << "id:{" << other.id().c_str() << "} ";
    out << "bt_collision_objects[" << other.bt_collision_objects().size() << "]:{";
    for( unsigned int i = 0; i < other.bt_collision_objects().size(); i++ ){
      if( other.bt_collision_objects()[ i ]->getBroadphaseHandle() != NULL ){
        out << other.bt_collision_objects()[ i ]->getBroadphaseHandle()->getUid();
      } else {
        out << "N/A";
      }
      out << ":{pos:(" << other.bt_collision_objects()[ i ]->getWorldTransform().getOrigin().x() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getOrigin().y() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getOrigin().z() << "),(" << other.bt_collision_objects()[ i ]->getWorldTransform().getRotation().getX() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getRotation().getY() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getRotation().getZ() << "," << other.bt_collision_objects()[ i ]->getWorldTransform().getRotation().getW() << ")}";
      out << ",box:(" << other.bt_box_shape().getImplicitShapeDimensions().x() + other.bt_box_shape().getMargin() << "," << other.bt_box_shape().getImplicitShapeDimensions().y() + other.bt_box_shape().getMargin() << "," << other.bt_box_shape().getImplicitShapeDimensions().z() + other.bt_box_shape().getMargin() << ")";
      if( i != ( other.bt_collision_objects().size() - 1 ) ){
        out << ",";
      }
    }
    out << "}";
    return out;
  }
}
