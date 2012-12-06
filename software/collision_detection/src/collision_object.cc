#include <collision_detection/collision_object.h>

using namespace std;
using namespace Eigen;
using namespace collision_detection;

/**
 * Collision_Object
 * class constructor
 */
Collision_Object::
Collision_Object( string id ) : _id( id ){

}

/**
 * ~Collision_Object
 * class destructor
 */
Collision_Object::
~Collision_Object(){

}

/**
 * set_transform
 * sets the world to object transform with a 3D vector and unit quaternion
 */
void
Collision_Object::
set_transform( Vector3f position,
                Vector4f orientation ){
  return;
}

/**
 * matches_uid
 * determines whether the function argument matches any of the uid's in each of
 *   the btCollisionObject classes contained by this Collision_Object class
 */
bool
Collision_Object::
matches_uid( unsigned int uid ){
  vector< btCollisionObject* > bt_collision_object_vector = bt_collision_objects();
  for( unsigned int i = 0; i < bt_collision_object_vector.size(); i++ ){
    if( bt_collision_object_vector[ i ]->getBroadphaseHandle()->getUid() == uid ){
      return true;
    }   
  }
  return false;
}

/**
 * id
 * returns the id
 */
string
Collision_Object::
id( void )const{
  return _id;
}

/**
 * bt_collision_objects
 * returns a std::vector of btCollisionObject pointers
 */
vector< btCollisionObject* >
Collision_Object::
bt_collision_objects( void ){
  vector< btCollisionObject* > bt_collision_objects;
  return bt_collision_objects;
}

/**
 * bt_collision_object
 * returns a std::vectorm of const btCollisionObject pointers
 */
vector< const btCollisionObject* >
Collision_Object::
bt_collision_objects( void )const{
  vector< const btCollisionObject* > bt_collision_objects;
  return bt_collision_objects;
}

/**
 * operator<<
 * ostream operator
 */
namespace collision_detection {
  ostream&
  operator<<( ostream& out,
              const Collision_Object& other ){
    out << "id:{" << other.id().c_str() << "} ";
    out << "bt_collision_objects[" << other.bt_collision_objects().size() << "]:{";
    for( unsigned int i = 0; i < other.bt_collision_objects().size(); i++ ){
      if( other.bt_collision_objects()[ i ]->getBroadphaseHandle() != NULL ){
        out << other.bt_collision_objects()[ i ]->getBroadphaseHandle()->getUid();
      } else {
        out << "N/A";
      }
      if( i != ( other.bt_collision_objects().size() - 1 ) ){
        out << ",";
      }
    }
    out << "}";
    return out;
  }
}
