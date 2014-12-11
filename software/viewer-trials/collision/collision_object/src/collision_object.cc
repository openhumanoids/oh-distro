#include <collision/collision_object.h>

using namespace std;
using namespace Eigen;
using namespace KDL;
using namespace collision;

/**
 * Collision_Object
 * class constructor
 */
Collision_Object::
Collision_Object( string id, 
                  bool active,
                  const Frame& offset ) : _id( id ), 
                                          _active(active),
                                          _offset( offset ),
                                          _bt_collision_objects(){

}

/**
 * Collision_Object 
 * copy constructor
 */
Collision_Object::
Collision_Object( const Collision_Object& other ) : _id( other._id ),
                                                    _active( other._active ),
                                                    _offset( other._offset ),
                                                    _bt_collision_objects( other._bt_collision_objects ){

}

/**
 * ~Collision_Object
 * class destructor
 */
Collision_Object::
~Collision_Object(){

}

void Collision_Object::set_active( bool active )
{
  _active = active;
}

void
Collision_Object::
set_offset( const KDL::Frame& offset ){
  _offset = offset;
  return;
}

/**
 * matches_uid
 * determines whether the function argument matches any of the uid's in each of
 *   the btCollisionObject classes contained by this Collision_Object class
 */
Collision_Object*
Collision_Object::
matches_uid( unsigned int uid ){
  vector< btCollisionObject* > bt_collision_object_vector = bt_collision_objects();
  for( unsigned int i = 0; i < bt_collision_object_vector.size(); i++ ){
    if( bt_collision_object_vector[ i ]->getBroadphaseHandle()->getUid() == uid ){
      return this;
    }   
  }
  return NULL;
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
vector< btCollisionObject* >&
Collision_Object::
bt_collision_objects( void ){
  return _bt_collision_objects;
}

/**
 * bt_collision_object
 * returns a std::vectorm of const btCollisionObject pointers
 */ 
const vector< btCollisionObject* >&
Collision_Object::
bt_collision_objects( void )const{
  return _bt_collision_objects;
}

/**
 * operator<<
 * ostream operator
 */
namespace collision {
  ostream&
  operator<<( ostream& out,
              const Collision_Object& other ){
    out << "id:{" << other.id().c_str() << "} ";
    out << "bt_collision_objects[" << other.bt_collision_objects().size() << "]:{";
    for( unsigned int i = 0; i < other.bt_collision_objects().size(); i++ ){
      if( other.bt_collision_objects()[ i ] != NULL ){

        if( other.bt_collision_objects()[ i ]->getBroadphaseHandle() != NULL ){
          out << other.bt_collision_objects()[ i ]->getBroadphaseHandle()->getUid();
        } else {
          out << "N/A";
        }

        btTransform bt_transform = other.bt_collision_objects()[ i ]->getWorldTransform();
        out << ":{pos:(" << bt_transform.getOrigin().x() << "," << bt_transform.getOrigin().y() << "," << bt_transform.getOrigin().z() << "),(" << bt_transform.getRotation().getX() << "," << bt_transform.getRotation().getY() << "," << bt_transform.getRotation().getZ() << "," << bt_transform.getRotation().getW() << ")}";
        if( i != ( other.bt_collision_objects().size() - 1 ) ){
          out << ",";
        }

      }
    }
    out << "}";
    return out;
  }
}
