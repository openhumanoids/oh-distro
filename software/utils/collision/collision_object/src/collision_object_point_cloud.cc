#include <string.h>
#include <stdlib.h>
#include <stdexcept>

#include <collision/collision_object_sphere.h>
#include <collision/collision_object_point_cloud.h>


using namespace std;
using namespace Eigen;
using namespace KDL;
using namespace collision;

/**
 * Collision_Object_Point_Cloud
 * class constructor
 */
Collision_Object_Point_Cloud::
Collision_Object_Point_Cloud( string id,
                              unsigned int maxPoints ) : Collision_Object( id ),
                                                          _max_points( maxPoints ),
                                                          _points(),
                                                          _collision_objects() {
  _load_collision_objects();
}



/**orientation
   get the world-frame orientation of the collision objects*/ 
Vector4f Collision_Object_Point_Cloud::orientation() const
{
  throw std::runtime_error("Not Implemented: collision_object_point_cloud.cc --> orientation()");
}

/**position
   get the world-frame orientation of the collision objects*/ 
Vector3f Collision_Object_Point_Cloud::position() const
{
  throw std::runtime_error("Not Implemented: collision_object_point_cloud.cc --> position()");
}



/** 
 * set_transform
 * sets the world-frame position and orientation of the collision object
 */
void
Collision_Object_Point_Cloud::
set_transform( const Vector3f position,
                const Vector4f orientation )
{
  throw std::runtime_error("Not Implemented: collision_object_point_cloud.cc --> set_transform");
}

void
Collision_Object_Point_Cloud::
set_transform( const Frame& transform ){
  throw std::runtime_error("Not Implemented: collision_object_point_cloud.cc --> set_transform");
}

/**
 * Collision_Object_Point_Cloud
 * copy constructor
 */
Collision_Object_Point_Cloud::
Collision_Object_Point_Cloud( const Collision_Object_Point_Cloud& other ): Collision_Object( other ),
                                                                            _max_points( other._max_points ),
                                                                            _points( other._points ),
                                                                            _collision_objects() {
  _load_collision_objects();
}   

/**
 * ~Collision_Object_Point_Cloud
 * class destructor
 */
Collision_Object_Point_Cloud::
~Collision_Object_Point_Cloud(){
  for( unsigned int i = 0; i < _collision_objects.size(); i++ ){
    if( _collision_objects[ i ] != NULL ){
      delete _collision_objects[ i ];
      _collision_objects[ i ] = NULL;
    }
  }
  _collision_objects.clear();
}

/** 
 * set
 * sets the point cloud to the points
 */
void
Collision_Object_Point_Cloud::
set( vector< Vector3f >& points ){
  _points = points;
  for( unsigned int i = 0; i < _points.size(); i++ ){
    if( i < _collision_objects.size() ){
      _collision_objects[ i ]->set_position( _points[ i ] );
      _collision_objects[ i ]->set_active( true );
    } else {
      cout << "adding too many points to the point cloud collision object" << endl;
    }
  }
  for( unsigned int i = _points.size(); i < _max_points; i++ ){
    if( i < _collision_objects.size() ){
      _collision_objects[ i ]->set_active( false );
    }
  }
  return;
}

/**
 * matches_uid
 */
Collision_Object*
Collision_Object_Point_Cloud::
matches_uid( unsigned int uid ){
  for( unsigned int i = 0; i < _collision_objects.size(); i++ ){
    vector< btCollisionObject* > bt_collision_object_vector = _collision_objects[ i ]->bt_collision_objects();
    for( unsigned int j = 0; j < bt_collision_object_vector.size(); j++ ){
      if( bt_collision_object_vector[ j ]->getBroadphaseHandle()->getUid() == uid ){
        return _collision_objects[ i ];
      }
    }
  }
  return NULL;
}

/** 
 * bt_collision_objects 
 * returns a std::vector of btCollisionObject pointers
 */
vector< btCollisionObject* >
Collision_Object_Point_Cloud::
bt_collision_objects( void ){
  vector< btCollisionObject* > bt_collision_objects;
  for( unsigned int i = 0; i < _collision_objects.size(); i++ ){
    if( _collision_objects[ i ] != NULL ){
      for( unsigned int j = 0; j < _collision_objects[ i ]->bt_collision_objects().size(); j++ ){
        bt_collision_objects.push_back( _collision_objects[ i ]->bt_collision_objects()[ j ] );
      } 
    }
  }
  return bt_collision_objects;
}

/**
 * bt_collision_objects
 * return a std::vector of const btCollisionObject pointers
 */
vector< const btCollisionObject* >
Collision_Object_Point_Cloud::
bt_collision_objects( void )const{
  vector< const btCollisionObject* > bt_collision_objects;
  for( unsigned int i = 0; i < _collision_objects.size(); i++ ){
    if( _collision_objects[ i ] != NULL ){
      for( unsigned int j = 0; j < _collision_objects[ i ]->bt_collision_objects().size(); j++ ){
        bt_collision_objects.push_back( _collision_objects[ i ]->bt_collision_objects()[ j ] );
      } 
    }
  }
  return bt_collision_objects;
}

/**
 * _load_collision_objects
 * iterates through all of the links and loads collision objects based on the link type
 */
void
Collision_Object_Point_Cloud::
_load_collision_objects( void ){
  while( _collision_objects.size() != _max_points ){
    char buffer[ 80 ];
    sprintf( buffer, "%06d", ( int )( _collision_objects.size() ) );
    _collision_objects.push_back( new Collision_Object_Sphere( string( buffer ), 0.01 ) );
  }
  return;
}
