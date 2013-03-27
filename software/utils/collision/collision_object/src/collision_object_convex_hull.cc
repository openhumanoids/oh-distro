#include <fstream>
#include <cstring>
#include <collision/collision_object_convex_hull.h>

using namespace std;
using namespace Eigen;
using namespace KDL;
using namespace collision;

/**
 * Collision_Object_Convex_Hull
 * class constructor
 */
Collision_Object_Convex_Hull::
Collision_Object_Convex_Hull( string id ) : Collision_Object( id ),
                                            _bt_collision_object(),
                                            _bt_convex_hull_shape(),
                                            _chull_obj_filename( "N/A" ) {
  _bt_collision_object.setCollisionShape( &_bt_convex_hull_shape );
//  _bt_convex_hull_shape.setMargin( 0.01 );
}

/**
 * Collision_Object_Convex_Hull
 * class constructor with id and dimension arguments
 */
Collision_Object_Convex_Hull::
Collision_Object_Convex_Hull( string id,
                              string chullObjFilename ) : Collision_Object( id ),
                                                  _bt_collision_object(),
                                                  _bt_convex_hull_shape(),
                                                  _chull_obj_filename( chullObjFilename ) {
  _bt_collision_object.setCollisionShape( &_bt_convex_hull_shape );
//  _bt_convex_hull_shape.setMargin( 0.01 );
  _load_chull_obj();
}

/**
 * Collision_Object_Convex_Hull
 * class constructor with id, dimension, position, and orientation arguments
 */
Collision_Object_Convex_Hull::
Collision_Object_Convex_Hull( string id,
                              string chullObjFilename,
                              Vector3f position,
                              Vector4f orientation ) : Collision_Object( id ),
                                                        _bt_collision_object(),
                                                        _bt_convex_hull_shape(),
                                                        _chull_obj_filename( chullObjFilename ) {
  _bt_collision_object.setCollisionShape( &_bt_convex_hull_shape );
//  _bt_convex_hull_shape.setMargin( 0.01 );
  set_transform( position, orientation );
  _load_chull_obj();
}

/**
 * Collision_Object_Convex_Hull
 * copy constructor
 */
Collision_Object_Convex_Hull::
Collision_Object_Convex_Hull( const Collision_Object_Convex_Hull& other ): Collision_Object( other ),
                                                                            _bt_collision_object(),
                                                                            _bt_convex_hull_shape(),
                                                                            _chull_obj_filename( other._chull_obj_filename ) {
  _bt_collision_object.setCollisionShape( &_bt_convex_hull_shape );
//  _bt_convex_hull_shape.setMargin( 0.01 );
  set_transform( other.position(), other.orientation() );
  _load_chull_obj();
}   

/**
 * ~Collision_Object_Convex_Hull
 * class destructor
 */
Collision_Object_Convex_Hull::
~Collision_Object_Convex_Hull(){

}

/**
 * load_chull_obj
 * loads a convex hull obj
 */
bool
Collision_Object_Convex_Hull::
load_chull_obj( std::string chullObjFilename ){
  _chull_obj_filename = chullObjFilename;
  return _load_chull_obj();
}

/** 
 * set_transform
 * sets the world-frame position and orientation of the collision object
 */
void
Collision_Object_Convex_Hull::
set_transform( const Vector3f position,
                const Vector4f orientation ){
  _bt_collision_object.setWorldTransform( btTransform( btQuaternion( orientation.x(), orientation.y(), orientation.z(), orientation.w() ),
                                                        btVector3( position.x(), position.y(), position.z() ) ) );
  return;
}

void
Collision_Object_Convex_Hull::
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

/**
 * chull_obj_filename
 * returns the name of the convex hull obj
 */
string
Collision_Object_Convex_Hull::
chull_obj_filename( void )const{
  return _chull_obj_filename;
}

/**
 * position
 * returns the position of the collision object
 */
Vector3f
Collision_Object_Convex_Hull::
position( void )const{
  Vector3f position( _bt_collision_object.getWorldTransform().getOrigin().getX(),
                      _bt_collision_object.getWorldTransform().getOrigin().getY(),
                      _bt_collision_object.getWorldTransform().getOrigin().getZ() );
  return position;
}

/**
 * orientation
 * returns the orientation of the collision object
 */
Vector4f
Collision_Object_Convex_Hull::
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
Collision_Object_Convex_Hull::
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
Collision_Object_Convex_Hull::
bt_collision_objects( void )const{
  vector< const btCollisionObject* > bt_collision_objects;
  bt_collision_objects.push_back( &_bt_collision_object );
  return bt_collision_objects;
}

/**
 * _load_chull_obj
 * loads a convex hull obj
 */
bool
Collision_Object_Convex_Hull::
_load_chull_obj( void ){
  ifstream infile( _chull_obj_filename.c_str() );
  if( !infile.is_open() ){
    cout << "could not open " << _chull_obj_filename.c_str() << endl;
    return false;    
  } else {
    int index = 0;
    while( infile.good() ){
      string line;
      getline( infile, line );
      size_t found_vertex = line.find( "v " );
      if( found_vertex != string::npos ){
        vector< string > line_elements;
        char * orig = ( char* )( line.c_str() );
        char * tmp = strtok( orig, " " );
        while( tmp != NULL ){
          line_elements.push_back( string( tmp ) );
          tmp = strtok( NULL, " " );
        }
        btVector3 point( atof( line_elements[ 1 ].c_str() ),
                          atof( line_elements[ 2 ].c_str() ),
                          atof( line_elements[ 3 ].c_str() ) );
        _bt_convex_hull_shape.addPoint( point );
      }
    }
    infile.close(); 
    return true;
  }
}

