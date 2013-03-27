#include <path_util/path_util.h>
#include <collision/collision_object_box.h>
#include <collision/collision_object_cylinder.h>
#include <collision/collision_object_sphere.h>
#include <collision/collision_object_convex_hull.h>
#include <collision/collision_object_urdf.h>

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace urdf;
using namespace KDL;
using namespace drc;
using namespace kinematics;
using namespace collision;

/**
 * Collision_Object_URDF
 * class constructor
 */
Collision_Object_URDF::
Collision_Object_URDF( string id ) : Collision_Object( id ),
                                    _collision_objects(),
                                    _kinematics_model() {
  _load_collision_objects();
}

/**
 * Collision_Object_URDF
 * class constructor with id and urdf filename
 */
Collision_Object_URDF::
Collision_Object_URDF( string id,
                      string urdfFilename ) : Collision_Object( id ),
                                              _collision_objects(),
                                              _kinematics_model( urdfFilename ){
  _load_collision_objects();
} 

/**
 * Collision_Object_URDF
 * class constructor with id, dimension, position, and orientation arguments
 */
Collision_Object_URDF::
Collision_Object_URDF( string id,
                      string urdfFilename,
                      robot_state_t& robotState ) : Collision_Object( id ),
                                                    _collision_objects(),
                                                    _kinematics_model( urdfFilename ) {
  _load_collision_objects();
}

/**
 * Collision_Object_URDF
 * copy constructor
 */
Collision_Object_URDF::
Collision_Object_URDF( const Collision_Object_URDF& other ): Collision_Object( other ),
                                                            _collision_objects( other._collision_objects ),
                                                            _kinematics_model( other._kinematics_model ) {
  _load_collision_objects();
}   

/**
 * ~Collision_Object_URDF
 * class destructor
 */
Collision_Object_URDF::
~Collision_Object_URDF(){
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
 * sets the kinematics model to the robot state
 */
void
Collision_Object_URDF::
set( void ){
  return;
}

void 
Collision_Object_URDF::
set_transform( const Eigen::Vector3f position, 
                const Eigen::Vector4f orientation ){
  return;
}

void
Collision_Object_URDF::
set_transform( const Frame& transform ){
  return;
}

/**
 * matches_uid
 */
Collision_Object*
Collision_Object_URDF::
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
 * kinematics_model
 * returns a reference to the Kinematics_Model class
 */
const Kinematics_Model_URDF&
Collision_Object_URDF::
kinematics_model( void )const{
  return _kinematics_model;
}

/**
 * position
 * returns the position of the collision object
 */
Vector3f
Collision_Object_URDF::
position( void )const{
  Vector3f position;
  return position;
}

/**
 * orientation
 * returns the orientation of the collision object
 */
Vector4f
Collision_Object_URDF::
orientation( void )const{
  Vector4f orientation;
  return orientation;
}

/** 
 * bt_collision_objects 
 * returns a std::vector of btCollisionObject pointers
 */
vector< btCollisionObject* >
Collision_Object_URDF::
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
Collision_Object_URDF::
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
Collision_Object_URDF::
_load_collision_objects( void ){
  vector< shared_ptr< Link > > links;
  _kinematics_model.model().getLinks( links );
  string models_path = getModelsPath();
  for( unsigned int i = 0; i < links.size(); i++ ){
    if( links[ i ]->collision != NULL ){
      if( links[ i ]->collision->geometry->type == Geometry::SPHERE ){
        shared_ptr< Sphere > sphere = shared_dynamic_cast< Sphere >( links[ i ]->collision->geometry );
        _collision_objects.push_back( new Collision_Object_Sphere( links[ i ]->name, sphere->radius ) );
      } else if ( links[ i ]->collision->geometry->type == Geometry::BOX ){
        shared_ptr< Box > box = shared_dynamic_cast< Box >( links[ i ]->collision->geometry );
        _collision_objects.push_back( new Collision_Object_Box( links[ i ]->name, Vector3f( box->dim.x, box->dim.y, box->dim.z ) ) );
      } else if ( links[ i ]->collision->geometry->type == Geometry::CYLINDER ){
        shared_ptr< Cylinder > cylinder = shared_dynamic_cast< Cylinder >( links[ i ]->collision->geometry );
        _collision_objects.push_back( new Collision_Object_Cylinder( links[ i ]->name, cylinder->radius, cylinder->length ) );
      } else if ( links[ i ]->collision->geometry->type == Geometry::MESH ){
        shared_ptr< Mesh > mesh = shared_dynamic_cast< Mesh >( links[ i ]->collision->geometry );
        std::string model_filename = mesh->filename;
        model_filename.erase( model_filename.begin(), model_filename.begin() + 9 );
        model_filename.erase( model_filename.end() - 4, model_filename.end() );
        model_filename = models_path + string( "/mit_gazebo_models" ) + model_filename + string( "_chull.obj" );
        _collision_objects.push_back( new Collision_Object_Convex_Hull( links[ i ]->name, model_filename ) );       
      } 
    }
  }
  return;
}
