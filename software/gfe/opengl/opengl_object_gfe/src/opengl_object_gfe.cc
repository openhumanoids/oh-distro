#include <path_util/path_util.h>

#include <opengl/opengl_object_dae.h>
#include <opengl/opengl_object_gfe.h>

using namespace std;
using namespace boost;
using namespace urdf;
using namespace KDL;
using namespace Eigen;
using namespace drc;
using namespace state;
using namespace kinematics_model;
using namespace opengl;

/**
 * OpenGL_Object_GFE
 * class constructor
 */
OpenGL_Object_GFE::
OpenGL_Object_GFE() : OpenGL_Object(),
                      _kinematics_model(),
                      _opengl_objects() {
  _load_opengl_objects();
}

/**
 * OpenGL_Object_GFE
 * class constructor with urdf filename argument
 */
OpenGL_Object_GFE::
OpenGL_Object_GFE( std::string urdfFilename ) : OpenGL_Object(),
                                                _kinematics_model( urdfFilename ),
                                                _opengl_objects() {
  _load_opengl_objects();
}

/**
 * ~OpenGL_Object_GFE
 * class destructor 
 */
OpenGL_Object_GFE::
~OpenGL_Object_GFE() {
  _opengl_objects.clear();
}

/**
 * OpenGL_Object_GFE
 * copy constructor
 */
OpenGL_Object_GFE::
OpenGL_Object_GFE( const OpenGL_Object_GFE& other ) : OpenGL_Object( other ),
                                                      _kinematics_model( other._kinematics_model ),
                                                      _opengl_objects( other._opengl_objects ) {

}

/**
 * operator=
 * assignment operator
 */
OpenGL_Object_GFE&
OpenGL_Object_GFE::
operator=( const OpenGL_Object_GFE& other ) {
  _id = other._id;
  _visible = other._visible;
  _color = other._color;
  _transparency = other._transparency;
  _transform = other._transform;
  _kinematics_model = other._kinematics_model;
  _opengl_objects = other._opengl_objects;
  return (*this);
}

/** 
 * set
 * sets the state of the opengl object from the drc::robot_state_t message
 */
void
OpenGL_Object_GFE::
set( robot_state_t& robotState ){
  _kinematics_model.set( robotState );
  return;
}

/**
 * set 
 * sets the state of the opengl object from the state::State_GFE class
 */
void
OpenGL_Object_GFE::
set( State_GFE& stateGFE ){
  _kinematics_model.set( stateGFE );
  return;
}

/**
 * draw
 * draws the opengl object representing the GFE
 */
void
OpenGL_Object_GFE::
draw( void ){
  if( visible() ){
    for( unsigned int i = 0; i < _opengl_objects.size(); i++ ){
      if( _opengl_objects[ i ] != NULL ){
        _opengl_objects[ i ]->set_transform( _kinematics_model.link( _opengl_objects[ i ]->id() ) );
        _opengl_objects[ i ]->draw();
      }
    }
  }
  return;
}

/**
 * _load_opengl_objects
 * traverse the kinematics model and load opengl objects based on the link type
 */
void
OpenGL_Object_GFE::
_load_opengl_objects( void ){
  vector< shared_ptr< Link > > links;
  _kinematics_model.model().getLinks( links );
  string models_path = getModelsPath();
  for( unsigned int i = 0; i < links.size(); i++ ){
    if( links[ i ]->visual != NULL ){
      if( links[ i ]->visual->geometry->type == Geometry::SPHERE ){
        shared_ptr< Sphere > sphere = shared_dynamic_cast< Sphere >( links[ i ]->visual->geometry );
      } else if ( links[ i ]->visual->geometry->type == Geometry::BOX ){
        shared_ptr< Box > box = shared_dynamic_cast< Box >( links[ i ]->visual->geometry );
      } else if ( links[ i ]->visual->geometry->type == Geometry::CYLINDER ){
        shared_ptr< Cylinder > cylinder = shared_dynamic_cast< Cylinder >( links[ i ]->visual->geometry );
      } else if ( links[ i ]->visual->geometry->type == Geometry::MESH ){
        shared_ptr< Mesh > mesh = shared_dynamic_cast< Mesh >( links[ i ]->visual->geometry );
        std::string model_filename = mesh->filename;
        model_filename.erase( model_filename.begin(), model_filename.begin() + 9 );
        model_filename.erase( model_filename.end()-3, model_filename.end() );
        model_filename = models_path + string( "/mit_gazebo_models" ) + model_filename + string( "dae" );
        _opengl_objects.push_back( new OpenGL_Object_DAE( links[ i ]->name, model_filename ) );
      }
    }
  }
  return;
}

/**
 * operator<<
 * ostream operator
 */
namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_GFE& other ) {
    return out;
  }
}
