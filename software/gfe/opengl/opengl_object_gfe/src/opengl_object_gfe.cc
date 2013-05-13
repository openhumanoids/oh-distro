#include <path_util/path_util.h>

#include <opengl/opengl_object_sphere.h>
#include <opengl/opengl_object_box.h>
#include <opengl/opengl_object_cylinder.h>
#include <opengl/opengl_object_dae.h>
#include <opengl/opengl_object_gfe.h>

using namespace std;
using namespace boost;
using namespace urdf;
using namespace KDL;
using namespace Eigen;
using namespace drc;
using namespace state;
using namespace kinematics;
using namespace opengl;

/**
 * OpenGL_Object_GFE
 * class constructor
 */
OpenGL_Object_GFE::
OpenGL_Object_GFE() : OpenGL_Object(),
                      _kinematics_model(),
                      _opengl_objects(),
                      _opengl_object_coordinate_axis( true, false ) {
  _load_opengl_objects();
  State_GFE state_gfe;
  set( state_gfe );
}

/**
 * OpenGL_Object_GFE
 * class constructor with urdf filename argument
 */
OpenGL_Object_GFE::
OpenGL_Object_GFE( std::string urdfFilename ) : OpenGL_Object(),
                                                _kinematics_model( urdfFilename ),
                                                _opengl_objects(),
                                                _opengl_object_coordinate_axis() {
  _load_opengl_objects();
  State_GFE state_gfe;
  set( state_gfe );
}

/**
 * ~OpenGL_Object_GFE
 * class destructor 
 */
OpenGL_Object_GFE::
~OpenGL_Object_GFE() {
  for( unsigned int i = 0; i < _opengl_objects.size(); i++ ){
    if( _opengl_objects[ i ] != NULL ){
      delete _opengl_objects[ i ];
      _opengl_objects[ i ] = NULL;
    } 
  }
  _opengl_objects.clear();
}

/**
 * OpenGL_Object_GFE
 * copy constructor
 */
OpenGL_Object_GFE::
OpenGL_Object_GFE( const OpenGL_Object_GFE& other ) : OpenGL_Object( other ),
                                                      _kinematics_model( other._kinematics_model ),
                                                      _opengl_objects( other._opengl_objects ),
                                                      _opengl_object_coordinate_axis( other._opengl_object_coordinate_axis ) {

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
  _opengl_object_coordinate_axis = other._opengl_object_coordinate_axis;
  return (*this);
}

/** 
 * set
 * sets the state of the opengl object from the drc::robot_state_t message
 */
void
OpenGL_Object_GFE::
set( const robot_state_t& robotState ){
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

void
OpenGL_Object_GFE::
set_transparency( double transparency ){
  for( unsigned int i = 0; i < _opengl_objects.size(); i++ ){
    _opengl_objects[ i ]->set_transparency( transparency );
  }
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
//        _opengl_object_coordinate_axis.set_transform( _kinematics_model.link( _opengl_objects[ i ]->id() ) );
//        _opengl_object_coordinate_axis.draw();
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
    for( std::map< std::string, boost::shared_ptr<std::vector<boost::shared_ptr<Visual> > > >::iterator it = links[i]->visual_groups.begin(); it != links[i]->visual_groups.end(); it++ ){
      for( unsigned int j = 0; j < it->second->size(); j++ ){
        if( (*it->second)[ j ] != NULL ){
          if( (*it->second)[ j ]->geometry->type == Geometry::SPHERE ){
            shared_ptr< Sphere > sphere = shared_dynamic_cast< Sphere >( (*it->second)[j]->geometry );
            _opengl_objects.push_back( new OpenGL_Object_Sphere( links[i]->name, KDL::Frame::Identity(), KDL::Frame::Identity(), sphere->radius ) );
            KDL::Frame offset;
            offset.p[0] = (*it->second)[j]->origin.position.x;
            offset.p[1] = (*it->second)[j]->origin.position.y;
            offset.p[2] = (*it->second)[j]->origin.position.z;
            offset.M = KDL::Rotation::Quaternion( (*it->second)[j]->origin.rotation.x,
                                          (*it->second)[j]->origin.rotation.y,
                                          (*it->second)[j]->origin.rotation.z,
                                          (*it->second)[j]->origin.rotation.w );
            _opengl_objects.back()->set_offset( offset );
          } else if ( (*it->second)[ j ]->geometry->type == Geometry::BOX ){
            shared_ptr< Box > box = shared_dynamic_cast< Box >( (*it->second)[j]->geometry );
            _opengl_objects.push_back( new OpenGL_Object_Box( links[i]->name, KDL::Frame::Identity(), KDL::Frame::Identity(), Vector3f( box->dim.x, box->dim.y, box->dim.z ) ) );
            KDL::Frame offset;
            offset.p[0] = (*it->second)[j]->origin.position.x;
            offset.p[1] = (*it->second)[j]->origin.position.y;
            offset.p[2] = (*it->second)[j]->origin.position.z;
            offset.M = KDL::Rotation::Quaternion( (*it->second)[j]->origin.rotation.x,
                                          (*it->second)[j]->origin.rotation.y,
                                          (*it->second)[j]->origin.rotation.z,
                                          (*it->second)[j]->origin.rotation.w );
            _opengl_objects.back()->set_offset( offset );

          } else if ( (*it->second)[ j ]->geometry->type == Geometry::CYLINDER ){
            shared_ptr< Cylinder > cylinder = shared_dynamic_cast< Cylinder >( (*it->second)[j]->geometry );
            _opengl_objects.push_back( new OpenGL_Object_Cylinder( links[ i ]->name, KDL::Frame::Identity(), KDL::Frame::Identity(), Vector2f( cylinder->radius, cylinder->length ) ) );
            KDL::Frame offset;
            offset.p[0] = (*it->second)[j]->origin.position.x;
            offset.p[1] = (*it->second)[j]->origin.position.y;
            offset.p[2] = (*it->second)[j]->origin.position.z;
            offset.M = KDL::Rotation::Quaternion( (*it->second)[j]->origin.rotation.x,
                                          (*it->second)[j]->origin.rotation.y,
                                          (*it->second)[j]->origin.rotation.z,
                                          (*it->second)[j]->origin.rotation.w );
            _opengl_objects.back()->set_offset( offset );
          }
        }
      }
    }
    if( links[ i ]->visual != NULL ){
      if( links[ i ]->visual->geometry->type == Geometry::SPHERE ){
        shared_ptr< Sphere > sphere = shared_dynamic_cast< Sphere >( links[ i ]->visual->geometry );
        _opengl_objects.push_back( new OpenGL_Object_Sphere( links[ i ]->name, KDL::Frame::Identity(), KDL::Frame::Identity(), sphere->radius ) );
        KDL::Frame offset;
        offset.p[0] = links[ i ]->visual->origin.position.x;
        offset.p[1] = links[ i ]->visual->origin.position.y;
        offset.p[2] = links[ i ]->visual->origin.position.z;
        offset.M = KDL::Rotation::Quaternion( links[ i ]->visual->origin.rotation.x,
                                          links[ i ]->visual->origin.rotation.y,
                                          links[ i ]->visual->origin.rotation.z,
                                          links[ i ]->visual->origin.rotation.w );
        _opengl_objects.back()->set_offset( offset );
      } else if ( links[ i ]->visual->geometry->type == Geometry::BOX ){
        shared_ptr< Box > box = shared_dynamic_cast< Box >( links[ i ]->visual->geometry );
        _opengl_objects.push_back( new OpenGL_Object_Box( links[ i ]->name, KDL::Frame::Identity(), KDL::Frame::Identity(), Vector3f( box->dim.x, box->dim.y, box->dim.z ) ) );
        KDL::Frame offset;
        offset.p[0] = links[ i ]->visual->origin.position.x;
        offset.p[1] = links[ i ]->visual->origin.position.y;
        offset.p[2] = links[ i ]->visual->origin.position.z;
        offset.M = KDL::Rotation::Quaternion( links[ i ]->visual->origin.rotation.x,
                                          links[ i ]->visual->origin.rotation.y,
                                          links[ i ]->visual->origin.rotation.z,
                                          links[ i ]->visual->origin.rotation.w );
        _opengl_objects.back()->set_offset( offset );
      } else if ( links[ i ]->visual->geometry->type == Geometry::CYLINDER ){
        shared_ptr< Cylinder > cylinder = shared_dynamic_cast< Cylinder >( links[ i ]->visual->geometry );
        _opengl_objects.push_back( new OpenGL_Object_Cylinder( links[ i ]->name, KDL::Frame::Identity(), KDL::Frame::Identity(), Vector2f( cylinder->radius, cylinder->length ) ) );
        KDL::Frame offset;
        offset.p[0] = links[ i ]->visual->origin.position.x;
        offset.p[1] = links[ i ]->visual->origin.position.y;
        offset.p[2] = links[ i ]->visual->origin.position.z;
        offset.M = KDL::Rotation::Quaternion( links[ i ]->visual->origin.rotation.x,
                                          links[ i ]->visual->origin.rotation.y,
                                          links[ i ]->visual->origin.rotation.z,
                                          links[ i ]->visual->origin.rotation.w );
        _opengl_objects.back()->set_offset( offset );
      } else if ( links[ i ]->visual->geometry->type == Geometry::MESH ){
        shared_ptr< Mesh > mesh = shared_dynamic_cast< Mesh >( links[ i ]->visual->geometry );
        std::string model_filename = mesh->filename;
        model_filename.erase( model_filename.begin(), model_filename.begin() + 9 );
        model_filename.erase( model_filename.end()-3, model_filename.end() );
        // chomp model filename up to models/ *TODO UGLY HACK*
        boost::replace_all(model_filename, "mit_drcsim_scripts/models/", "");
        model_filename = models_path + string( "/mit_gazebo_models" ) + model_filename + string( "dae" );
        _opengl_objects.push_back( new OpenGL_Object_DAE( links[ i ]->name, KDL::Frame::Identity(), KDL::Frame::Identity(), model_filename ) );
        KDL::Frame offset;
        offset.p[0] = links[ i ]->visual->origin.position.x;
        offset.p[1] = links[ i ]->visual->origin.position.y;
        offset.p[2] = links[ i ]->visual->origin.position.z;
        offset.M = KDL::Rotation::Quaternion( links[ i ]->visual->origin.rotation.x,
                                          links[ i ]->visual->origin.rotation.y,
                                          links[ i ]->visual->origin.rotation.z,
                                          links[ i ]->visual->origin.rotation.w );
        _opengl_objects.back()->set_offset( offset );
      }
    }
  }
  return;
}

Kinematics_Model_GFE&
OpenGL_Object_GFE::
kinematics_model( void ){
  return _kinematics_model;
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
