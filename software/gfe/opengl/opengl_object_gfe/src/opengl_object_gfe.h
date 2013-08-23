#ifndef OPENGL_OPENGL_OBJECT_GFE_H
#define OPENGL_OPENGL_OBJECT_GFE_H

#include <iostream>

// sorry, boost
#include <boost/algorithm/string.hpp>

#include <state/state_gfe.h>
#include <kinematics/kinematics_model_gfe.h>
#include <opengl/opengl_object.h>
#include <opengl/opengl_object_coordinate_axis.h>

namespace opengl {
  class OpenGL_Object_GFE: public OpenGL_Object {
  public:
    OpenGL_Object_GFE();
    OpenGL_Object_GFE( const std::string& xmlString );
    ~OpenGL_Object_GFE();
    OpenGL_Object_GFE( const OpenGL_Object_GFE& other );
    OpenGL_Object_GFE& operator=( const OpenGL_Object_GFE& other );
    
    void set( const drc::robot_state_t& robotState );
    void set( state::State_GFE& stateGFE );
    virtual void set_transparency( double transparency ); 
    void set_selected_links(const std::vector<std::string>& link_names, const Eigen::Vector3f& select_color, const Eigen::Vector3f& not_select_color);
 
    virtual void draw( void );

    kinematics::Kinematics_Model_GFE& kinematics_model( void );

  protected:
    void _load_opengl_objects(void );
    
    kinematics::Kinematics_Model_GFE _kinematics_model;
    std::vector< OpenGL_Object* > _opengl_objects;
    OpenGL_Object_Coordinate_Axis _opengl_object_coordinate_axis;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_GFE& other );
}

#endif /* OPENGL_OPENGL_OBJECT_GFE_H */
