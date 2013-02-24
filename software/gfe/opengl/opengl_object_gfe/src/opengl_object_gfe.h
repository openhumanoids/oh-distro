#ifndef OPENGL_OPENGL_OBJECT_GFE_H
#define OPENGL_OPENGL_OBJECT_GFE_H

#include <iostream>

#include <state/state_gfe.h>
#include <kinematics_model/kinematics_model_gfe.h>
#include <opengl/opengl_object.h>

namespace opengl {
  class OpenGL_Object_GFE: public OpenGL_Object {
  public:
    OpenGL_Object_GFE();
    OpenGL_Object_GFE( std::string urdfFilename );
    ~OpenGL_Object_GFE();
    OpenGL_Object_GFE( const OpenGL_Object_GFE& other );
    OpenGL_Object_GFE& operator=( const OpenGL_Object_GFE& other );
    
    void set( const drc::robot_state_t& robotState );
    void set( state::State_GFE& stateGFE );
  
    virtual void draw( void );

  protected:
    void _load_opengl_objects(void );
    
    kinematics_model::Kinematics_Model_GFE _kinematics_model;
    std::vector< OpenGL_Object* > _opengl_objects;

  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_GFE& other );
}

#endif /* OPENGL_OPENGL_OBJECT_GFE_H */
