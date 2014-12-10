#ifndef AUTHORING_OPENGL_OBJECT_AFFORDANCE_COLLECTION_H
#define AUTHORING_OPENGL_OBJECT_AFFORDANCE_COLLECTION_H

#include <iostream>
#include <vector>

#include <affordance/AffordanceState.h>

#include "opengl/opengl_object.h"
#include "authoring/opengl_object_affordance.h"

namespace authoring {
  class OpenGL_Object_Affordance_Collection: public opengl::OpenGL_Object {
  public:
    OpenGL_Object_Affordance_Collection();
    ~OpenGL_Object_Affordance_Collection();
    OpenGL_Object_Affordance_Collection( const OpenGL_Object_Affordance_Collection& other );
    OpenGL_Object_Affordance_Collection& operator=( const OpenGL_Object_Affordance_Collection& other );

    virtual void set( const std::vector< affordance::AffordanceState >& affordanceCollection );
    virtual void draw( void );
    virtual void set_color( Eigen::Vector3f color );
    virtual void set_transparency( double transparency );
    virtual void set_highlight( const std::vector< std::string >& highlightIds );

  protected:
    OpenGL_Object_Affordance _opengl_object_affordance;
    std::vector< affordance::AffordanceState > _affordance_collection;
    std::vector< std::string > _highlight_ids;
  
  private:

  };
  std::ostream& operator<<( std::ostream& out, const OpenGL_Object_Affordance_Collection& other );
}

#endif /* AUTHORING_OPENGL_OBJECT_AFFORDANCE_COLLECTION_H */
