#include <GL/gl.h>

#include <opengl/opengl_object_box.h>

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace opengl;

/**
 * OpenGL_Object_Box
 * class constructor
 */
OpenGL_Object_Box::
OpenGL_Object_Box( string id,
                    const Frame& transform,
                    const Frame& offset,
                    Vector3f dimensions ) : OpenGL_Object( id, transform, offset ),
                                          _dimensions( dimensions ),
                                          _dl( 0 ){

}

/**
 * ~OpenGL_Object_Box
 * class destructor
 */
OpenGL_Object_Box::
~OpenGL_Object_Box() {
  if( _dl != 0 && glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
    _dl = 0;
  }
}

/**
 * OpenGL_Object_Box
 * copy constructor
 */
OpenGL_Object_Box::
OpenGL_Object_Box( const OpenGL_Object_Box& other ) : OpenGL_Object( other ),
                                                      _dimensions( other._dimensions ),
                                                      _dl( 0 ){

}

/**
 * operator=
 * assignment operator
 */
OpenGL_Object_Box&
OpenGL_Object_Box::
operator=( const OpenGL_Object_Box& other ) {
  _id = other._id;
  _visible = other._visible;
  _color = other._color;
  _transparency = other._transparency;
  _transform = other._transform;
  _offset = other._offset;
  _dimensions = other._dimensions;
  _dl = 0;
  return (*this);
}

void
OpenGL_Object_Box::
set( Vector3f dimensions ){
  _dimensions = dimensions;
  if( _dl != 0 && glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
    _dl = 0;
  }
  return;
}

/**
 * set
 * sets the transform and dimensions of the box
 */
void
OpenGL_Object_Box::
set( Frame transform,
      Vector3f dimensions ){
  _transform = transform;
  _dimensions = dimensions;
  if( _dl != 0 && glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
    _dl = 0;
  }
  return;
}

void
OpenGL_Object_Box::
set_color( Vector3f color ){
  _color = color;
  if( _dl != 0 && glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
    _dl = 0;
  }
  return;
}

void
OpenGL_Object_Box::
set_color( Vector4f color ){
  _color(0) = color(0);
  _color(1) = color(1);
  _color(2) = color(2);
  _transparency = color(3);
  if( _dl != 0 && glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
    _dl = 0;
  }
  return;
}

void
OpenGL_Object_Box::
set_transparency( double transparency ){
  _transparency = transparency;
  if( _dl != 0 && glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
    _dl = 0;
  }
  return;
}

/** 
 * draw
 * draws the box using opengl commands 
 */
void
OpenGL_Object_Box::
draw( void ){
  if( visible() ){
    if( glIsList( _dl ) == GL_TRUE ){
      glPushMatrix();
      apply_transform();
      glCallList( _dl );
      glPopMatrix();
    } else if( _generate_dl() ){ 
      draw();
    } 
  }
  return;
}

/**
 * draw
 * draws the box using opengl commands with a specific color
 */
void
OpenGL_Object_Box::
draw( Eigen::Vector3f color ){
  if (visible()){
    glPushMatrix();
    apply_transform();
    _draw_box( color );
    glPopMatrix();
  }
  return;
}

/**
 * dimensions
 * returns the dimensions of the box
 */
Vector3f
OpenGL_Object_Box::
dimensions( void )const{
  return _dimensions;
}

/**
 * _generate_dl
 * generates the display list
 */
bool 
OpenGL_Object_Box::
_generate_dl( void ){
  if( glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
    _dl = 0;
  }
  _dl = glGenLists( 1 );
  glNewList( _dl, GL_COMPILE );
  _draw_box( color() );
  glEndList();
  return true;
}

void
OpenGL_Object_Box::
_draw_box( Vector3f color ){
  glColor4f( color( 0 ), color( 1 ), color( 2 ), transparency() );
  glBegin( GL_QUADS );
  glNormal3f( 0.0, 0.0, 1.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );

  glNormal3f( 0.0, 0.0, -1.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );

  glNormal3f( 1.0, 0.0, 0.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );

  glNormal3f( -1.0, 0.0, 0.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );

  glNormal3f( 0.0, 1.0, 0.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, _dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );

  glNormal3f( 0.0, -1.0, 0.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, _dimensions( 2 ) / 2.0 );
  glVertex3f( -_dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glVertex3f( _dimensions( 0 ) / 2.0, -_dimensions( 1 ) / 2.0, -_dimensions( 2 ) / 2.0 );
  glEnd();
  return;
}
 
/**
 * operator<<
 * ostream operator
 */ 
namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Box& other ) {
    out << "color[3]:{" << other.color()(0) << "," << other.color()(1) << "," << other.color()(2) << "} ";
    out << "transparency[1]:{" << other.transparency() << "} ";
    out << "position[3]:{" << other.transform().p.x() << "," << other.transform().p.y() << "," << other.transform().p.z() << "} ";
    double qx, qy, qz, qw;
    other.transform().M.GetQuaternion( qx, qy, qz, qw );
    out << "rotation:{" << qw << ",{" << qx << "," << qy << "," << qz << "}} ";
    out << "length:{" << other.dimensions()(0) << "} ";
    out << "width:{" << other.dimensions()(1) << "} ";
    out << "height:{" << other.dimensions()(2) << "} ";
    return out;
  }
}
