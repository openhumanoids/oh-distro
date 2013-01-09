#include <GL/gl.h>

#include <opengl_object/opengl_object_box.h>

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace opengl;

/**
 * OpenGL_Object_Box
 * class constructor
 */
OpenGL_Object_Box::
OpenGL_Object_Box() : OpenGL_Object(),
                      _dimensions( 1.0, 1.0, 1.0 ){

}

/**
 * ~OpenGL_Object_Box
 * class destructor
 */
OpenGL_Object_Box::
~OpenGL_Object_Box() {

}

/**
 * OpenGL_Object_Box
 * copy constructor
 */
OpenGL_Object_Box::
OpenGL_Object_Box( const OpenGL_Object_Box& other ) : OpenGL_Object( other ),
                                                      _dimensions( other._dimensions ){

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
  _dimensions = other._dimensions;
  return (*this);
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
  return;
}

/** 
 * draw
 * draws the box using opengl commands 
 */
void
OpenGL_Object_Box::
draw( void ){
  glPushMatrix();
  apply_transform();
  glColor4f( color()( 0 ), color()( 1 ), color()( 2 ), transparency() );
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
  glPopMatrix();
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
