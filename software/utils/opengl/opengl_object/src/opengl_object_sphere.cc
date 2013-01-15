#include "opengl/opengl_object_sphere.h"

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace opengl;

/**
 * OpenGL_Object_Sphere
 * class constructor
 */
OpenGL_Object_Sphere::
OpenGL_Object_Sphere() : OpenGL_Object(),
                            _dimensions( 1.0 ),
                            _quadric( NULL ),
                            _dl( 0 ){

}

/**
 * ~OpenGL_Object_Sphere
 * class destructor
 */
OpenGL_Object_Sphere::
~OpenGL_Object_Sphere() {

}

/** 
 * OpenGL_Object_Sphere
 * copy constructor
 */
OpenGL_Object_Sphere::
OpenGL_Object_Sphere( const OpenGL_Object_Sphere& other ) : OpenGL_Object( other ),
                                                                _dimensions( other._dimensions ),
                                                                _quadric( NULL ),
                                                                _dl( 0 ){

}

/**
 * operator=
 * assignment operator
 */
OpenGL_Object_Sphere&
OpenGL_Object_Sphere::
operator=( const OpenGL_Object_Sphere& other ) {
  _id = other._id;
  _visible = other._visible;
  _color = other._color;
  _transparency = other._transparency;
  _transform = other._transform;
  _dimensions = other._dimensions;
  _quadric = NULL;
  _dl = 0;
  return (*this);
}

/**
 * set
 * sets the transform and dimensions of the sphere
 */
void
OpenGL_Object_Sphere::
set( Frame transform,
      double dimensions ){
  _transform = transform;
  _dimensions = dimensions;
  return;
}

/**
 * draw
 * draws the sphere using opengl commands
 */
void
OpenGL_Object_Sphere::
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
 * draws the sphere using openg commands and a specific color
 */
void
OpenGL_Object_Sphere::
draw( Vector3f color ){
  if( visible() ){
    glPushMatrix();
    apply_transform();
    glColor4f( color( 0 ), color( 1 ), color( 2 ), transparency() );
    if( _quadric == NULL ){
      _quadric = gluNewQuadric();
    }
    gluSphere( _quadric, _dimensions, 16, 16 );
    glPopMatrix();
  }
  return;
}

/**
 * dimensions
 * returns the dimensions of the sphere 
 */
double
OpenGL_Object_Sphere::
dimensions( void )const{
  return _dimensions;
}

/**
 * _generate_dl
 * generates the display lists
 */
bool
OpenGL_Object_Sphere::
_generate_dl( void ){
  if( glIsList( _dl ) == GL_TRUE ){
    glDeleteLists( _dl, 0 );
  }
  _dl = glGenLists( 1 );
  glNewList( _dl, GL_COMPILE );
  glColor4f( color()( 0 ), color()( 1 ), color()( 2 ), transparency() );
  if( _quadric == NULL ){
    _quadric = gluNewQuadric();
  }
  gluSphere( _quadric, _dimensions, 16, 16 );
  glEndList();
  return true;
}

/**
 * operator<<
 * ostream operator
 */
namespace opengl {
  ostream&
  operator<<( ostream& out,
              const OpenGL_Object_Sphere& other ) {
    out << "color[3]:{" << other.color()(0) << "," << other.color()(1) << "," << other.color()(2) << "} ";
    out << "transparency[1]:{" << other.transparency() << "} ";
    out << "position[3]:{" << other.transform().p.x() << "," << other.transform().p.y() << "," << other.transform().p.z() << "} ";
    double qx, qy, qz, qw;
    other.transform().M.GetQuaternion( qx, qy, qz, qw );
    out << "rotation:{" << qw << ",{" << qx << "," << qy << "," << qz << "}} ";
    out << "radius:{" << other.dimensions() << "} ";
    return out;
  }
}
