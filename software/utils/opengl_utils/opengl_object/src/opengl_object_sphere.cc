#include "opengl_object/opengl_object_sphere.h"

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
                            _quadric( NULL ){

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
                                                                _quadric( NULL ){

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
  glPushMatrix();
  apply_transform();
  glColor4f( color()( 0 ), color()( 1 ), color()( 2 ), transparency() );
  if( _quadric == NULL ){
    _quadric = gluNewQuadric();
  }
  gluSphere( _quadric, _dimensions, 16, 16 );
  glPopMatrix();
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
