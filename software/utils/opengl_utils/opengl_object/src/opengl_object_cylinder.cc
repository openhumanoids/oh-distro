#include "opengl_object/opengl_object_cylinder.h"

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace opengl;

/**
 * OpenGL_Object_Cylinder
 * class constructor
 */
OpenGL_Object_Cylinder::
OpenGL_Object_Cylinder() : OpenGL_Object(),
                            _dimensions( 1.0, 1.0 ),
                            _quadric( NULL ){

}

/**
 * ~OpenGL_Object_Cylinder
 * class destructor
 */
OpenGL_Object_Cylinder::
~OpenGL_Object_Cylinder() {

}

/** 
 * OpenGL_Object_Cylinder
 * copy constructor
 */
OpenGL_Object_Cylinder::
OpenGL_Object_Cylinder( const OpenGL_Object_Cylinder& other ) : OpenGL_Object( other ),
                                                                _dimensions( other._dimensions ),
                                                                _quadric( NULL ){

}

/**
 * operator=
 * assignment operator
 */
OpenGL_Object_Cylinder&
OpenGL_Object_Cylinder::
operator=( const OpenGL_Object_Cylinder& other ) {
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
 * sets the position and dimensions of the cylinder
 */
void
OpenGL_Object_Cylinder::
set( Frame transform,
      Vector2f dimensions ){
  _transform = transform;
  _dimensions = dimensions;
  return;
}

/**
 * draw
 * draws the cylinder using opengl commands
 */
void
OpenGL_Object_Cylinder::
draw( void ){
  glPushMatrix();
  apply_transform();
  glTranslatef( 0.0, 0.0, -_dimensions( 1 ) / 2.0 );
  glColor4f( color()( 0 ), color()( 1 ), color()( 2 ), transparency() );
  if( _quadric == NULL ){
    _quadric = gluNewQuadric();
  }
  gluCylinder( _quadric, _dimensions( 0 ), _dimensions( 0 ), _dimensions( 1 ), 16, 16 );
  gluDisk( _quadric, 0.0, _dimensions( 0 ), 16, 16 );
  glTranslatef( 0.0, 0.0, _dimensions( 1 ) );
  gluDisk( _quadric, 0.0, _dimensions( 0 ), 16, 16 );
  glPopMatrix();
}

/**
 * dimensions
 * returns the dimensions of the cylinder
 */
Vector2f
OpenGL_Object_Cylinder::
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
              const OpenGL_Object_Cylinder& other ) {
    out << "color[3]:{" << other.color()(0) << "," << other.color()(1) << "," << other.color()(2) << "} ";
    out << "transparency[1]:{" << other.transparency() << "} ";
    out << "position[3]:{" << other.transform().p.x() << "," << other.transform().p.y() << "," << other.transform().p.z() << "} ";
    double qx, qy, qz, qw;
    other.transform().M.GetQuaternion( qx, qy, qz, qw );
    out << "rotation:{" << qw << ",{" << qx << "," << qy << "," << qz << "}} ";
    out << "radius:{" << other.dimensions()(0) << "} ";
    out << "height:{" << other.dimensions()(1) << "} ";
    return out;
  }
}
