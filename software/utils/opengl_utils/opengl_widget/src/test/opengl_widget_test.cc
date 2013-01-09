#include <iostream>

#include <QtGui/QApplication>

#include "opengl_object/opengl_object_box.h"
#include "opengl_object/opengl_object_cylinder.h"
#include "opengl_object/opengl_object_sphere.h"
#include "opengl_widget/opengl_widget.h"

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace opengl;

int
main( int argc,
      char* argv[] ) {
  QApplication app( argc, argv );

  OpenGL_Widget opengl_widget;
  OpenGL_Object_Box opengl_object_box;
  opengl_object_box.set( Frame( Vector( 1.0, 0.0, 0.0 ) ), Vector3f( 0.25, 0.25, 0.25 ) );
  opengl_object_box.set_color( Vector3f( 1.0, 1.0, 0.0 ) );
  opengl_widget.opengl_scene().add_object( opengl_object_box );

  OpenGL_Object_Cylinder opengl_object_cylinder;
  opengl_object_cylinder.set( Frame( Vector( 0.0, 1.0, 0.0 ) ), Vector2f( 0.25, 0.25 ) );
  opengl_object_cylinder.set_color( Vector3f( 0.0, 1.0, 1.0 ) );
  opengl_widget.opengl_scene().add_object( opengl_object_cylinder );

  OpenGL_Object_Sphere opengl_object_sphere;
  opengl_object_sphere.set( Frame( Vector( -0.5, -0.5, 0.0 ) ), 0.125 );
  opengl_object_sphere.set_color( Vector3f( 1.0, 0.0, 1.0 ) );
  opengl_widget.opengl_scene().add_object( opengl_object_sphere );

  opengl_widget.show();
  
  return app.exec();
}
