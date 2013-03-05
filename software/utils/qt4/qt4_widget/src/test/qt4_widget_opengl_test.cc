#include <iostream>

#include <QtGui/QApplication>

#include <path_util/path_util.h>

#include "collision/collision_object_box.h"
#include "collision/collision_object_sphere.h"
#include "collision/collision_object_cylinder.h"
#include "collision/collision_object_torus.h"

#include "opengl/opengl_object_box.h"
#include "opengl/opengl_object_cylinder.h"
#include "opengl/opengl_object_sphere.h"
#include "opengl/opengl_object_torus.h"
#include "opengl/opengl_object_dae.h"
#include "opengl/opengl_object_coordinate_axis.h"
#include "opengl/opengl_object_collision_detector.h"
#include "qt4/qt4_widget_opengl.h"

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace collision;
using namespace opengl;
using namespace qt4;

int
main( int argc,
      char* argv[] ) {
  QApplication app( argc, argv );

  Qt4_Widget_OpenGL qt4_widget_opengl;

  OpenGL_Object_Coordinate_Axis opengl_object_coordinate_axis;
  opengl_object_coordinate_axis.set_scale( 0.2 );
  opengl_object_coordinate_axis.set_transform( Frame( Vector( 0.0, 1.0, 0.0 ) ) );
  qt4_widget_opengl.opengl_scene().add_object( opengl_object_coordinate_axis );

  OpenGL_Object_Box opengl_object_box;
  opengl_object_box.set( Frame( Vector( 1.0, 0.0, 0.0 ) ), Vector3f( 0.25, 0.25, 0.25 ) );
  opengl_object_box.set_color( Vector3f( 1.0, 1.0, 0.0 ) );
  qt4_widget_opengl.opengl_scene().add_object( opengl_object_box );

  OpenGL_Object_Torus opengl_object_torus( 0.1, 0.025 );
  qt4_widget_opengl.opengl_scene().add_object( opengl_object_torus );

  OpenGL_Object_Cylinder opengl_object_cylinder;
  opengl_object_cylinder.set( Frame( Vector( 0.0, 1.0, 0.0 ) ), Vector2f( 0.125, 0.25 ) );
  opengl_object_cylinder.set_transparency( 0.5 );
  opengl_object_cylinder.set_color( Vector3f( 0.0, 1.0, 1.0 ) );
  qt4_widget_opengl.opengl_scene().add_object( opengl_object_cylinder );

  OpenGL_Object_Sphere opengl_object_sphere;
  opengl_object_sphere.set( Frame( Vector( -0.5, -0.5, 0.0 ) ), 0.125 );
  opengl_object_sphere.set_transparency( 0.5 );
  opengl_object_sphere.set_color( Vector3f( 1.0, 0.0, 1.0 ) );
  qt4_widget_opengl.opengl_scene().add_object( opengl_object_sphere );

  OpenGL_Object_DAE opengl_object_dae( "object-object-dae", getModelsPath() + string( "/mit_gazebo_models/mit_golf_cart/meshes/new_golf_cart.dae" ) );
  opengl_object_dae.set( Frame( Vector( -1.0, 0.0, 0.0 ) ) );
  qt4_widget_opengl.opengl_scene().add_object( opengl_object_dae );

  Collision_Object_Box collision_object_box( "box", Vector3f( 0.25, 0.25, 0.25 ), Vector3f( 1.0, 0.0, 0.0 ), Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
  Collision_Object_Sphere collision_object_sphere( "sphere", 0.125, Vector3f( -0.5, -0.5, 0.0 ), Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
  Collision_Object_Cylinder collision_object_cylinder( "cylinder", 0.125, 0.25, Vector3f( 0.0, 1.0, 0.0 ), Vector4f( 0.0, 0.0, 0.0, 1.0 ) );
  Collision_Object_Torus collision_object_torus( "torus", 0.1, 0.025, Vector3f( 0.0, 0.0, 0.0 ), Vector4f( 0.0, 0.0, 0.0, 1.0 ) );

  qt4_widget_opengl.add_collision_object( collision_object_box );
  qt4_widget_opengl.add_collision_object( collision_object_sphere );
  qt4_widget_opengl.add_collision_object( collision_object_cylinder );
  qt4_widget_opengl.add_collision_object( collision_object_torus );

  qt4_widget_opengl.show();
  
  return app.exec();
}
