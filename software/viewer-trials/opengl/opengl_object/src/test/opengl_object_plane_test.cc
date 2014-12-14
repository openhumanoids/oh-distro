#include <iostream>
#include "opengl/opengl_object_plane.h"

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace opengl;

int
main( int argc,
      char* argv[] ) {
  OpenGL_Object_Plane opengl_object_plane;
  opengl_object_plane.set( Frame( Vector( 1.0, 1.0, 1.0 ) ), Vector2f( 0.25, 0.5 ) );
  cout << "opengl_object_plane: " << opengl_object_plane << endl;
  return 0;
}
