#include <iostream>
#include "opengl_object/opengl_object_box.h"

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace opengl;

int
main( int argc,
      char* argv[] ) {
  OpenGL_Object_Box opengl_object_box;
  opengl_object_box.set( Frame( Vector( 1.0, 1.0, 1.0 ) ), Vector3f( 0.25, 0.5, 0.75 ) );
  cout << "opengl_object_box: " << opengl_object_box << endl;
  return 0;
}
