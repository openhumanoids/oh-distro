#include <iostream>
#include "opengl/opengl_object_cylinder.h"

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace opengl;

int
main( int argc,
      char* argv[] ) {
  OpenGL_Object_Cylinder opengl_object_cylinder;
  opengl_object_cylinder.set( Frame( Vector( 1.0, 0.0, 0.0 ) ), Vector2f( 0.25, 0.25 ) ); 
  cout << "opengl_object_cylinder: " << opengl_object_cylinder << endl;
  return 0;
}
