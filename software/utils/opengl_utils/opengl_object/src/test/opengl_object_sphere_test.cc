#include <iostream>
#include "opengl_object/opengl_object_sphere.h"

using namespace std;
using namespace KDL;
using namespace Eigen;
using namespace opengl;

int
main( int argc,
      char* argv[] ) {
  OpenGL_Object_Sphere opengl_object_sphere;
  opengl_object_sphere.set( Frame( Vector( 1.0, 0.0, 0.0 ) ), 0.5 ); 
  cout << "opengl_object_sphere: " << opengl_object_sphere << endl;
  return 0;
}
