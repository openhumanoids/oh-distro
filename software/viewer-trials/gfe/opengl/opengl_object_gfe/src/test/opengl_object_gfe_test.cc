#include <iostream>
#include <state/state_gfe.h>
#include "opengl/opengl_object_gfe.h"

using namespace std;
using namespace Eigen;
using namespace opengl;
using namespace state;

int
main( int argc,
      char* argv[] ) {
  OpenGL_Object_GFE opengl_object_gfe;
  State_GFE state_gfe;
  opengl_object_gfe.set( state_gfe );
  cout << "opengl_object_gfe: " << opengl_object_gfe << endl;
  return 0;
}
