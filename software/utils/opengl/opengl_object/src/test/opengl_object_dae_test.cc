#include <iostream>
#include <path_util/path_util.h>
#include "opengl/opengl_object_dae.h"

using namespace std;
using namespace KDL;
using namespace opengl;

int
main( int argc,
      char* argv[] ) {

  OpenGL_Object_DAE opengl_object_dae( "object-object-dae", Frame::Identity(), Frame::Identity(), getModelsPath() + string( "/mit_gazebo_models/mit_robot/meshes/utorso.dae" ) );
  cout << "opengl_object_dae: " << opengl_object_dae << endl;
  return 0;
}
