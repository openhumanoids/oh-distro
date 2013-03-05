#include <iostream>
#include <map>
#include <path_util/path_util.h>
#include <kinematics/kinematics_model_urdf.h>

using namespace std;
using namespace urdf;
using namespace KDL;
using namespace drc;
using namespace kinematics;

int 
main( int argc, 
      char* argv[])
{
  cout << endl << "start of kinematics-model-urdf-test" << endl << endl;
  
  Kinematics_Model_URDF kinematics_model_urdf(getModelsPath() + string( "/mit_gazebo_models/mit_robot_PnC/model.urdf" ));
  cout << kinematics_model_urdf << endl;
  cout << endl << "end of kinematics-model-urdf-test" << endl << endl;

  return 0;
}
