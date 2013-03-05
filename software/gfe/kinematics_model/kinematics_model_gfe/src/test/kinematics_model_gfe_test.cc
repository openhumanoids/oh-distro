#include <iostream>
#include <map>
#include <path_util/path_util.h>
#include <state/state_gfe.h>
#include <kinematics/kinematics_model_gfe.h>

using namespace std;
using namespace urdf;
using namespace KDL;
using namespace state;
using namespace kinematics;

int 
main( int argc, 
      char* argv[])
{
  cout << endl << "start of kinematics-model-gfe-test" << endl << endl;
  
  Kinematics_Model_GFE kinematics_model_gfe;
  cout << "kinematics_model_gfe: " << kinematics_model_gfe << endl;

  State_GFE robot_state;
  Frame utorso_to_hand_pose = Frame::Identity();
  utorso_to_hand_pose.p[2] = 0.3;
  State_GFE_Arm left_arm_state;
  
  if( kinematics_model_gfe.inverse_kinematics_left_arm( robot_state, utorso_to_hand_pose, left_arm_state ) ){
    cout << "found inverse kinematics: " << left_arm_state << endl;
  } else {
    cout << "could not solve inverse kinematics" << endl;
  }

  cout << endl << "end of kinematics-model-gfe-test" << endl << endl;

  return 0;
}
