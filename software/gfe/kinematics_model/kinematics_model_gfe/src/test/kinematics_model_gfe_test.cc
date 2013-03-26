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

  const SegmentMap& segment_map = kinematics_model_gfe.tree().getSegments();
  for ( map< string, TreeElement >::const_iterator it = segment_map.begin(); it != segment_map.end(); it++ ){
    cout << (*it).first << ":" << endl;
    const Segment& segment = (*it).second.segment;
    cout << "  Name: " << segment.getName() << endl;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    segment.getFrameToTip().M.GetRPY( roll, pitch, yaw );
    cout << "  FrameToTip: xyz={" << segment.getFrameToTip().p[0] << "," << segment.getFrameToTip().p[1] << "," << segment.getFrameToTip().p[2] << "} rpy:{" << roll << "," << pitch << "," << yaw << "}" << endl;
    cout << "  Joint Name: " << segment.getJoint().getName() << endl;
    cout << "  Joint Type: " << segment.getJoint().getTypeName() << endl;
    cout << "  Joint Origin: " << segment.getJoint().JointOrigin().x() << "," << segment.getJoint().JointOrigin().y() << "," << segment.getJoint().JointOrigin().z() << endl;
    cout << "  Joint Axis: " << segment.getJoint().JointAxis().x() << "," << segment.getJoint().JointAxis().y() << "," << segment.getJoint().JointAxis().z() << endl;
  }

  cout << endl << "end of kinematics-model-gfe-test" << endl << endl;

  return 0;
}
