FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sandia_hand_msgs/msg"
  "../src/sandia_hand_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/sandia_hand_msgs/RawTactile.h"
  "../msg_gen/cpp/include/sandia_hand_msgs/RawFingerInertial.h"
  "../msg_gen/cpp/include/sandia_hand_msgs/RawFingerState.h"
  "../msg_gen/cpp/include/sandia_hand_msgs/RawMoboState.h"
  "../msg_gen/cpp/include/sandia_hand_msgs/CalFingerState.h"
  "../msg_gen/cpp/include/sandia_hand_msgs/RawPalmState.h"
  "../msg_gen/cpp/include/sandia_hand_msgs/RelativeJointCommands.h"
  "../msg_gen/cpp/include/sandia_hand_msgs/SimpleGrasp.h"
  "../msg_gen/cpp/include/sandia_hand_msgs/RawFingerCommands.h"
  "../msg_gen/cpp/include/sandia_hand_msgs/Parameter.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
