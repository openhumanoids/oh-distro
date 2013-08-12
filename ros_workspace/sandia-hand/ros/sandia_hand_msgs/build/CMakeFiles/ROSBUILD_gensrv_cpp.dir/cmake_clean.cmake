FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sandia_hand_msgs/msg"
  "../src/sandia_hand_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/sandia_hand_msgs/SimpleGraspSrv.h"
  "../srv_gen/cpp/include/sandia_hand_msgs/SetFingerHome.h"
  "../srv_gen/cpp/include/sandia_hand_msgs/GetParameters.h"
  "../srv_gen/cpp/include/sandia_hand_msgs/SetJointLimitPolicy.h"
  "../srv_gen/cpp/include/sandia_hand_msgs/SetParameters.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
