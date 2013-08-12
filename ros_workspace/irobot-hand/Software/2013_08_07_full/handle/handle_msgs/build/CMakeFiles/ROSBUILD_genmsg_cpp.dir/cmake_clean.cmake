FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/handle_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/handle_msgs/HandleControl.h"
  "../msg_gen/cpp/include/handle_msgs/HandleSensorsCalibrated.h"
  "../msg_gen/cpp/include/handle_msgs/HandleCollisions.h"
  "../msg_gen/cpp/include/handle_msgs/CableTension.h"
  "../msg_gen/cpp/include/handle_msgs/Collision.h"
  "../msg_gen/cpp/include/handle_msgs/HandleSensors.h"
  "../msg_gen/cpp/include/handle_msgs/Finger.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
