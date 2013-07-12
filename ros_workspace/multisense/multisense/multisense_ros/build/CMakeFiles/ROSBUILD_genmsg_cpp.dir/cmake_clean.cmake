FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/multisense_ros/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/multisense_ros/RawCamData.h"
  "../msg_gen/cpp/include/multisense_ros/SensorStatus.h"
  "../msg_gen/cpp/include/multisense_ros/RawLidarData.h"
  "../msg_gen/cpp/include/multisense_ros/SensorDiagnostics.h"
  "../msg_gen/cpp/include/multisense_ros/RawCamConfig.h"
  "../msg_gen/cpp/include/multisense_ros/JointDiagnostics.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
