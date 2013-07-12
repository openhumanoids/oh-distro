FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/multisense_ros/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/multisense_ros/CameraConfig.h"
  "../docs/CameraConfig.dox"
  "../docs/CameraConfig-usage.dox"
  "../src/multisense_ros/cfg/CameraConfig.py"
  "../docs/CameraConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
