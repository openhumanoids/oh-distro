FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/multisense_ros/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/multisense_ros/msg/__init__.py"
  "../src/multisense_ros/msg/_RawCamData.py"
  "../src/multisense_ros/msg/_SensorStatus.py"
  "../src/multisense_ros/msg/_RawLidarData.py"
  "../src/multisense_ros/msg/_SensorDiagnostics.py"
  "../src/multisense_ros/msg/_RawCamConfig.py"
  "../src/multisense_ros/msg/_JointDiagnostics.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
