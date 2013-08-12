FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/handle_msgs/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/handle_msgs/msg/__init__.py"
  "../src/handle_msgs/msg/_HandleControl.py"
  "../src/handle_msgs/msg/_HandleSensorsCalibrated.py"
  "../src/handle_msgs/msg/_HandleCollisions.py"
  "../src/handle_msgs/msg/_CableTension.py"
  "../src/handle_msgs/msg/_Collision.py"
  "../src/handle_msgs/msg/_HandleSensors.py"
  "../src/handle_msgs/msg/_Finger.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
