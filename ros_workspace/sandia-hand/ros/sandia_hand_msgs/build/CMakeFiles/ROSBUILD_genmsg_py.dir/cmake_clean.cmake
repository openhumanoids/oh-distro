FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sandia_hand_msgs/msg"
  "../src/sandia_hand_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/sandia_hand_msgs/msg/__init__.py"
  "../src/sandia_hand_msgs/msg/_RawTactile.py"
  "../src/sandia_hand_msgs/msg/_RawFingerInertial.py"
  "../src/sandia_hand_msgs/msg/_RawFingerState.py"
  "../src/sandia_hand_msgs/msg/_RawMoboState.py"
  "../src/sandia_hand_msgs/msg/_CalFingerState.py"
  "../src/sandia_hand_msgs/msg/_RawPalmState.py"
  "../src/sandia_hand_msgs/msg/_RelativeJointCommands.py"
  "../src/sandia_hand_msgs/msg/_SimpleGrasp.py"
  "../src/sandia_hand_msgs/msg/_RawFingerCommands.py"
  "../src/sandia_hand_msgs/msg/_Parameter.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
