FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/sandia_hand_msgs/msg"
  "../src/sandia_hand_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/sandia_hand_msgs/srv/__init__.py"
  "../src/sandia_hand_msgs/srv/_SimpleGraspSrv.py"
  "../src/sandia_hand_msgs/srv/_SetFingerHome.py"
  "../src/sandia_hand_msgs/srv/_GetParameters.py"
  "../src/sandia_hand_msgs/srv/_SetJointLimitPolicy.py"
  "../src/sandia_hand_msgs/srv/_SetParameters.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
